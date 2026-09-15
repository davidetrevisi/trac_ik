// ik_benchmark - the ROS 2 successor of trac_ik_examples/src/ik_tests.cpp.
//
// Measures solve rate and per-call cost for the solvers inside trac_ik_lib on a fixed sample set, so that
// changes to the solver math (ticket 12), the threading model (13) and Distance-mode early exit (14) can be
// compared across revisions on the same footing.
//
// Protocol (wayfinder ticket 11; do not change without reopening it):
//   - a *sample* is a reachable target (FK of a random coupling-consistent full configuration) plus a seed;
//   - every solver under comparison sees the same sample set, in per-solver blocks, with the first
//     --warmup samples discarded;
//   - one --rng-seed pins the sample set and derives the solvers' own stream;
//   - solve rate is verified by FK against the query's tolerance bounds, never taken from the return code,
//     and is reported beside the realisable rate: solved *and* every joint coupling actually satisfied.
//
// Written against the library as it stands today: joint couplings are handled *here* (the library treats
// mimic joints as independent, ticket 04), unbounded joints still use the float-max sentinel (ticket 08),
// and per-call parameters still arrive through the constructor (ticket 06). When those land, this tool
// changes with them; the protocol above does not.

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trac_ik/trac_ik.hpp>
#include <trac_ik/nlopt_ik.hpp>
#include <trac_ik/kdl_tl.hpp>

// Jazzy's urdf 2.10.1 has no <urdf/model.hpp>; on Kilted and Rolling <urdf/model.h> is a shim that
// warns. Delete this guard, keeping the .hpp, when Jazzy reaches EOL (May 2029).
#if __has_include(<urdf/model.hpp>)
#include <urdf/model.hpp>
#else
#include <urdf/model.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{

using Clock = std::chrono::steady_clock;

constexpr double kInf = std::numeric_limits<double>::infinity();

// ---------------------------------------------------------------------------------------------
// statistics
// ---------------------------------------------------------------------------------------------

struct Stats
{
  size_t n = 0;
  double median = 0, mean = 0, p90 = 0, p95 = 0, min = 0, max = 0;
};

Stats summarize(std::vector<double> v)
{
  Stats s;
  s.n = v.size();
  if (v.empty())
    return s;
  std::sort(v.begin(), v.end());
  const auto at = [&v](double q) { return v[static_cast<size_t>(q * (v.size() - 1))]; };
  s.median = at(0.50);
  s.mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
  s.p90 = at(0.90);
  s.p95 = at(0.95);
  s.min = v.front();
  s.max = v.back();
  return s;
}

double median_of(std::vector<double> v)
{
  return summarize(std::move(v)).median;
}

// ---------------------------------------------------------------------------------------------
// the mechanism
// ---------------------------------------------------------------------------------------------

// One joint coupling, in chain index space: mimic joint `mimic` follows `mimicked` by
// value = multiplier * value(mimicked) + offset.
struct Coupling
{
  unsigned int mimic = 0;
  unsigned int mimicked = 0;
  double multiplier = 1.0;
  double offset = 0.0;
};

std::string readFile(const std::string& path)
{
  std::ifstream in(path);
  if (!in.good())
  {
    std::fprintf(stderr, "cannot read %s\n", path.c_str());
    std::exit(2);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

struct Mechanism
{
  std::string urdf_path;
  std::string xml;
  urdf::Model model;
  KDL::Chain chain;
  KDL::JntArray lb, ub;
  std::vector<std::string> joint_names;   // chain order, movable joints only
  std::vector<Coupling> couplings;
  std::vector<unsigned int> active;       // chain indices that are not mimic joints

  Mechanism(const std::string& urdf_file, const std::string& base, const std::string& tip)
    : urdf_path(urdf_file)
  {
    xml = readFile(urdf_file);
    if (!model.initString(xml))
    {
      std::fprintf(stderr, "urdf parse failed: %s\n", urdf_file.c_str());
      std::exit(2);
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(xml, tree))
    {
      std::fprintf(stderr, "kdl_parser failed: %s\n", urdf_file.c_str());
      std::exit(2);
    }
    if (!tree.getChain(base, tip, chain))
    {
      std::fprintf(stderr, "no chain %s -> %s in %s\n", base.c_str(), tip.c_str(), urdf_file.c_str());
      std::exit(2);
    }

    const unsigned int n = chain.getNrOfJoints();
    lb.resize(n);
    ub.resize(n);
    unsigned int i = 0;
    for (const auto& seg : chain.segments)
    {
      const auto joint = model.getJoint(seg.getJoint().getName());
      if (!joint || joint->type == urdf::Joint::UNKNOWN || joint->type == urdf::Joint::FIXED)
        continue;
      joint_names.push_back(joint->name);
      if (joint->type == urdf::Joint::CONTINUOUS)
      {
        // The library's pre-ticket-08 spelling of "unbounded".
        lb(i) = std::numeric_limits<float>::lowest();
        ub(i) = std::numeric_limits<float>::max();
      }
      else
      {
        lb(i) = joint->limits->lower;
        ub(i) = joint->limits->upper;
      }
      ++i;
    }

    // Couplings, from the URDF: the library cannot see them yet (ticket 04).
    for (unsigned int k = 0; k < joint_names.size(); ++k)
    {
      const auto joint = model.getJoint(joint_names[k]);
      if (!joint || !joint->mimic)
        continue;
      const auto it = std::find(joint_names.begin(), joint_names.end(), joint->mimic->joint_name);
      if (it == joint_names.end())
      {
        std::fprintf(stderr, "mimic joint %s follows %s, which is outside the chain\n",
                     joint_names[k].c_str(), joint->mimic->joint_name.c_str());
        std::exit(2);
      }
      Coupling c;
      c.mimic = k;
      c.mimicked = static_cast<unsigned int>(std::distance(joint_names.begin(), it));
      c.multiplier = joint->mimic->multiplier;
      c.offset = joint->mimic->offset;
      couplings.push_back(c);
    }
    for (unsigned int k = 0; k < joint_names.size(); ++k)
      if (std::none_of(couplings.begin(), couplings.end(), [k](const Coupling& c) { return c.mimic == k; }))
        active.push_back(k);
  }

  bool unbounded(unsigned int i) const
  {
    return ub(i) >= std::numeric_limits<float>::max() && lb(i) <= std::numeric_limits<float>::lowest();
  }

  // Effective bounds (ticket 03): a joint's own bounds intersected with each of its mimics', mapped back
  // through the coupling. Sampling from these is what keeps a generated configuration inside every limit.
  void effectiveBounds(unsigned int i, double& lo, double& hi) const
  {
    lo = unbounded(i) ? -M_PI : lb(i);
    hi = unbounded(i) ? M_PI : ub(i);
    for (const auto& c : couplings)
    {
      if (c.mimicked != i || c.multiplier == 0.0)
        continue;
      double a = (lb(c.mimic) - c.offset) / c.multiplier;
      double b = (ub(c.mimic) - c.offset) / c.multiplier;
      if (a > b)
        std::swap(a, b);
      lo = std::max(lo, a);
      hi = std::min(hi, b);
    }
    if (lo > hi)
    {
      std::fprintf(stderr, "joint %s has empty effective bounds\n", joint_names[i].c_str());
      std::exit(2);
    }
  }

  void imposeCouplings(KDL::JntArray& q) const
  {
    for (const auto& c : couplings)
      q(c.mimic) = c.multiplier * q(c.mimicked) + c.offset;
  }

  KDL::JntArray randomConfig(std::mt19937& rng) const
  {
    KDL::JntArray q(chain.getNrOfJoints());
    for (const unsigned int i : active)
    {
      double lo, hi;
      effectiveBounds(i, lo, hi);
      q(i) = std::uniform_real_distribution<double>(lo, hi)(rng);
    }
    imposeCouplings(q);
    return q;
  }

  // A seed near a configuration: each active joint moved by up to `radius` of its effective range.
  KDL::JntArray nearConfig(const KDL::JntArray& q0, double radius, std::mt19937& rng) const
  {
    KDL::JntArray q(q0);
    for (const unsigned int i : active)
    {
      double lo, hi;
      effectiveBounds(i, lo, hi);
      const double d = radius * (hi - lo);
      q(i) = std::clamp(q0(i) + std::uniform_real_distribution<double>(-d, d)(rng), lo, hi);
    }
    imposeCouplings(q);
    return q;
  }

  double couplingViolation(const KDL::JntArray& q) const
  {
    double worst = 0;
    for (const auto& c : couplings)
      worst = std::max(worst, std::fabs(q(c.mimic) - (c.multiplier * q(c.mimicked) + c.offset)));
    return worst;
  }

  double jointDistance(const KDL::JntArray& a, const KDL::JntArray& b) const
  {
    double sum = 0;
    for (const unsigned int i : active)
      sum += (a(i) - b(i)) * (a(i) - b(i));
    return std::sqrt(sum);
  }

  size_t fixtureHash() const
  {
    return std::hash<std::string>{}(xml);
  }
};

// ---------------------------------------------------------------------------------------------
// the query
// ---------------------------------------------------------------------------------------------

struct Query
{
  double timeout = 0.05;
  double epsilon = 1e-5;
  TRAC_IK::SolveType solve_type = TRAC_IK::Speed;
  double tol[6] = { 0, 0, 0, 0, 0, 0 };  // x, y, z, rx, ry, rz - in the goal frame (ticket 09)

  KDL::Twist bounds() const
  {
    return KDL::Twist(KDL::Vector(tol[0], tol[1], tol[2]), KDL::Vector(tol[3], tol[4], tol[5]));
  }
};

struct Sample
{
  KDL::Frame target;
  KDL::JntArray seed;
};

struct Outcome
{
  int rc = -1;
  bool solved = false;
  // Solved *and* a configuration the mechanism can actually take: a solution that violates a coupling is
  // one MoveIt will overwrite from the mimicked joint, landing the tip somewhere else entirely (ticket 01).
  bool realisable = false;
  double us = 0, pos_err = 0, rot_err = 0, coupling = 0, joint_dist = 0;
};

// Solve rate is this function, not the return code: the solution's own FK must meet the query's
// tolerance bounds, floored by epsilon (ticket 09: acceptance is max(tolerance_i, epsilon)).
void verify(const Mechanism& m, const Query& q, const Sample& s, const KDL::JntArray& sol, Outcome& o)
{
  KDL::ChainFkSolverPos_recursive fk(m.chain);
  KDL::Frame f;
  if (fk.JntToCart(sol, f) < 0)
    return;
  const KDL::Twist e = KDL::diffRelative(s.target, f);
  o.pos_err = e.vel.Norm();
  o.rot_err = e.rot.Norm();
  o.coupling = m.couplingViolation(sol);
  o.joint_dist = m.jointDistance(s.seed, sol);
  bool ok = true;
  for (int i = 0; i < 6; ++i)
    ok = ok && std::fabs(e(i)) <= std::max(q.tol[i], q.epsilon);
  o.solved = ok;
  o.realisable = ok && o.coupling <= q.epsilon;
}

// ---------------------------------------------------------------------------------------------
// the solvers under comparison
// ---------------------------------------------------------------------------------------------

enum class Solver { KdlNrJl, KdlTl, Nlopt, TracIk };

const char* name_of(Solver s)
{
  switch (s)
  {
    case Solver::KdlNrJl: return "KDL NR_JL";
    case Solver::KdlTl: return "KDL-TL alone";
    case Solver::Nlopt: return "NLopt alone";
    case Solver::TracIk: return "TRAC-IK";
  }
  return "?";
}

// MoveIt's KDL plugin runs NR_JL with an iteration cap and no time budget, which is the configuration a
// user migrating away from it actually has; the ROS 1 tool's hand-rolled retry loop is deliberately gone.
constexpr int kNrJlMaxIter = 500;

std::vector<Outcome> run(Solver which, const Mechanism& m, const Query& q,
                         const std::vector<Sample>& samples, unsigned int solver_seed)
{
  const rclcpp::Logger logger = rclcpp::get_logger("ik_benchmark");

  // The library's restarts still come from the global rand() (ticket 10): seed it so a run replays.
  std::srand(solver_seed);

  KDL::ChainFkSolverPos_recursive fk(m.chain);
  KDL::ChainIkSolverVel_pinv vik(m.chain);
  KDL::ChainIkSolverPos_NR_JL nr_jl(m.chain, m.lb, m.ub, fk, vik, kNrJlMaxIter, q.epsilon);
  KDL::ChainIkSolverPos_TL kdl_tl(m.chain, m.lb, m.ub, q.timeout, q.epsilon, true, true);
  NLOPT_IK::NLOPT_IK nlopt(m.chain, m.lb, m.ub, q.timeout, q.epsilon, NLOPT_IK::SumSq, logger);
  TRAC_IK::TRAC_IK trac_ik(m.chain, m.lb, m.ub, q.timeout, q.epsilon, q.solve_type, logger);

  std::vector<Outcome> out;
  out.reserve(samples.size());
  for (const Sample& s : samples)
  {
    KDL::JntArray sol(m.chain.getNrOfJoints());
    Outcome o;
    const auto t0 = Clock::now();
    switch (which)
    {
      case Solver::KdlNrJl: o.rc = nr_jl.CartToJnt(s.seed, s.target, sol); break;
      case Solver::KdlTl: o.rc = kdl_tl.CartToJnt(s.seed, s.target, sol, q.bounds()); break;
      case Solver::Nlopt: o.rc = nlopt.CartToJnt(s.seed, s.target, sol, q.bounds()); break;
      case Solver::TracIk: o.rc = trac_ik.CartToJnt(s.seed, s.target, sol, q.bounds()); break;
    }
    o.us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
    if (o.rc >= 0)
      verify(m, q, s, sol, o);
    out.push_back(o);
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// reporting
// ---------------------------------------------------------------------------------------------

void reportHeader(const Mechanism& m, const Query& q, const std::string& base, const std::string& tip,
                  int samples, int warmup, const std::string& seed_mode, double near_radius,
                  unsigned int rng_seed, unsigned int solver_seed)
{
  const char* type_names[] = { "Speed", "Distance", "Manip1", "Manip2", "Manip3" };
  std::printf("\n=== %s (%s -> %s) ===\n", m.urdf_path.c_str(), base.c_str(), tip.c_str());
  std::printf("  revision      %s\n", IK_BENCHMARK_GIT_REV);
  std::printf("  fixture hash  %016zx\n", m.fixtureHash());
  std::printf("  chain         %u joints, %zu mimic, %zu active\n", m.chain.getNrOfJoints(),
              m.couplings.size(), m.active.size());
  std::printf("  samples       %d (+%d warmup discarded), seed mode %s", samples, warmup, seed_mode.c_str());
  if (seed_mode == "near")
    std::printf(" (radius %.3f of range)", near_radius);
  std::printf("\n");
  std::printf("  query         timeout %.4f s, epsilon %.1e, solve type %s\n", q.timeout, q.epsilon,
              type_names[static_cast<int>(q.solve_type)]);
  std::printf("  tolerances    [%g %g %g %g %g %g] (goal frame)\n", q.tol[0], q.tol[1], q.tol[2], q.tol[3],
              q.tol[4], q.tol[5]);
  std::printf("  rng seed      %u (sample set), %u (solver stream, derived)\n", rng_seed, solver_seed);
  std::printf("\n  %-14s %7s %8s  %10s %10s %10s %10s   %9s %9s %9s %9s\n", "solver", "solved",
              "realisab", "median us", "mean us", "p90 us", "p95 us", "pos m", "rot rad", "coupling",
              "jnt dist");
}

void reportRow(Solver which, const std::vector<Outcome>& outcomes)
{
  std::vector<double> us, pos, rot, dist;
  double worst_coupling = 0;
  size_t solved = 0, realisable = 0;
  for (const Outcome& o : outcomes)
  {
    us.push_back(o.us);
    if (!o.solved)
      continue;
    ++solved;
    realisable += o.realisable ? 1 : 0;
    pos.push_back(o.pos_err);
    rot.push_back(o.rot_err);
    dist.push_back(o.joint_dist);
    worst_coupling = std::max(worst_coupling, o.coupling);
  }
  const Stats t = summarize(us);
  const double n = outcomes.empty() ? 1.0 : static_cast<double>(outcomes.size());
  std::printf("  %-14s %6.1f%% %7.1f%%  %10.2f %10.2f %10.2f %10.2f   %9.2e %9.2e %9.2e %9.3f\n",
              name_of(which), 100.0 * solved / n, 100.0 * realisable / n, t.median, t.mean, t.p90, t.p95,
              median_of(pos), median_of(rot), worst_coupling, median_of(dist));
  std::fflush(stdout);
}

void writeCsvRows(std::FILE* csv, const std::string& fixture, const Query& q, const std::string& seed_mode,
                  Solver which, const std::vector<Outcome>& outcomes, int warmup)
{
  const char* type_names[] = { "Speed", "Distance", "Manip1", "Manip2", "Manip3" };
  for (size_t i = 0; i < outcomes.size(); ++i)
  {
    const Outcome& o = outcomes[i];
    std::fprintf(csv, "%s,%s,%s,%s,%zu,%d,%d,%d,%.3f,%.9g,%.9g,%.9g,%.9g\n", fixture.c_str(),
                 name_of(which), type_names[static_cast<int>(q.solve_type)], seed_mode.c_str(), i + warmup,
                 o.rc, o.solved ? 1 : 0, o.realisable ? 1 : 0, o.us, o.pos_err, o.rot_err, o.coupling,
                 o.joint_dist);
  }
}

// ---------------------------------------------------------------------------------------------
// micro mode (absorbed from ticket 06's throwaway bench_construction.cpp)
// ---------------------------------------------------------------------------------------------

void runMicro(const Mechanism& m, const Query& q, int reps)
{
  const rclcpp::Logger logger = rclcpp::get_logger("ik_benchmark");
  std::vector<double> whole, whole_dtor, nlo, tl, jac, threads;

  for (int i = 0; i < reps; ++i)
  {
    {
      const auto t0 = Clock::now();
      TRAC_IK::TRAC_IK ik(m.chain, m.lb, m.ub, q.timeout, q.epsilon, q.solve_type, logger);
      whole.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
    }
    {
      const auto t0 = Clock::now();
      {
        TRAC_IK::TRAC_IK ik(m.chain, m.lb, m.ub, q.timeout, q.epsilon, q.solve_type, logger);
      }
      whole_dtor.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
    }
    {
      const auto t0 = Clock::now();
      NLOPT_IK::NLOPT_IK n(m.chain, m.lb, m.ub, q.timeout, q.epsilon, NLOPT_IK::SumSq, logger);
      nlo.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
    }
    {
      const auto t0 = Clock::now();
      KDL::ChainIkSolverPos_TL s(m.chain, m.lb, m.ub, q.timeout, q.epsilon, true, true);
      tl.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
    }
    {
      const auto t0 = Clock::now();
      KDL::ChainJntToJacSolver s(m.chain);
      jac.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
    }
    {
      const auto t0 = Clock::now();
      std::thread a([] {});
      std::thread b([] {});
      a.join();
      b.join();
      threads.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
    }
  }

  std::printf("\n  %-40s %6s %10s %10s %10s %10s\n", "micro", "n", "median us", "mean us", "p90 us", "p95 us");
  const auto row = [](const char* label, const Stats& s) {
    std::printf("  %-40s %6zu %10.2f %10.2f %10.2f %10.2f\n", label, s.n, s.median, s.mean, s.p90, s.p95);
  };
  row("TRAC_IK ctor", summarize(whole));
  row("TRAC_IK ctor+dtor", summarize(whole_dtor));
  row("NLOPT_IK ctor", summarize(nlo));
  row("ChainIkSolverPos_TL ctor", summarize(tl));
  row("ChainJntToJacSolver ctor", summarize(jac));
  row("2x std::thread create+join (empty)", summarize(threads));
  std::fflush(stdout);
}

// ---------------------------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------------------------

struct FixtureSpec
{
  std::string name, urdf, base, tip;
  bool position_only = false;  // baked-in query, per ticket 09: the crane ships position-only
};

const std::vector<FixtureSpec>& defaultFixtures()
{
  static const std::vector<FixtureSpec> v = {
    { "crane", std::string(IK_BENCHMARK_FIXTURE_DIR) + "/crane.urdf", "base_link", "jib_ext_link", true },
    { "arm6", std::string(IK_BENCHMARK_FIXTURE_DIR) + "/arm6.urdf", "base_link", "tool_link", false },
    { "arm7", std::string(IK_BENCHMARK_FIXTURE_DIR) + "/arm7.urdf", "base_link", "tool_link", false },
  };
  return v;
}

void usage()
{
  std::printf(
    "ik_benchmark - solve rate and per-call cost for the solvers in trac_ik_lib\n\n"
    "  --fixture NAME        crane | arm6 | arm7 (default: all)\n"
    "  --urdf FILE           benchmark another mechanism (requires --base and --tip)\n"
    "  --base LINK --tip LINK\n"
    "  --samples N           measured samples per solver (default 1000)\n"
    "  --warmup N            samples discarded before measuring (default 20)\n"
    "  --timeout S           per-solve budget in seconds (default 0.05, MoveIt's default)\n"
    "  --epsilon E           residual floor (default 1e-5)\n"
    "  --solve-type T        Speed | Distance | Manip1 | Manip2 | Manip3 (default Speed)\n"
    "  --tolerances a,b,c,d,e,f   per-axis slack in the goal frame; 'inf' frees an axis\n"
    "  --position-only       shorthand for --tolerances 0,0,0,inf,inf,inf\n"
    "  --full-pose           override a fixture's baked-in tolerances\n"
    "  --seed-mode MODE      random | near (default random)\n"
    "  --near-radius F       'near' perturbation, fraction of each joint's range (default 0.05)\n"
    "  --solvers LIST        comma list of kdl_nr_jl,kdl_tl,nlopt,trac_ik (default all)\n"
    "  --rng-seed N          pins the sample set and derives the solver stream (default 1)\n"
    "  --mode MODE           solve | micro (default solve)\n"
    "  --micro-reps N        repetitions in micro mode (default 200)\n"
    "  --csv FILE            one row per sample per solver\n");
}

double parseTol(const std::string& s)
{
  if (s == "inf" || s == "+inf" || s == ".inf")
    return kInf;
  return std::stod(s);
}

std::vector<std::string> split(const std::string& s, char sep)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, sep))
    out.push_back(item);
  return out;
}

}  // namespace

int main(int argc, char** argv)
{
  std::string fixture, urdf, base, tip, seed_mode = "random", mode = "solve", csv_path;
  std::string solver_list = "kdl_nr_jl,kdl_tl,nlopt,trac_ik";
  int samples = 1000, warmup = 20, micro_reps = 200;
  unsigned int rng_seed = 1;
  double near_radius = 0.05;
  Query query;
  bool tol_given = false, full_pose = false;

  for (int i = 1; i < argc; ++i)
  {
    const std::string a = argv[i];
    const auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc)
      {
        std::fprintf(stderr, "%s needs a value\n", what);
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--help" || a == "-h") { usage(); return 0; }
    else if (a == "--fixture") fixture = next("--fixture");
    else if (a == "--urdf") urdf = next("--urdf");
    else if (a == "--base") base = next("--base");
    else if (a == "--tip") tip = next("--tip");
    else if (a == "--samples") samples = std::stoi(next("--samples"));
    else if (a == "--warmup") warmup = std::stoi(next("--warmup"));
    else if (a == "--timeout") query.timeout = std::stod(next("--timeout"));
    else if (a == "--epsilon") query.epsilon = std::stod(next("--epsilon"));
    else if (a == "--micro-reps") micro_reps = std::stoi(next("--micro-reps"));
    else if (a == "--near-radius") near_radius = std::stod(next("--near-radius"));
    else if (a == "--rng-seed") rng_seed = static_cast<unsigned int>(std::stoul(next("--rng-seed")));
    else if (a == "--seed-mode") seed_mode = next("--seed-mode");
    else if (a == "--mode") mode = next("--mode");
    else if (a == "--csv") csv_path = next("--csv");
    else if (a == "--solvers") solver_list = next("--solvers");
    else if (a == "--full-pose") full_pose = true;
    else if (a == "--position-only")
    {
      query.tol[0] = query.tol[1] = query.tol[2] = 0;
      query.tol[3] = query.tol[4] = query.tol[5] = kInf;
      tol_given = true;
    }
    else if (a == "--tolerances")
    {
      const auto parts = split(next("--tolerances"), ',');
      if (parts.size() != 6)
      {
        std::fprintf(stderr, "--tolerances needs six comma-separated values\n");
        return 2;
      }
      for (int k = 0; k < 6; ++k)
        query.tol[k] = parseTol(parts[k]);
      tol_given = true;
    }
    else if (a == "--solve-type")
    {
      const std::string t = next("--solve-type");
      if (t == "Speed") query.solve_type = TRAC_IK::Speed;
      else if (t == "Distance") query.solve_type = TRAC_IK::Distance;
      else if (t == "Manip1") query.solve_type = TRAC_IK::Manip1;
      else if (t == "Manip2") query.solve_type = TRAC_IK::Manip2;
      else if (t == "Manip3") query.solve_type = TRAC_IK::Manip3;
      else { std::fprintf(stderr, "unknown solve type %s\n", t.c_str()); return 2; }
    }
    else
    {
      std::fprintf(stderr, "unknown argument %s\n", a.c_str());
      usage();
      return 2;
    }
  }

  if (seed_mode != "random" && seed_mode != "near")
  {
    std::fprintf(stderr, "--seed-mode must be random or near\n");
    return 2;
  }
  if (!urdf.empty() && (base.empty() || tip.empty()))
  {
    std::fprintf(stderr, "--urdf requires --base and --tip\n");
    return 2;
  }

  std::vector<Solver> solvers;
  for (const auto& s : split(solver_list, ','))
  {
    if (s == "kdl_nr_jl") solvers.push_back(Solver::KdlNrJl);
    else if (s == "kdl_tl") solvers.push_back(Solver::KdlTl);
    else if (s == "nlopt") solvers.push_back(Solver::Nlopt);
    else if (s == "trac_ik") solvers.push_back(Solver::TracIk);
    else { std::fprintf(stderr, "unknown solver %s\n", s.c_str()); return 2; }
  }

  std::vector<FixtureSpec> to_run;
  if (!urdf.empty())
    to_run.push_back({ urdf, urdf, base, tip, false });
  else
    for (const auto& f : defaultFixtures())
      if (fixture.empty() || fixture == f.name)
        to_run.push_back(f);
  if (to_run.empty())
  {
    std::fprintf(stderr, "no such fixture: %s\n", fixture.c_str());
    return 2;
  }

  std::FILE* csv = nullptr;
  if (!csv_path.empty())
  {
    csv = std::fopen(csv_path.c_str(), "w");
    if (!csv)
    {
      std::fprintf(stderr, "cannot write %s\n", csv_path.c_str());
      return 2;
    }
    std::fprintf(csv, "fixture,solver,solve_type,seed_mode,sample,rc,solved,realisable,elapsed_us,"
                      "pos_err_m,rot_err_rad,coupling_violation,joint_distance\n");
  }

  for (const FixtureSpec& spec : to_run)
  {
    Mechanism m(spec.urdf, spec.base, spec.tip);
    Query q = query;
    if (!tol_given && !full_pose && spec.position_only)
    {
      q.tol[3] = q.tol[4] = q.tol[5] = kInf;  // ticket 09: the crane's shipped configuration
    }

    // One sample set, shared by every solver. The solver stream is derived from the same seed so a run
    // replays whole, and so that changing --solvers cannot change the samples.
    std::mt19937 rng(rng_seed);
    // Knuth's multiplicative hash: one --rng-seed, two independent streams.
    const unsigned int derived = rng_seed * 2654435761u + 1u;

    const size_t total = static_cast<size_t>(samples) + static_cast<size_t>(warmup);
    std::vector<Sample> set;
    set.reserve(total);
    for (size_t i = 0; i < total; ++i)
    {
      const KDL::JntArray q_target = m.randomConfig(rng);
      KDL::ChainFkSolverPos_recursive fk(m.chain);
      Sample s;
      if (fk.JntToCart(q_target, s.target) < 0)
      {
        std::fprintf(stderr, "FK failed while generating a sample\n");
        return 2;
      }
      s.seed = (seed_mode == "near") ? m.nearConfig(q_target, near_radius, rng) : m.randomConfig(rng);
      set.push_back(s);
    }

    reportHeader(m, q, spec.base, spec.tip, samples, warmup, seed_mode, near_radius, rng_seed, derived);

    for (const Solver which : solvers)
    {
      std::vector<Outcome> outcomes = run(which, m, q, set, derived);
      outcomes.erase(outcomes.begin(), outcomes.begin() + std::min<size_t>(warmup, outcomes.size()));
      reportRow(which, outcomes);
      if (csv)
        writeCsvRows(csv, spec.name, q, seed_mode, which, outcomes, warmup);
    }

    if (mode == "micro")
      runMicro(m, q, micro_reps);
    else if (mode != "solve")
    {
      std::fprintf(stderr, "--mode must be solve or micro\n");
      return 2;
    }
  }

  if (csv)
    std::fclose(csv);
  std::printf("\n");
  return 0;
}
