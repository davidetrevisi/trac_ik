# TRAC-IK for MoveIt 2

Numerical inverse kinematics for a single kinematic chain, packaged as a library and as a MoveIt 2 kinematics
plugin. This glossary pins down the vocabulary for joints, configurations and the IK query.

## Language

### Joints and configurations

**Chain**:
The ordered sequence of movable joints between the base frame and the tip frame that the solver operates on.
_Avoid_: group (MoveIt's planning group is a superset that may include fixed and out-of-chain joints)

**Active joint**:
A chain joint whose value the solver chooses.
_Avoid_: independent joint, free joint, DOF (a count, not a joint)

**Mimic joint**:
A chain joint whose value is fixed by another joint through a multiplier and an offset; the solver never chooses it.
_Avoid_: passive joint (the SRDF term means "not actuated", a different property), slave joint, follower
(say "its mimic joints" for the ones a given mimicked joint drives)

**Mimicked joint**:
The joint a mimic joint follows.
_Avoid_: parent joint (collides with parent link), master joint, driving joint

**Joint coupling**:
The relation that fixes a mimic joint's value: a mimicked joint, a multiplier and an offset. Distinct from the
mimic joint, which is the joint the relation is attached to.
_Avoid_: mimic (ambiguous with the joint), constraint (suggests something the solver satisfies approximately)

**Effective bounds**:
The interval a joint's value may actually take once its couplings are accounted for: its own bounds intersected
with those of each of its mimic joints, mapped back through the coupling. Distinct from the joint's own bounds.
An unbounded joint's bound is an infinity; a joint with an infinite bound is unbounded, and there is no other
encoding of the fact.
_Avoid_: joint limits (ambiguous between the two), tightened limits, float-max sentinel (the old spelling of
an infinity)

**Reduced Jacobian**:
The 6 x (active joint count) map from reduced joint velocities to tip twist; the full chain Jacobian with each
mimic joint's column folded into its mimicked joint's, scaled by the multiplier. The coupled chain's actual
Jacobian, since no other velocity is commandable.
_Avoid_: folded Jacobian, mimic Jacobian, projected Jacobian

**Full configuration**:
One value per chain joint, in chain order, mimic joints included; the shape of seeds and solutions at the plugin
boundary and in MoveIt.
_Avoid_: joint state (a ROS message), joint positions

**Reduced configuration**:
One value per active joint; the solver's decision variables.
_Avoid_: active configuration, independent coordinates

**Solver joint list**:
The ordered names the plugin reports to MoveIt; every chain joint with a variable, mimic joints included.
_Avoid_: active joints, joint names (ambiguous)

**Joint distance**:
How far one configuration is from another: the Euclidean norm of the per-joint differences over the **active**
joints, with every joint weighted equally, so radians and metres are added directly — the same mixed-unit
convention as MoveIt's own group distance. A continuous joint is measured by the shortest wrap.
_Avoid_: error, cost, joint-space distance

### The IK query

**Seed**:
The full configuration a query starts from and measures joint distance against.
_Avoid_: initial guess, current state

**Tolerance bounds**:
Per-axis Cartesian slack (x, y, z, roll, pitch, yaw) inside which an error component counts as zero, measured in
the **goal frame** — the axes are the commanded pose's, not the base frame's. An infinite bound frees that axis.
_Avoid_: bounds (ambiguous with joint limits), epsilon, position-only flag (a special case of these)

**Freed axis**:
An axis whose tolerance bound is infinite. Distinct from a merely tolerated axis, whose bound is finite: a freed
axis is a degree of freedom the solver may spend elsewhere, while a tolerated one is still something the solver
aims at and only judges leniently.
_Avoid_: ignored axis, unconstrained axis (collides with MoveIt's goal constraints), don't-care

**Epsilon**:
The floor under the tolerance bounds: the residual each axis must reach once its tolerance has been applied, so
an axis is met at the looser of the two and a tolerance below epsilon has no effect.
_Avoid_: tolerance, accuracy

**Solve type**:
The rule that picks among the solutions collected within the timeout: Speed (first found), Distance (closest to
the seed), Manipulation1/2/3 (best conditioning of the reduced Jacobian).
_Avoid_: mode, strategy

**Query**:
Everything about one IK call that may differ from the next: timeout, epsilon, solve type, tolerance bounds and
the per-call joint bounds a caller may narrow the search to.
Distinct from the solver instance, which holds only what describes the mechanism (chain, bounds, couplings).
_Avoid_: options (collides with MoveIt's `KinematicsQueryOptions`), request, parameters (ambiguous with the
plugin's ROS parameters)

**Solver instance**:
One constructed `TRAC_IK` object. It serves one solve at a time; separate instances are fully independent.
_Avoid_: solver (ambiguous with the two inner solvers a `TRAC_IK` runs in parallel), session

**Candidate**:
One solution collected during a solve, before anything outside the solver has judged it. A solve yields a
candidate list ranked by the solve type; the winner is the first candidate the validity callback accepts.
_Avoid_: solution (reserve that for the one returned), result

**Validity callback**:
The predicate MoveIt hands the plugin to judge a candidate, typically a collision check. It is MoveIt's to
define and the plugin's to call, one candidate at a time, never from the solver threads.
_Avoid_: collision callback (collision is only its usual use), constraint (suggests the solver satisfies it)

**Consistency limits**:
A per-active-joint cap on how far a solution may move from the seed, supplied per query by the caller. Distinct
from effective bounds, which describe the mechanism and not one query.
_Avoid_: joint distance limits, redundancy limits

**Approximate solution**:
The best iterate a solve reached, returned only when the caller asks for it and only when the solve failed.
Not a candidate: a solve that fails collects no candidates at all, which is why an approximate solution has
to be tracked separately from them.
_Avoid_: partial solution, failed solution, best-effort, best candidate (a candidate met the tolerance
bounds; this did not)

**Best iterate**:
The configuration with the smallest Cartesian residual a branch has visited during one solve, measured after
the tolerance bounds are applied. Distinct from the branch's *last* iterate, which after a restart is a fresh
random configuration and says nothing about how close the solve got.
_Avoid_: best guess, closest solution (it is not a solution), best so far (ambiguous with the candidate
ranking)

**Stall exit**:
A solve ending before its timeout because nothing has improved for a set fraction of the budget. One rule with
two outcomes, told apart by whether a candidate exists: with none, the improving quantity is each branch's best
iterate and the solve reports no solution; with candidates in hand, it is the query's best joint distance and
the solve returns its best candidate (an *early exit*). Distinct from a timeout, which means the budget
genuinely ran out and more of it might have helped — a distinction the caller can see, because a stall exit
with no candidate reports no solution rather than a timeout.
_Avoid_: give-up, abort (the flag one branch sets on the other when it wins), early exit as a synonym (it names
only the success-side outcome, below)

**Early exit**:
The success-side outcome of a stall exit: the solve returns its best candidate, before its timeout, because
none better has appeared for a set fraction of the budget. Only solve types that rank a collected list can have
one — a solve type that stops at its first candidate has nothing left to wait for. What a caller observes is a
solution that arrived early, never a worse return code, and never a promise that no closer solution existed.
_Avoid_: early return, short-circuit, converged (nothing converged; the search stopped improving)

**Structural reject**:
Refusing a goal before any search, because no configuration of the mechanism could reach it. The only claim
the solver makes without searching, so it has to be provable rather than probable, and it has to respect the
query's tolerance bounds.
_Avoid_: workspace check, early rejection, reachability test (suggests a decision procedure; this one only
ever proves the negative)

**Retry**:
A fresh solve within the query's remaining time, from the same seed, after every candidate was rejected.
It needs unspent budget, which a solve type gets either by stopping at its first candidate or by ending in an
early exit.
_Avoid_: attempt (the old `kinematics_solver_attempts` parameter, which TRAC-IK does not use), reseed (the seed
does not change; only the solver's internal sampling does)

**Restart**:
The solver resampling its start configuration mid-search when a branch stalls, inside one solve. Distinct from
a retry, which is a fresh solve from the caller's unchanged seed.
_Avoid_: reseed (collides with the RNG seed), random restart and `rr` (the code's two spellings), jump

**RNG seed**:
The value that fixes a solver instance's random stream, so its restarts are replayable. Never bare "seed": that
is the query's start configuration. Fixing it makes the samples repeat, not the answer — the search is bounded
by wall-clock and runs two threads, so which candidate wins still varies.
_Avoid_: seed (reserved), random seed, determinism (the stream repeats; the result does not)

**Declined hook**:
A `KinematicsBase` entry point the plugin serves nothing for, answered with a logged, documented reason rather
than silence or a hard failure. Distinct from the load-time failures (a robot the fork refuses) and from a
per-call error code (a query that found no solution): the request is well-formed and we simply do not serve it.
_Avoid_: unsupported (says nothing about what the caller gets back), ignored option (silence is the thing this
convention replaces), stub

**The crane**:
The SPX532.2 test fixture: a 9-joint chain with 4 mimic joints and 5 active joints, whose mimicked joints sit
downstream of their mimics.
_Avoid_: the robot, the arm

### The benchmark

**Reachable target**:
A pose produced by forward kinematics of a full configuration that respects every joint coupling, so a solution
is known to exist before any solver runs. Distinct from an arbitrary pose, whose solvability is unknown and
which therefore measures nothing when a solve fails.
_Avoid_: valid pose, random pose, goal

**Sample**:
One benchmark trial: a reachable target paired with the seed the solve starts from. The sample set is generated
once per run and every solver under comparison sees the same one.
_Avoid_: test, trial, case, target (only half of it)

**Seed mode**:
The rule that produces a sample's seed from its target: `random` (an independent random configuration) or
`near` (a small perturbation of the target's own configuration, which is what a Cartesian waypoint hands the
solver). The choice changes what a timing means, so it is reported with every number.
_Avoid_: start mode, nominal seed (the legacy ROS 1 tool's fixed midpoint, no longer used)

**Goal class**:
Which of four rules made a sample's goal: `reachable` (forward kinematics of a real configuration, so a
solution exists), `edge` (pushed just past the reach measured in its own direction), `far` (well past any
reach) or `orient` (a reachable position with an arbitrary orientation). Only `reachable` and `far` are
certain either way; the rest are named by intent, and the forward-kinematics check is what decides what a
sample actually was.
_Avoid_: unreachable sample (three of the four classes only usually are), invalid goal, bad target

**Failure path**:
What a solve does when it cannot answer: what it costs, what it leaves in the output, and what it reports.
Measured separately from the solve rate, because a failing call and a succeeding one differ in cost by
orders of magnitude and averaging them hides both.
_Avoid_: failure case, error path (suggests something went wrong; nothing did)

**Solve rate**:
The fraction of a sample set for which the solver returned a solution that meets the query's tolerance bounds,
verified by forward kinematics rather than trusted from the return code.
_Avoid_: success rate, accuracy (reserve that for Cartesian error), hit rate
