# ncode-lp
Wrapper for linear programming solvers.

This library wraps either GLPK or CPLEX with an easy-to-use C++ programming
interface. In addition it can compute maximum flow through a network and solve
basic multi-commodity flow problems.

To compile you will need either CPLEX or GLPK. GLPK is free and can be installed
from the glpk-dev package on Ubuntu, but it can be an order of magnitude slower
than the commercial CPLEX. Just like with any other CMake project, create a
build directory and run `ccmake ../` from there. If you have CPLEX set
CPLEX_INCLUDE_DIR and CPLEX_LIBRARY, for GLPK set GLPK_INCLUDE_DIR and
GLPK_LIBRARY. There is a CMake module that will try to automatically figure out
what is installed on the system and populate the appropriate variables, but it
sometimes fails to properly detect CPLEX and manual intervention may be needed.

## Linear programming API

There are two classes that can be used to create and solve LP problems, they are
both in `lp.h`.  The following example maximizes 5 * x1 + 6 * x2 subject to the
0 <= x1 + x2 <= 10, 3 <= x1 - x2 and 5 * x1 + 4 * x2 <= 35 constraints.

```c++
#include "lp.h"
using namespace nc::lp;

Problem problem(MAXIMIZE);
VariableIndex x1 = problem.AddVariable();
VariableIndex x2 = problem.AddVariable();

problem.SetVariableRange(x1, 0, Problem::kInifinity);
problem.SetVariableRange(x2, 0, Problem::kInifinity);

problem.SetObjectiveOffset(1000);
problem.SetObjectiveCoefficient(x1, 5);
problem.SetObjectiveCoefficient(x2, 6);

ConstraintIndex c1 = problem.AddConstraint();
problem.SetConstraintRange(c1, 0, 10);

ConstraintIndex c2 = problem.AddConstraint();
problem.SetConstraintRange(c2, 3, Problem::kInifinity);

ConstraintIndex c3 = problem.AddConstraint();
problem.SetConstraintRange(c3, Problem::kNegativeInifinity, 35);

std::vector<ProblemMatrixElement> matrix_elements = {
    {c1, x1, 1},  {c1, x2, 1}, {c2, x1, 1},
    {c2, x2, -1}, {c3, x1, 5}, {c3, x2, 4}};
problem.SetMatrix(matrix_elements);

auto solution = problem.Solve();
```

After solving `solution->type()` can be used to figure out if a solution is
optimal, feasible or unfeasible. The actual values of the variables and the
objective can be accessed like so:

```c++
double x1 = solution->VariableValue(x1);
double x2 = solution->VariableValue(x2);
double obj = solution->ObjectiveValue();
```
