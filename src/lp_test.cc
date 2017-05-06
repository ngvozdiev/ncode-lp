#include "lp.h"

#include <gtest/gtest.h>

namespace nc {
namespace lp {
namespace {

TEST(LP, EmptyProblem) {
  Problem problem(MINIMIZE);

  auto solution = problem.Solve();
  ASSERT_EQ(OPTIMAL, solution->type());
}

TEST(LP, UnboundedProblem) {
  Problem problem(MINIMIZE);
  VariableIndex a = problem.AddVariable();
  problem.SetObjectiveCoefficient(a, 10);
  problem.SetVariableRange(a, Problem::kNegativeInifinity, Problem::kInifinity);

  auto solution = problem.Solve();
  ASSERT_EQ(INFEASIBLE_OR_UNBOUNDED, solution->type());
}

TEST(LP, LPOne) {
  Problem problem(MINIMIZE);
  VariableIndex a = problem.AddVariable();
  VariableIndex b = problem.AddVariable();
  VariableIndex c = problem.AddVariable();
  problem.SetVariableRange(a, 0, Problem::kInifinity);
  problem.SetVariableRange(b, 0, Problem::kInifinity);
  problem.SetVariableRange(c, 0, Problem::kInifinity);

  problem.SetObjectiveCoefficient(a, 4);
  problem.SetObjectiveCoefficient(b, 5);
  problem.SetObjectiveCoefficient(c, 6);

  ConstraintIndex c1 = problem.AddConstraint();
  problem.SetConstraintRange(c1, 11, Problem::kInifinity);

  ConstraintIndex c2 = problem.AddConstraint();
  problem.SetConstraintRange(c2, Problem::kNegativeInifinity, 5);

  ConstraintIndex c3 = problem.AddConstraint();
  problem.SetConstraintRange(c3, 0, 0);

  ConstraintIndex c4 = problem.AddConstraint();
  problem.SetConstraintRange(c4, 35, Problem::kInifinity);

  std::vector<ProblemMatrixElement> matrix_elements = {
      {c1, a, 1},  {c1, b, 1},  {c2, a, 1}, {c2, b, -1}, {c3, c, 1},
      {c3, a, -1}, {c3, b, -1}, {c4, a, 7}, {c4, b, 12}};
  problem.SetMatrix(matrix_elements);

  auto solution = problem.Solve();
  ASSERT_EQ(OPTIMAL, solution->type());
  ASSERT_DOUBLE_EQ(8, solution->VariableValue(a));
  ASSERT_DOUBLE_EQ(3, solution->VariableValue(b));
  ASSERT_DOUBLE_EQ(11, solution->VariableValue(c));
  ASSERT_DOUBLE_EQ(113, solution->ObjectiveValue());
}

TEST(LP, LPTwo) {
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
  ASSERT_EQ(OPTIMAL, solution->type());

  // The fractions in the solution are periodic, will trim them to 2 places
  ASSERT_NEAR(5.222, solution->VariableValue(x1), 0.001);
  ASSERT_NEAR(2.222, solution->VariableValue(x2), 0.001);
  ASSERT_NEAR(1039.444, solution->ObjectiveValue(), 0.001);
}

TEST(LP, BinaryLP) {
  Problem problem(MINIMIZE);
  VariableIndex x1 = problem.AddVariable(true);
  VariableIndex x2 = problem.AddVariable(true);
  VariableIndex x3 = problem.AddVariable(true);
  VariableIndex x4 = problem.AddVariable(true);
  VariableIndex x5 = problem.AddVariable(true);
  VariableIndex x6 = problem.AddVariable(true);

  problem.SetObjectiveCoefficient(x1, 3);
  problem.SetObjectiveCoefficient(x2, 5);
  problem.SetObjectiveCoefficient(x3, 6);
  problem.SetObjectiveCoefficient(x4, 9);
  problem.SetObjectiveCoefficient(x5, 10);
  problem.SetObjectiveCoefficient(x6, 10);

  ConstraintIndex c1 = problem.AddConstraint();
  problem.SetConstraintRange(c1, 2, Problem::kInifinity);

  ConstraintIndex c2 = problem.AddConstraint();
  problem.SetConstraintRange(c2, -2, Problem::kInifinity);

  ConstraintIndex c3 = problem.AddConstraint();
  problem.SetConstraintRange(c3, 3, Problem::kInifinity);

  std::vector<ProblemMatrixElement> matrix_elements = {
      {c1, x1, -2}, {c1, x2, 6},  {c1, x3, -3}, {c1, x4, 4},  {c1, x5, 1},
      {c1, x6, -2}, {c2, x1, -5}, {c2, x2, -3}, {c2, x3, 1},  {c2, x4, 3},
      {c2, x5, -2}, {c2, x6, 1},  {c3, x1, 5},  {c3, x2, -1}, {c3, x3, 4},
      {c3, x4, -2}, {c3, x5, 2},  {c3, x6, -1}};
  problem.SetMatrix(matrix_elements);

  auto solution = problem.Solve();
  ASSERT_EQ(OPTIMAL, solution->type());

  ASSERT_EQ(0, solution->VariableValue(x1));
  ASSERT_EQ(1, solution->VariableValue(x2));
  ASSERT_EQ(1, solution->VariableValue(x3));
  ASSERT_EQ(0, solution->VariableValue(x4));
  ASSERT_EQ(0, solution->VariableValue(x5));
  ASSERT_EQ(0, solution->VariableValue(x6));
}

static std::unique_ptr<Problem> GetProblem(
    std::vector<VariableIndex>* variables) {
  auto problem = make_unique<Problem>(MAXIMIZE);
  size_t size = variables->size();
  CHECK(size);

  std::vector<ConstraintIndex> constraints(size);
  for (size_t i = 0; i < size; ++i) {
    VariableIndex variable = problem->AddVariable();
    problem->SetObjectiveCoefficient(variable, 1.0);
    problem->SetVariableRange(variable, Problem::kNegativeInifinity,
                              Problem::kInifinity);
    (*variables)[i] = variable;
  }
  for (size_t i = 0; i < size; ++i) {
    ConstraintIndex constraint = problem->AddConstraint();
    problem->SetConstraintRange(constraint, Problem::kNegativeInifinity, 10);
    constraints[i] = constraint;
  }

  std::vector<ProblemMatrixElement> matrix_elements;
  for (size_t i = 0; i < size; ++i) {
    matrix_elements.emplace_back(constraints[i], (*variables)[i], 5.0);
  }
  problem->SetMatrix(matrix_elements);

  return problem;
}

TEST(LP, LPLargeMatrix) {
  std::vector<VariableIndex> variables(1000);
  auto problem = GetProblem(&variables);

  auto solution = problem->Solve();
  ASSERT_EQ(OPTIMAL, solution->type());
  ASSERT_DOUBLE_EQ(1000 * 2.0, solution->ObjectiveValue());
  for (size_t i = 0; i < 1000; ++i) {
    ASSERT_EQ(2.0, solution->VariableValue(variables[i]));
  }
}

TEST(LP, LPLargeMatrixTimeout) {
  std::vector<VariableIndex> variables(200000);
  auto problem = GetProblem(&variables);

  auto solution = problem->Solve(std::chrono::milliseconds(50));
  ASSERT_TRUE(solution->timed_out());
}

}  // namespace
}  // namespace lp
}  // namespace ncode
