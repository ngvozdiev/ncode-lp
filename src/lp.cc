#include "lp.h"
#include "ncode_config.h"

#include "ncode_common/src/common.h"
#include "ncode_common/src/logging.h"
#include "ncode_common/src/map_util.h"

#ifdef LP_SOLVER_CPLEX
#include <ilcplex/cplex.h>
#include <set>
#endif

#ifdef LP_SOLVER_GLPK
#include <glpk.h>
#endif

namespace nc {
namespace lp {

#ifdef LP_SOLVER_CPLEX
struct CPLEXHandle {
  CPLEXHandle(Direction direction) : obj_offset(0), direction(direction) {}

  // Variable state
  std::vector<double> variable_lb;       // lower bounds
  std::vector<double> variable_ub;       // upper bounds
  std::vector<double> obj_coefficients;  // corresponding objective coefficients

  // The set of variables that can be either 0 or 1.
  std::set<VariableIndex> binary_variables;

  // Constraint state
  std::vector<double> rhs;       // the right hand side values
  std::vector<double> rangeval;  // if a constraint is ranged, it can take
                                 // values between rhs and rhs + rangeval.
  std::vector<char> sense;       // one of 'LEGR'

  // The problem matrix coefficients
  std::vector<ProblemMatrixElement> matrix_elements;

  // Constant-term offset of the objective function.
  double obj_offset;

  // The direction of optimization.
  Direction direction;
};

static std::pair<double, double> HandleInifinities(double min, double max) {
  if (min == Problem::kNegativeInifinity) {
    min = -CPX_INFBOUND;
  }

  if (max == Problem::kInifinity) {
    max = CPX_INFBOUND;
  }

  return std::make_pair(min, max);
}

Problem::Problem(Direction direction) : has_binary_variables_(false) {
  CPLEXHandle* handle = new CPLEXHandle(direction);
  handle_ = handle;
}

Problem::~Problem() {
  CPLEXHandle* handle = static_cast<CPLEXHandle*>(handle_);
  delete handle;
}

ConstraintIndex Problem::AddConstraint() {
  CPLEXHandle* handle = static_cast<CPLEXHandle*>(handle_);
  size_t num_constraints = handle->rhs.size();
  ConstraintIndex return_index(num_constraints);

  handle->rhs.emplace_back(0);
  handle->rangeval.emplace_back(0);
  handle->sense.emplace_back('E');
  return return_index;
}

void Problem::SetConstraintName(ConstraintIndex constraint,
                                const std::string& name) {
  Unused(constraint);
  Unused(name);
  LOG(INFO) << "Not implemented yet";
}

VariableIndex Problem::AddVariable(bool binary) {
  CPLEXHandle* handle = static_cast<CPLEXHandle*>(handle_);
  size_t num_variables = handle->variable_lb.size();
  VariableIndex return_index(num_variables);

  handle->variable_lb.emplace_back(0);
  handle->obj_coefficients.emplace_back(0);

  if (binary) {
    handle->binary_variables.insert(return_index);
    handle->variable_ub.emplace_back(1.0);
  } else {
    handle->variable_ub.emplace_back(0);
  }

  return return_index;
}

void Problem::SetVariableName(VariableIndex variable, const std::string& name) {
  Unused(variable);
  Unused(name);
  LOG(INFO) << "Not implemented yet";
}

void Problem::SetMatrix(
    const std::vector<ProblemMatrixElement>& matrix_elements) {
  CPLEXHandle* handle = static_cast<CPLEXHandle*>(handle_);
  handle->matrix_elements = matrix_elements;
}

void Problem::SetConstraintRange(ConstraintIndex constraint, double min,
                                 double max) {
  CPLEXHandle* handle = static_cast<CPLEXHandle*>(handle_);
  if (min == Problem::kNegativeInifinity &&
      max == Problem::kNegativeInifinity) {
    handle->rhs[constraint] = -CPX_INFBOUND;
    handle->rangeval[constraint] = 2 * CPX_INFBOUND;
    handle->sense[constraint] = 'R';
  } else if (min == max) {
    handle->rangeval[constraint] = 0;
    handle->rhs[constraint] = min;
    handle->sense[constraint] = 'E';
  } else if (min == Problem::kNegativeInifinity) {
    handle->rangeval[constraint] = 0;
    handle->rhs[constraint] = max;
    handle->sense[constraint] = 'L';
  } else if (max == Problem::kInifinity) {
    handle->rangeval[constraint] = 0;
    handle->rhs[constraint] = min;
    handle->sense[constraint] = 'G';
  } else {
    handle->rangeval[constraint] = max - min;
    handle->rhs[constraint] = min;
    handle->sense[constraint] = 'R';
  }
}

void Problem::SetVariableRange(VariableIndex variable, double min, double max) {
  CPLEXHandle* handle = static_cast<CPLEXHandle*>(handle_);

  CHECK(!ContainsKey(handle->binary_variables, variable))
      << "Tried to set bounds for binary variable";

  double new_min, new_max;
  std::tie(new_min, new_max) = HandleInifinities(min, max);
  handle->variable_lb[variable] = new_min;
  handle->variable_ub[variable] = new_max;
}

void Problem::SetObjectiveCoefficient(VariableIndex variable, double value) {
  CPLEXHandle* handle = static_cast<CPLEXHandle*>(handle_);
  handle->obj_coefficients[variable] = value;
}

void Problem::SetObjectiveOffset(double value) {
  CPLEXHandle* handle = static_cast<CPLEXHandle*>(handle_);
  handle->obj_offset = value;
}

static std::pair<CPXENVptr, CPXLPptr> GetProblem(const CPLEXHandle& handle) {
  // Have to construct vectors of row/col/value for non-zero matrix elements.
  size_t nonzero_count = handle.matrix_elements.size();
  std::vector<int> rowlist(nonzero_count);
  std::vector<int> collist(nonzero_count);
  std::vector<double> vallist(nonzero_count);

  for (size_t i = 0; i < nonzero_count; ++i) {
    const ProblemMatrixElement& element = handle.matrix_elements[i];
    rowlist[i] = element.constraint;
    collist[i] = element.variable;
    vallist[i] = element.value;
  }

  int status;
  CPXENVptr env = CPXopenCPLEX(&status);
  if (env == nullptr) {
    char errmsg[CPXMESSAGEBUFSIZE];
    CPXgeterrorstring(env, status, errmsg);
    LOG(ERROR) << "Could not open CPLEX environment: " << errmsg;
    return std::make_pair(nullptr, nullptr);
  }

  CPXLPptr lp = CPXcreateprob(env, &status, "LP");
  if (lp == nullptr) {
    char errmsg[CPXMESSAGEBUFSIZE];
    CPXgeterrorstring(env, status, errmsg);
    LOG(ERROR) << "Failed to create LP: " << errmsg;
    CPXcloseCPLEX(&env);
    return std::make_pair(nullptr, nullptr);
  }

  if (handle.direction == MAXIMIZE) {
    CPXchgobjsen(env, lp, CPX_MAX);
  } else {
    CPXchgobjsen(env, lp, CPX_MIN);
  }

  if (CPXnewrows(env, lp, handle.rhs.size(), handle.rhs.data(),
                 handle.sense.data(), handle.rangeval.data(), nullptr)) {
    LOG(ERROR) << "Unable to set rows!";
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);
    return std::make_pair(nullptr, nullptr);
  }

  // Will construct a vector with 'C' if the variable is continuous or 'B' if
  // binary.
  size_t max_index = handle.variable_lb.size();
  std::vector<char> col_types(max_index);
  for (size_t i = 0; i < max_index; ++i) {
    VariableIndex variable_index(i);
    if (ContainsKey(handle.binary_variables, variable_index)) {
      col_types[i] = CPX_BINARY;
    } else {
      col_types[i] = CPX_CONTINUOUS;
    }
  }

  if (CPXnewcols(env, lp, handle.variable_lb.size(),
                 handle.obj_coefficients.data(), handle.variable_lb.data(),
                 handle.variable_ub.data(),
                 handle.binary_variables.empty() ? nullptr : col_types.data(),
                 nullptr)) {
    LOG(ERROR) << "Unable to set columns!";
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);
    return std::make_pair(nullptr, nullptr);
  }

  if (CPXchgcoeflist(env, lp, nonzero_count, rowlist.data(), collist.data(),
                     vallist.data())) {
    LOG(ERROR) << "Unable to set matrix!";
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);
    return std::make_pair(nullptr, nullptr);
  }

  return std::make_pair(env, lp);
}

std::unique_ptr<Solution> Problem::Solve(std::chrono::milliseconds time_limit) {
  CPLEXHandle* handle = static_cast<CPLEXHandle*>(handle_);
  auto solution = std::unique_ptr<Solution>(new Solution());
  solution->solution_type_ = INFEASIBLE_OR_UNBOUNDED;

  CPXENVptr env;
  CPXLPptr lp;
  std::tie(env, lp) = GetProblem(*handle);
  if (env == nullptr || lp == nullptr) {
    return solution;
  }

  if (time_limit != std::chrono::milliseconds::max()) {
    typedef std::chrono::duration<double> DoubleSeconds;
    double time_sec =
        std::chrono::duration_cast<DoubleSeconds>(time_limit).count();
    CHECK(CPXsetdblparam(env, CPX_PARAM_TILIM, time_sec) == 0);
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  int status;
  if (handle->binary_variables.empty()) {
    status = CPXlpopt(env, lp);
  } else {
    status = CPXmipopt(env, lp);
  }

  auto done_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      done_time - start_time);
  if (duration > time_limit) {
    LOG(ERROR) << "Timed out after " << duration.count() << "ms";
    solution->timed_out_ = true;
  }

  if (status) {
    char errmsg[CPXMESSAGEBUFSIZE];
    CPXgeterrorstring(env, status, errmsg);
    LOG(ERROR) << "Failed to optimize LP: " << errmsg;
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);
    return solution;
  }

  size_t cur_numcols = CPXgetnumcols(env, lp);
  std::vector<double> x(cur_numcols);
  double obj_value;

  int solstat = 0;
  status = CPXsolution(env, lp, &solstat, &obj_value, x.data(), nullptr,
                       nullptr, nullptr);
  if (status) {
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);
    return solution;
  }

  if (solstat == CPX_STAT_OPTIMAL || solstat == CPXMIP_OPTIMAL ||
      solstat == CPXMIP_OPTIMAL_TOL) {
    solution->solution_type_ = SolutionType::OPTIMAL;
  } else if (solstat == CPX_STAT_FEASIBLE || solstat == CPXMIP_FEASIBLE) {
    solution->solution_type_ = SolutionType::FEASIBLE;
  } else {
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);
    return solution;
  }

  solution->variables_ = std::move(x);
  solution->objective_value_ = obj_value + handle->obj_offset;

  CPXfreeprob(env, &lp);
  CPXcloseCPLEX(&env);
  return solution;
}

void Problem::DumpToFile(const std::string& file) {
  CPLEXHandle* handle = static_cast<CPLEXHandle*>(handle_);

  CPXENVptr env;
  CPXLPptr lp;
  std::tie(env, lp) = GetProblem(*handle);
  if (env == nullptr || lp == nullptr) {
    return;
  }

  CPXwriteprob(env, lp, file.c_str(), "LP");
  CPXfreeprob(env, &lp);
  CPXcloseCPLEX(&env);
}

#endif

#ifdef LP_SOLVER_GLPK
static constexpr uint32_t kInitialNumRows = 100;
static constexpr uint32_t kInitialNumCols = 100;
struct GLPKHandle {
  glp_prob* lp = nullptr;
  size_t num_rows = 0;
  size_t num_cols = 0;
};

static int GetBoundType(double min, double max) {
  int type;
  if (min == Problem::kNegativeInifinity && max == Problem::kInifinity) {
    type = GLP_FR;
  } else if (min == max) {
    type = GLP_FX;
  } else if (min == Problem::kNegativeInifinity) {
    type = GLP_UP;
  } else if (max == Problem::kInifinity) {
    type = GLP_LO;
  } else {
    type = GLP_DB;
  }

  return type;
}

Problem::Problem(Direction direction) : has_binary_variables_(false) {
  glp_prob* lp = glp_create_prob();
  if (direction == MAXIMIZE) {
    glp_set_obj_dir(lp, GLP_MAX);
  } else {
    glp_set_obj_dir(lp, GLP_MIN);
  }
  glp_add_rows(lp, kInitialNumRows);
  glp_add_cols(lp, kInitialNumCols);

  GLPKHandle* handle = new GLPKHandle();
  handle->lp = lp;
  handle_ = handle;
}

Problem::~Problem() {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  glp_delete_prob(handle->lp);
  delete handle;
}

ConstraintIndex Problem::AddConstraint() {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  size_t rows_in_problem = glp_get_num_rows(handle->lp);
  if (rows_in_problem == handle->num_rows) {
    glp_add_rows(handle->lp, kInitialNumRows);
  }

  ConstraintIndex return_index(handle->num_rows);
  ++handle->num_rows;
  return return_index;
}

void Problem::SetConstraintName(ConstraintIndex constraint,
                                const std::string& name) {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  glp_set_row_name(handle->lp, constraint + 1, name.c_str());
}

VariableIndex Problem::AddVariable(bool binary) {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  size_t cols_in_problem = glp_get_num_cols(handle->lp);
  if (cols_in_problem == handle->num_cols) {
    glp_add_cols(handle->lp, kInitialNumCols);
  }

  if (binary) {
    glp_set_col_kind(handle->lp, handle->num_cols + 1, GLP_BV);
    has_binary_variables_ = true;
  }

  VariableIndex return_index(handle->num_cols);
  ++handle->num_cols;
  return return_index;
}

void Problem::SetVariableName(VariableIndex variable, const std::string& name) {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  glp_set_col_name(handle->lp, variable + 1, name.c_str());
}

void Problem::SetMatrix(
    const std::vector<ProblemMatrixElement>& matrix_elements) {
  std::vector<int> row_indices;
  std::vector<int> col_indices;
  std::vector<double> values;

  row_indices.resize(matrix_elements.size() + 1);
  col_indices.resize(matrix_elements.size() + 1);
  values.resize(matrix_elements.size() + 1);

  for (size_t element_index = 0; element_index < matrix_elements.size();
       ++element_index) {
    const ProblemMatrixElement& element = matrix_elements[element_index];
    row_indices[element_index + 1] = element.constraint + 1;
    col_indices[element_index + 1] = element.variable + 1;
    values[element_index + 1] = element.value;
  }

  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  glp_load_matrix(handle->lp, matrix_elements.size(), row_indices.data(),
                  col_indices.data(), values.data());
}

void Problem::SetConstraintRange(ConstraintIndex constraint_index, double min,
                                 double max) {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  glp_set_row_bnds(handle->lp, constraint_index + 1, GetBoundType(min, max),
                   min, max);
}

void Problem::SetVariableRange(VariableIndex variable_index, double min,
                               double max) {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  CHECK(glp_get_col_kind(handle->lp, variable_index + 1) != GLP_BV)
      << "Tried to set bounds for binary variable";

  glp_set_col_bnds(handle->lp, variable_index + 1, GetBoundType(min, max), min,
                   max);
}

void Problem::SetObjectiveCoefficient(VariableIndex variable_index,
                                      double value) {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  glp_set_obj_coef(handle->lp, variable_index + 1, value);
}

void Problem::SetObjectiveOffset(double value) {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  glp_set_obj_coef(handle->lp, 0, value);
}

std::unique_ptr<Solution> Problem::Solve(std::chrono::milliseconds time_limit) {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  auto solution = std::unique_ptr<Solution>(new Solution());

  // GLPK does not support time limit. Will print an error message and return if
  // set.
  if (time_limit != std::chrono::milliseconds::max()) {
    LOG(ERROR) << "Not supported yet";
    solution->timed_out_ = true;
    return solution;
  }

  int status;
  if (has_binary_variables_) {
    glp_iocp iocp;
    glp_init_iocp(&iocp);
    iocp.msg_lev = GLP_MSG_OFF;
    iocp.presolve = GLP_ON;
    glp_intopt(handle->lp, &iocp);
    status = glp_mip_status(handle->lp);
  } else {
    glp_smcp smcp;
    glp_init_smcp(&smcp);
    smcp.msg_lev = GLP_MSG_OFF;
    glp_simplex(handle->lp, &smcp);
    status = glp_get_status(handle->lp);
  }

  SolutionType solution_type;
  if (status == GLP_OPT) {
    solution_type = OPTIMAL;
  } else if (status == GLP_FEAS) {
    solution_type = FEASIBLE;
  } else {
    solution_type = INFEASIBLE_OR_UNBOUNDED;
  }

  solution->solution_type_ = solution_type;
  for (size_t variable = 0; variable < handle->num_cols; ++variable) {
    double value;
    if (has_binary_variables_) {
      value = glp_mip_col_val(handle->lp, variable + 1);
    } else {
      value = glp_get_col_prim(handle->lp, variable + 1);
    }
    solution->variables_.emplace_back(value);
  }

  if (has_binary_variables_) {
    solution->objective_value_ = glp_mip_obj_val(handle->lp);
  } else {
    solution->objective_value_ = glp_get_obj_val(handle->lp);
  }

  return solution;
}

void Problem::DumpToFile(const std::string& file) {
  GLPKHandle* handle = static_cast<GLPKHandle*>(handle_);
  glp_write_lp(handle->lp, nullptr, file.c_str());
}

#endif

}  // namespace lp
}  // namespace ncode
