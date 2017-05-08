#include "demand_matrix.h"

#include <functional>
#include <set>

#include "ncode_common/src/map_util.h"
#include "ncode_net/external/ncode_common/src/logging.h"
#include "ncode_net/external/ncode_common/src/perfect_hash.h"
#include "ncode_net/src/algorithm.h"
#include "lp.h"
#include "mc_flow.h"

namespace nc {
namespace lp {

using namespace std::chrono;

net::GraphLinkMap<double> DemandMatrix::SPUtilization() const {
  using namespace net;
  using NodeAndElement = std::pair<GraphNodeIndex, const DemandMatrixElement*>;

  GraphLinkSet all_links = graph_storage_->AllLinks();
  GraphLinkMap<double> out;

  GraphNodeMap<std::vector<NodeAndElement>> src_to_destinations;
  for (const DemandMatrixElement& element : elements_) {
    src_to_destinations[element.src].emplace_back(element.dst, &element);
  }

  for (const auto& src_and_destinations : src_to_destinations) {
    GraphNodeIndex src = src_and_destinations.first;
    const std::vector<NodeAndElement>& destinations =
        *src_and_destinations.second;

    GraphNodeSet destinations_set;
    for (const NodeAndElement& node_and_element : destinations) {
      destinations_set.Insert(node_and_element.first);
    }

    ShortestPath sp(src, destinations_set, {}, graph_storage_->AdjacencyList(),
                    nullptr, nullptr);
    for (const NodeAndElement& node_and_element : destinations) {
      std::unique_ptr<Walk> shortest_path = sp.GetPath(node_and_element.first);
      for (GraphLinkIndex link : shortest_path->links()) {
        out[link] += node_and_element.second->load.Mbps();
      }
    }
  }

  for (GraphLinkIndex link : all_links) {
    double capacity = graph_storage_->GetLink(link)->bandwidth().Mbps();
    double& link_utilization = out[link];
    link_utilization = link_utilization / capacity;
  }

  return out;
}

net::Bandwidth DemandMatrix::TotalLoad() const {
  double load_mbps = 0;
  for (const DemandMatrixElement& element : elements_) {
    load_mbps += element.load.Mbps();
  }

  return net::Bandwidth::FromMBitsPerSecond(load_mbps);
}

net::Bandwidth DemandMatrix::Load(const NodePair& node_pair) const {
  for (const DemandMatrixElement& element : elements_) {
    if (element.src == node_pair.first && element.dst == node_pair.second) {
      return element.load;
    }
  }

  return net::Bandwidth::FromMBitsPerSecond(0);
}

std::map<DemandMatrix::NodePair, std::pair<size_t, net::Delay>>
DemandMatrix::SPStats() const {
  using namespace net;

  std::map<DemandMatrix::NodePair, std::pair<size_t, net::Delay>> out;
  for (const DemandMatrixElement& element : elements_) {
    ShortestPath sp(element.src, {element.dst}, {},
                    graph_storage_->AdjacencyList(), nullptr, nullptr);
    std::unique_ptr<Walk> shortest_path = sp.GetPath(element.dst);
    out[{element.src, element.dst}] = {shortest_path->size(),
                                       shortest_path->delay()};
  }

  return out;
}

double DemandMatrix::SPGlobalUtilization() const {
  net::GraphLinkMap<double> sp_utilizaiton = SPUtilization();
  double total_load = 0;
  double total_capacity = 0;
  for (const auto& link_index_and_utilization : sp_utilizaiton) {
    net::GraphLinkIndex link_index = link_index_and_utilization.first;
    double utilization = *link_index_and_utilization.second;
    double capacity = graph_storage_->GetLink(link_index)->bandwidth().Mbps();
    double load = utilization * capacity;

    total_load += load;
    total_capacity += capacity;
  }

  return total_load / total_capacity;
}

std::unique_ptr<DemandMatrix> DemandMatrix::Scale(double factor) const {
  auto tm = make_unique<DemandMatrix>(elements_, graph_storage_);
  for (auto& element : tm->elements_) {
    double new_load = element.load.Mbps() * factor;
    element.load = net::Bandwidth::FromMBitsPerSecond(new_load);
  }

  return tm;
}

std::unique_ptr<DemandMatrix> DemandMatrix::IsolateLargest() const {
  net::Bandwidth max_rate = net::Bandwidth::Zero();
  const DemandMatrixElement* element_ptr = nullptr;
  for (const DemandMatrixElement& element : elements_) {
    if (element.load > max_rate) {
      element_ptr = &element;
      max_rate = element.load;
    }
  }
  CHECK(element_ptr != nullptr);

  std::vector<DemandMatrixElement> new_elements = {*element_ptr};
  return make_unique<DemandMatrix>(new_elements, graph_storage_);
}

std::pair<net::Bandwidth, double> DemandMatrix::GetMaxFlow(
    const net::GraphLinkSet& to_exclude) const {
  lp::MaxFlowMCProblem max_flow_problem(to_exclude, graph_storage_);
  for (const DemandMatrixElement& element : elements_) {
    max_flow_problem.AddCommodity(element.src, element.dst, element.load);
  }

  net::Bandwidth max_flow = net::Bandwidth::Zero();
  max_flow_problem.GetMaxFlow(&max_flow);
  return std::make_pair(max_flow, max_flow_problem.MaxCommodityScaleFactor());
}

bool DemandMatrix::IsFeasible(const net::GraphLinkSet& to_exclude) const {
  lp::MaxFlowMCProblem max_flow_problem(to_exclude, graph_storage_);
  for (const DemandMatrixElement& element : elements_) {
    max_flow_problem.AddCommodity(element.src, element.dst, element.load);
  }

  return max_flow_problem.IsFeasible();
}

double DemandMatrix::MaxCommodityScaleFractor() const {
  lp::MaxFlowMCProblem max_flow_problem({}, graph_storage_);
  for (const DemandMatrixElement& element : elements_) {
    max_flow_problem.AddCommodity(element.src, element.dst, element.load);
  }

  return max_flow_problem.MaxCommodityScaleFactor();
}

bool DemandMatrix::ResilientToFailures() const {
  for (net::GraphLinkIndex link : graph_storage_->AllLinks()) {
    if (!IsFeasible({link})) {
      return false;
    }
  }

  return true;
}

std::string DemandMatrix::ToString() const {
  std::string out;
  for (const auto& element : elements_) {
    const std::string& src = graph_storage_->GetNode(element.src)->id();
    const std::string& dst = graph_storage_->GetNode(element.dst)->id();

    out += StrCat("(", src, ",", dst, ") -> ", element.load.Mbps(), "mbps\n");
  }
  return out;
}

void DemandGenerator::AddUtilizationConstraint(double fraction,
                                               double utilization) {
  CHECK(utilization >= 0.0);
  CHECK(fraction >= 0.0);
  CHECK(fraction <= 1.0);
  utilization_constraints_.emplace_back(fraction, utilization);
}

void DemandGenerator::AddHopCountLocalityConstraint(double fraction,
                                                    size_t hop_count) {
  CHECK(hop_count > 0);
  CHECK(fraction >= 0.0);
  locality_hop_constraints_.emplace_back(fraction, hop_count);
}

void DemandGenerator::AddDistanceLocalityConstraint(
    double fraction, std::chrono::milliseconds distance) {
  CHECK(distance > std::chrono::milliseconds::zero());
  CHECK(fraction >= 0.0);
  locality_delay_constraints_.emplace_back(fraction, distance);
}

void DemandGenerator::AddOutgoingFractionConstraint(double fraction,
                                                    double out_fraction) {
  CHECK(fraction >= 0);
  CHECK(out_fraction >= 0);
  outgoing_fraction_constraints_.emplace_back(fraction, out_fraction);
  std::sort(outgoing_fraction_constraints_.begin(),
            outgoing_fraction_constraints_.end(),
            std::greater<FractionAndOutgoingFraction>());

  for (size_t i = 0; i < outgoing_fraction_constraints_.size() - 1; ++i) {
    CHECK(outgoing_fraction_constraints_[i].second >=
          outgoing_fraction_constraints_[i + 1].second);
  }
}

static void AddLocalityConstraint(
    const std::vector<std::vector<lp::VariableIndex>>& vars,
    const std::vector<lp::VariableIndex>& all_variables, double fraction,
    size_t limit, lp::Problem* problem,
    std::vector<lp::ProblemMatrixElement>* problem_matrix) {
  // The constraint says that all paths of 'limit' or more need to receive
  // 'fraction' of the total load.
  std::set<lp::VariableIndex> affected_vars;
  for (size_t i = limit - 1; i < vars.size(); ++i) {
    affected_vars.insert(vars[i].begin(), vars[i].end());
  }

  lp::ConstraintIndex sum_constraint = problem->AddConstraint();
  problem->SetConstraintRange(sum_constraint, 0, lp::Problem::kInifinity);
  for (lp::VariableIndex ie_variable : all_variables) {
    bool affected = ContainsKey(affected_vars, ie_variable);
    double coefficient = affected ? 1 - fraction : -fraction;
    problem_matrix->emplace_back(sum_constraint, ie_variable, coefficient);
  }
}

std::unique_ptr<DemandMatrix> DemandGenerator::GenerateMatrix(bool explore_alt,
                                                              double scale) {
  size_t tries = explore_alt ? kMaxTries : 1;

  double max_gu = 0;
  std::unique_ptr<DemandMatrix> best_matrix;
  for (size_t i = 0; i < tries; ++i) {
    auto matrix = GenerateMatrixPrivate();
    if (!matrix) {
      continue;
    }

    matrix = matrix->Scale(scale);
    if (!matrix->IsFeasible({})) {
      continue;
    }

    double scale_factor = matrix->MaxCommodityScaleFractor();
    CHECK(scale_factor < 10.0);
    if (scale_factor < min_scale_factor_) {
      continue;
    }

    double global_utilization = matrix->SPGlobalUtilization();
    if (global_utilization > max_gu) {
      max_gu = global_utilization;
      best_matrix = std::move(matrix);
    }
  }

  return best_matrix;
}

std::unique_ptr<DemandMatrix> DemandGenerator::GenerateMatrixPrivate() {
  using namespace net;
  using namespace std::chrono;

  // The adjacency map.
  const GraphNodeMap<std::vector<net::AdjacencyList::LinkInfo>>& adjacencies =
      graph_->AdjacencyList().Adjacencies();

  GraphLinkSet all_links = graph_->AllLinks();
  lp::Problem problem(lp::MAXIMIZE);
  std::vector<lp::ProblemMatrixElement> problem_matrix;

  // First need to create a variable for each of the N^2 possible pairs, will
  // also compute for each link the pairs on whose shortest path the link is.
  // Will also group pairs based on hop count.
  GraphNodeSet all_nodes = graph_->AllNodes();
  std::map<std::pair<GraphNodeIndex, GraphNodeIndex>, lp::VariableIndex>
      ie_pair_to_variable;

  // The n-th element in this vector contains the variables for all pairs whose
  // shortest paths have length of n + 1 milliseconds.
  std::vector<std::vector<lp::VariableIndex>> by_ms_count;

  // The n-th element in this vector contains the variables for all pairs whose
  // shortest paths have length of n + 1 hops.
  std::vector<std::vector<lp::VariableIndex>> by_hop_count;

  // All variables.
  std::vector<lp::VariableIndex> ordered_variables;

  // For each ie-pair (variable) the total load that the source can emit. This
  // is the sum of the capacities of all links going out of the source. This is
  // redundant -- the value will be the same for all src, but it is convenient.
  std::map<lp::VariableIndex, double> variable_to_total_out_capacity;

  GraphLinkMap<std::vector<lp::VariableIndex>> link_to_variables;
  for (GraphNodeIndex src : all_nodes) {
    double total_out = 0;

    if (adjacencies.HasValue(src)) {
      for (const auto& adjacency : adjacencies.GetValueOrDie(src)) {
        GraphLinkIndex adj_link_index = adjacency.link_index;
        total_out += graph_->GetLink(adj_link_index)->bandwidth().Mbps();
      }
    }

    ShortestPath sp(src, all_nodes, {}, graph_->AdjacencyList(), nullptr,
                    nullptr);
    for (GraphNodeIndex dst : all_nodes) {
      if (src == dst) {
        continue;
      }

      lp::VariableIndex new_variable = problem.AddVariable();
      problem.SetVariableRange(new_variable, 0, lp::Problem::kInifinity);
      ie_pair_to_variable[{src, dst}] = new_variable;
      std::unique_ptr<Walk> shortest_path = sp.GetPath(dst);

      size_t ms_count =
          duration_cast<milliseconds>(shortest_path->delay()).count() + 1;
      by_ms_count.resize(std::max(by_ms_count.size(), ms_count));
      by_ms_count[ms_count - 1].emplace_back(new_variable);

      size_t hop_count = shortest_path->size();
      CHECK(hop_count > 0);
      by_hop_count.resize(std::max(by_hop_count.size(), hop_count));
      by_hop_count[hop_count - 1].emplace_back(new_variable);

      variable_to_total_out_capacity[new_variable] = total_out;
      ordered_variables.emplace_back(new_variable);
      for (GraphLinkIndex link : shortest_path->links()) {
        link_to_variables[link].emplace_back(new_variable);
      }
    }
  }

  std::vector<GraphLinkIndex> ordered_links;
  for (GraphLinkIndex link_index : all_links) {
    ordered_links.emplace_back(link_index);
  }

  std::shuffle(ordered_links.begin(), ordered_links.end(), rnd_);
  std::shuffle(ordered_variables.begin(), ordered_variables.end(), rnd_);

  for (const FractionAndUtilization& utilization_constraint :
       utilization_constraints_) {
    double fraction = utilization_constraint.first;
    size_t index_to = fraction * ordered_links.size();

    for (size_t i = 0; i < index_to; ++i) {
      GraphLinkIndex link_index = ordered_links[i];
      lp::ConstraintIndex set_constraint = problem.AddConstraint();
      Bandwidth link_capacity = graph_->GetLink(link_index)->bandwidth();
      problem.SetConstraintRange(
          set_constraint, 0,
          utilization_constraint.second * link_capacity.Mbps());

      for (lp::VariableIndex ie_variable : link_to_variables[link_index]) {
        problem_matrix.emplace_back(set_constraint, ie_variable, 1.0);
      }
    }
  }

  for (const auto& node_pair_and_variable : ie_pair_to_variable) {
    lp::VariableIndex variable = node_pair_and_variable.second;
    problem.SetObjectiveCoefficient(variable, 1.0);
  }

  for (const FractionAndHopCount& locality_constraint :
       locality_hop_constraints_) {
    double fraction = locality_constraint.first;
    size_t hop_count = locality_constraint.second;
    AddLocalityConstraint(by_hop_count, ordered_variables, fraction, hop_count,
                          &problem, &problem_matrix);
  }

  for (const FractionAndDistance& locality_constraint :
       locality_delay_constraints_) {
    double fraction = locality_constraint.first;
    std::chrono::milliseconds distance = locality_constraint.second;
    AddLocalityConstraint(by_ms_count, ordered_variables, fraction,
                          distance.count(), &problem, &problem_matrix);
  }

  // To enforce the global utilization constraint will have to collect for all
  // links the total load they see from all variables.
  double total_capacity = 0;
  std::map<lp::VariableIndex, double> variable_to_coefficient;
  for (net::GraphLinkIndex link_index : graph_->AllLinks()) {
    Bandwidth link_capacity = graph_->GetLink(link_index)->bandwidth();
    total_capacity += link_capacity.Mbps();
    for (lp::VariableIndex ie_variable : link_to_variables[link_index]) {
      variable_to_coefficient[ie_variable] += 1.0;
    }
  }

  lp::ConstraintIndex gu_constraint = problem.AddConstraint();
  problem.SetConstraintRange(gu_constraint, 0,
                             max_global_utilization_ * total_capacity);
  for (const auto& variable_and_coefficient : variable_to_coefficient) {
    lp::VariableIndex var = variable_and_coefficient.first;
    double coefficient = variable_and_coefficient.second;
    problem_matrix.emplace_back(gu_constraint, var, coefficient);
  }

  for (const FractionAndOutgoingFraction& out_fraction_constraint :
       outgoing_fraction_constraints_) {
    double fraction = out_fraction_constraint.first;
    size_t index_to = fraction * ordered_variables.size();

    for (size_t i = 0; i < index_to; ++i) {
      lp::VariableIndex variable = ordered_variables[i];
      lp::ConstraintIndex out_constraint = problem.AddConstraint();

      double out_from_variable = variable_to_total_out_capacity[variable];
      double limit = out_fraction_constraint.second;

      problem.SetConstraintRange(out_constraint, 0, limit * out_from_variable);
      problem_matrix.emplace_back(out_constraint, variable, 1);
    }
  }

  problem.SetMatrix(problem_matrix);
  std::unique_ptr<lp::Solution> solution = problem.Solve();
  if (solution->type() == lp::INFEASIBLE_OR_UNBOUNDED) {
    return {};
  }

  std::vector<DemandMatrixElement> out;
  for (const auto& node_pair_and_variable : ie_pair_to_variable) {
    const std::pair<GraphNodeIndex, GraphNodeIndex>& ie_pair =
        node_pair_and_variable.first;
    lp::VariableIndex variable = node_pair_and_variable.second;
    double value = solution->VariableValue(variable);
    if (value > 0) {
      out.emplace_back(ie_pair.first, ie_pair.second,
                       net::Bandwidth::FromMBitsPerSecond(value));
    }
  }

  return make_unique<DemandMatrix>(std::move(out), graph_);
}

void DemandGenerator::SetMaxGlobalUtilization(double fraction) {
  CHECK(fraction > 0.0);
  CHECK(fraction < 1.0);
  max_global_utilization_ = fraction;
}

void DemandGenerator::SetMinScaleFactor(double factor) {
  CHECK(factor >= 1.0);
  min_scale_factor_ = factor;
}

}  // namespace lp
}  // namespace nc
