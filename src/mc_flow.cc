#include "mc_flow.h"

#include <map>
#include <memory>
#include <string>

#include "ncode_common/src/logging.h"
#include "ncode_common/src/map_util.h"
#include "lp.h"

namespace nc {
namespace lp {

MCProblem::MCProblem(const net::GraphLinkSet& to_exclude,
                     const net::GraphStorage* graph_storage,
                     double capacity_multiplier)
    : all_links_(graph_storage->AllLinks()),
      graph_storage_(graph_storage),
      capacity_multiplier_(capacity_multiplier) {
  all_links_.RemoveAll(to_exclude);
  for (net::GraphLinkIndex link_index : all_links_) {
    const net::GraphLink* link = graph_storage->GetLink(link_index);

    net::GraphNodeIndex out = link->src();
    net::GraphNodeIndex in = link->dst();
    adjacent_to_v_[out].first.emplace_back(link_index);
    adjacent_to_v_[in].second.emplace_back(link_index);
  }
}

MCProblem::MCProblem(const MCProblem& mc_problem, double scale_factor,
                     net::Bandwidth increment) {
  graph_storage_ = mc_problem.graph_storage_;
  capacity_multiplier_ = mc_problem.capacity_multiplier_;
  adjacent_to_v_ = mc_problem.adjacent_to_v_;
  all_links_ = mc_problem.all_links_;

  for (const auto& dst_index_and_commodities : mc_problem.commodities_) {
    net::GraphNodeIndex dst_index = dst_index_and_commodities.first;
    const std::vector<SrcAndLoad>& commodities =
        *dst_index_and_commodities.second;

    std::vector<SrcAndLoad>& other_commodities = commodities_[dst_index];
    for (const auto& src_and_load : commodities) {
      net::Bandwidth new_demand = net::Bandwidth::FromMBitsPerSecond(
          src_and_load.second.Mbps() * scale_factor + increment.Mbps());
      other_commodities.emplace_back(src_and_load.first, new_demand);
    }
  }
}

MCProblem::VarMap MCProblem::GetLinkToVariableMap(
    bool constrain_links, Problem* problem,
    std::vector<ProblemMatrixElement>* problem_matrix) {
  VarMap link_to_variables;

  // There will be a variable per-link per-destination.
  for (net::GraphLinkIndex link_index : all_links_) {
    const net::GraphLink* link = graph_storage_->GetLink(link_index);

    // One constraint per link to make sure the sum of all commodities over it
    // fit the capacity of the link.
    ConstraintIndex link_constraint = problem->AddConstraint();

    double scaled_limit;
    if (constrain_links) {
      scaled_limit = link->bandwidth().Mbps() * capacity_multiplier_;
    } else {
      scaled_limit = Problem::kInifinity;
    }

    problem->SetConstraintRange(link_constraint, 0, scaled_limit);
    for (const auto& dst_index_and_commodities : commodities_) {
      net::GraphNodeIndex dst_index = dst_index_and_commodities.first;
      VariableIndex var = problem->AddVariable();
      problem->SetVariableRange(var, 0, Problem::kInifinity);
      link_to_variables[link_index][dst_index] = var;

      problem_matrix->emplace_back(link_constraint, var, 1.0);
    }
  }

  return link_to_variables;
}

static VariableIndex GetVar(const MCProblem::VarMap& var_map,
                            net::GraphLinkIndex edge,
                            net::GraphNodeIndex dst_index) {
  const auto& vars = var_map[edge];
  return vars[dst_index];
}

static bool IsSource(const std::vector<SrcAndLoad>& commodities,
                     net::GraphNodeIndex index, net::Bandwidth* load) {
  for (const auto& src_and_load : commodities) {
    if (src_and_load.first == index) {
      *load = src_and_load.second;
      return true;
    }
  }

  return false;
}

void MCProblem::AddFlowConservationConstraints(
    const VarMap& link_to_variables, Problem* problem,
    std::vector<ProblemMatrixElement>* problem_matrix) {
  // Per-commodity flow conservation.
  for (const auto& dst_index_and_commodities : commodities_) {
    net::GraphNodeIndex dst_index = dst_index_and_commodities.first;
    const std::vector<SrcAndLoad>& commodities =
        *dst_index_and_commodities.second;

    for (const auto& node_and_adj_lists : adjacent_to_v_) {
      net::GraphNodeIndex node = node_and_adj_lists.first;

      const std::vector<net::GraphLinkIndex>& edges_out =
          node_and_adj_lists.second->first;
      const std::vector<net::GraphLinkIndex>& edges_in =
          node_and_adj_lists.second->second;

      // For all nodes except the source and the sink the sum of the flow into
      // the node should be equal to the sum of the flow out. All flow into the
      // source should be 0. All flow out of the sink should be 0.
      ConstraintIndex flow_conservation_constraint = problem->AddConstraint();
      problem->SetConstraintRange(flow_conservation_constraint, 0, 0);

      net::Bandwidth flow_from_source;
      if (IsSource(commodities, node, &flow_from_source)) {
        // Traffic that leaves the source should sum up to at least the demand
        // of the commodity.
        ConstraintIndex source_load_constraint = problem->AddConstraint();
        problem->SetConstraintRange(source_load_constraint,
                                    flow_from_source.Mbps(),
                                    Problem::kInifinity);

        for (net::GraphLinkIndex edge_out : edges_out) {
          VariableIndex var = GetVar(link_to_variables, edge_out, dst_index);
          problem_matrix->emplace_back(source_load_constraint, var, 1.0);
        }

        for (net::GraphLinkIndex edge_in : edges_in) {
          VariableIndex var = GetVar(link_to_variables, edge_in, dst_index);
          problem_matrix->emplace_back(source_load_constraint, var, -1.0);
        }

      } else if (node == dst_index) {
        for (net::GraphLinkIndex edge_out : edges_out) {
          VariableIndex var = GetVar(link_to_variables, edge_out, dst_index);
          problem_matrix->emplace_back(flow_conservation_constraint, var, 1.0);
        }
      } else {
        for (net::GraphLinkIndex edge_out : edges_out) {
          VariableIndex var = GetVar(link_to_variables, edge_out, dst_index);
          problem_matrix->emplace_back(flow_conservation_constraint, var, -1.0);
        }

        for (net::GraphLinkIndex edge_in : edges_in) {
          VariableIndex var = GetVar(link_to_variables, edge_in, dst_index);
          problem_matrix->emplace_back(flow_conservation_constraint, var, 1.0);
        }
      }
    }
  }
}

std::map<SrcAndDst, std::vector<FlowAndPath>> MCProblem::RecoverPaths(
    const VarMap& link_to_variables, const lp::Solution& solution) const {
  std::map<SrcAndDst, std::vector<FlowAndPath>> out;

  for (const auto& dst_index_and_commodities : commodities_) {
    net::GraphNodeIndex dst_index = dst_index_and_commodities.first;

    net::GraphLinkMap<double> link_to_flow;
    for (const auto& link_and_variables : link_to_variables) {
      net::GraphLinkIndex link = link_and_variables.first;
      const net::GraphNodeMap<VariableIndex>& variables =
          *link_and_variables.second;
      CHECK(variables.HasValue(dst_index));

      double flow = solution.VariableValue(variables[dst_index]);
      if (flow > 0) {
        link_to_flow[link] = flow;
      }
    }

    const std::vector<SrcAndLoad>& commodities =
        *dst_index_and_commodities.second;
    for (const SrcAndLoad& commodity : commodities) {
      net::Links links;
      std::vector<FlowAndPath>& paths = out[{commodity.first, dst_index}];
      bool commodity_has_volume = commodity.second > net::Bandwidth::Zero();

      double starting_flow = commodity_has_volume
                                 ? commodity.second.Mbps()
                                 : std::numeric_limits<double>::max();
      RecoverPathsRecursive(commodity, dst_index, commodity.first,
                            starting_flow, &link_to_flow, &links, &paths);
      if (commodity_has_volume) {
        double total_flow = 0;
        for (const FlowAndPath& path : paths) {
          total_flow += path.flow().Mbps();
        }
        CHECK(std::abs(total_flow - starting_flow) / total_flow < 0.01)
            << total_flow << " vs " << starting_flow;
      }
    }
  }

  return out;
}

static bool AlreadySeen(net::GraphLinkIndex link,
                        const net::Links& links_so_far) {
  return std::find(links_so_far.begin(), links_so_far.end(), link) !=
         links_so_far.end();
}

double MCProblem::RecoverPathsRecursive(
    const SrcAndLoad& commodity, net::GraphNodeIndex dst_index,
    net::GraphNodeIndex at_node, double overall_flow,
    net::GraphLinkMap<double>* flow_over_links, net::Links* links_so_far,
    std::vector<FlowAndPath>* out) const {
  if (at_node == dst_index) {
    CHECK(overall_flow != std::numeric_limits<double>::max());
    if (commodity.second > net::Bandwidth::Zero()) {
      CHECK(overall_flow <= commodity.second.Mbps());
    }

    for (net::GraphLinkIndex link : *links_so_far) {
      double& remaining = flow_over_links->GetValueOrDie(link);
      remaining -= overall_flow;
    }

    auto new_path = make_unique<net::Walk>(*links_so_far, *graph_storage_);
    auto flow_on_path = net::Bandwidth::FromMBitsPerSecond(overall_flow);
    out->emplace_back(flow_on_path, std::move(new_path));
    //    LOG(ERROR) << "FF " << flow_on_path << " "
    //               << new_path.ToStringNoPorts(graph_storage_);
    return 0;
  }

  const auto& adj_lists = adjacent_to_v_.GetValueOrDie(at_node);
  const std::vector<net::GraphLinkIndex>& edges_out = adj_lists.first;
  for (net::GraphLinkIndex edge_out : edges_out) {
    if (!flow_over_links->HasValue(edge_out)) {
      continue;
    }

    if (AlreadySeen(edge_out, *links_so_far)) {
      continue;
    }

    const net::GraphLink* edge = graph_storage_->GetLink(edge_out);
    double& remaining = flow_over_links->GetValueOrDie(edge_out);
    double to_take = std::min(remaining, overall_flow);
    //    LOG(ERROR) << "OF " << overall_flow << " R " << remaining << " TT "
    //               << to_take << " at " <<
    //               graph_storage_->GetNode(at_node)->id();
    if (to_take > 0) {
      links_so_far->emplace_back(edge_out);
      double remainder =
          RecoverPathsRecursive(commodity, dst_index, edge->dst(), to_take,
                                flow_over_links, links_so_far, out);
      overall_flow -= (to_take - remainder);
      links_so_far->pop_back();
      //      LOG(ERROR) << "Remainder " << remainder << " OF " << overall_flow
      //                 << " at " << graph_storage_->GetNode(at_node)->id();
    }
  }

  return overall_flow;
}

void MCProblem::AddCommodity(const std::string& source, const std::string& sink,
                             net::Bandwidth demand) {
  AddCommodity(graph_storage_->NodeFromStringOrDie(source),
               graph_storage_->NodeFromStringOrDie(sink), demand);
}

void MCProblem::AddCommodity(net::GraphNodeIndex source,
                             net::GraphNodeIndex sink, net::Bandwidth demand) {
  std::vector<SrcAndLoad>& src_and_loads = commodities_[sink];
  for (const SrcAndLoad& src_and_load : src_and_loads) {
    CHECK(src_and_load.first != source);
  }

  src_and_loads.emplace_back(source, demand);
}

bool MCProblem::IsFeasible() {
  Problem problem(MAXIMIZE);
  std::vector<ProblemMatrixElement> problem_matrix;
  VarMap link_to_variables =
      GetLinkToVariableMap(true, &problem, &problem_matrix);
  AddFlowConservationConstraints(link_to_variables, &problem, &problem_matrix);

  // Solve the problem.
  problem.SetMatrix(problem_matrix);
  std::unique_ptr<Solution> solution = problem.Solve();
  return solution->type() == lp::OPTIMAL || solution->type() == lp::FEASIBLE;
}

static constexpr double kMaxScaleFactor = 10000000.0;
static constexpr double kStopThreshold = 0.0001;

double MCProblem::MaxCommodityScaleFactor() {
  if (!IsFeasible()) {
    return 0;
  }

  bool all_zero = true;
  for (const auto& dst_index_and_commodities : commodities_) {
    const std::vector<SrcAndLoad>& commodities =
        *dst_index_and_commodities.second;
    for (const SrcAndLoad& commodity : commodities) {
      if (commodity.second != net::Bandwidth::Zero()) {
        all_zero = false;
        break;
      }
    }

    if (!all_zero) {
      break;
    }
  }

  if (all_zero) {
    return 0;
  }

  // Will do a binary search to look for the problem that is the closes to being
  // infeasible in a given number of steps.
  double min_bound = 1.0;
  double max_bound = kMaxScaleFactor;

  double curr_estimate = kMaxScaleFactor;
  while (true) {
    CHECK(max_bound >= min_bound);
    double delta = max_bound - min_bound;
    if (delta <= kStopThreshold) {
      break;
    }

    double guess = min_bound + (max_bound - min_bound) / 2;
    MCProblem test_problem(*this, guess, net::Bandwidth::Zero());

    bool is_feasible = test_problem.IsFeasible();
    if (is_feasible) {
      curr_estimate = guess;
      min_bound = guess;
    } else {
      max_bound = guess;
    }
  }

  if (curr_estimate == kMaxScaleFactor) {
    return 1.0;
  }

  return curr_estimate;
}

net::Bandwidth MCProblem::MaxCommodityIncrement() {
  if (!IsFeasible() || commodities_.Count() == 0) {
    return net::Bandwidth::Zero();
  }

  // The initial increment will be the max of the cpacity of all links.
  net::Bandwidth max_capacity = net::Bandwidth::Zero();
  for (net::GraphLinkIndex link_index : all_links_) {
    const net::GraphLink* link = graph_storage_->GetLink(link_index);

    if (link->bandwidth() > max_capacity) {
      max_capacity = link->bandwidth();
    }
  }

  double min_bound = 1.0;
  double max_bound = max_capacity.bps();
  double curr_estimate = max_capacity.bps();
  while (true) {
    CHECK(max_bound >= min_bound);
    double delta = max_bound - min_bound;
    if (delta <= kStopThreshold) {
      break;
    }

    double guess = min_bound + (max_bound - min_bound) / 2;
    MCProblem test_problem(*this, 1.0,
                           net::Bandwidth::FromBitsPerSecond(guess));

    bool is_feasible = test_problem.IsFeasible();
    if (is_feasible) {
      curr_estimate = guess;
      min_bound = guess;
    } else {
      max_bound = guess;
    }
  }

  return net::Bandwidth::FromBitsPerSecond(curr_estimate);
}

bool MaxFlowMCProblem::GetMaxFlow(
    net::Bandwidth* max_flow,
    std::map<SrcAndDst, std::vector<FlowAndPath>>* paths) {
  Problem problem(MAXIMIZE);
  std::vector<ProblemMatrixElement> problem_matrix;
  VarMap link_to_variables =
      GetLinkToVariableMap(true, &problem, &problem_matrix);
  AddFlowConservationConstraints(link_to_variables, &problem, &problem_matrix);

  for (const auto& dst_index_and_commodities : commodities_) {
    net::GraphNodeIndex dst_index = dst_index_and_commodities.first;
    const std::vector<SrcAndLoad>& commodities =
        *dst_index_and_commodities.second;

    // Records net total flow per variable.
    std::map<VariableIndex, double> totals;
    for (const auto& node_and_adj_lists : adjacent_to_v_) {
      net::GraphNodeIndex node = node_and_adj_lists.first;

      const std::vector<net::GraphLinkIndex>& edges_out =
          node_and_adj_lists.second->first;
      const std::vector<net::GraphLinkIndex>& edges_in =
          node_and_adj_lists.second->second;

      net::Bandwidth flow_from_source;
      if (IsSource(commodities, node, &flow_from_source)) {
        for (net::GraphLinkIndex edge_out : edges_out) {
          VariableIndex var = GetVar(link_to_variables, edge_out, dst_index);
          totals[var] += 1.0;
        }

        for (net::GraphLinkIndex edge_in : edges_in) {
          VariableIndex var = GetVar(link_to_variables, edge_in, dst_index);
          totals[var] -= 1.0;
        }
      }
    }

    for (const auto& var_index_and_total : totals) {
      VariableIndex var = var_index_and_total.first;
      double total = var_index_and_total.second;
      problem.SetObjectiveCoefficient(var, total);
    }
  }

  // Solve the problem.
  problem.SetMatrix(problem_matrix);
  std::unique_ptr<Solution> solution = problem.Solve();
  if (solution->type() != lp::OPTIMAL && solution->type() != lp::FEASIBLE) {
    return false;
  }

  *max_flow = net::Bandwidth::FromMBitsPerSecond(solution->ObjectiveValue());
  if (paths) {
    *paths = RecoverPaths(link_to_variables, *solution);
  }

  return true;
}

}  // namespace lp
}  // namespace ncode
