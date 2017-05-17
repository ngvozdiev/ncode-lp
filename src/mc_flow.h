#ifndef NCODE_MC_FLOW_H
#define NCODE_MC_FLOW_H

#include <cstdint>
#include <iostream>
#include <vector>

#include "ncode_net/src/net_common.h"
#include "lp.h"

namespace nc {
namespace lp {

// Path and flow on a path.
class FlowAndPath {
 public:
  FlowAndPath(net::Bandwidth flow, std::unique_ptr<net::Walk> path)
      : flow_(flow), path_(std::move(path)) {}

  net::Bandwidth flow() const { return flow_; }

  const net::Walk& path() const { return *path_; }

  std::unique_ptr<net::Walk> TakeOwnershipOfPath() { return std::move(path_); }

  friend bool operator==(const FlowAndPath& lhs, const FlowAndPath& rhs) {
    return std::tie(lhs.flow_, *lhs.path_) == std::tie(rhs.flow_, *rhs.path_);
  }

 private:
  net::Bandwidth flow_;
  std::unique_ptr<net::Walk> path_;
};

// A source node and flow out of that node.
using SrcAndLoad = std::pair<net::GraphNodeIndex, net::Bandwidth>;

// Source and destination nodes.
using SrcAndDst = std::pair<net::GraphNodeIndex, net::GraphNodeIndex>;

// A multi-commodity flow problem. Edge capacities will be taken from the
// bandwidth values of the links in the graph this object is constructed with
// times a multiplier.
class MCProblem {
 public:
  // For each link a map from destination node index to LP variable.
  using VarMap = net::GraphLinkMap<net::GraphNodeMap<VariableIndex>>;

  MCProblem(const net::GraphLinkSet& to_exclude,
            const net::GraphStorage* graph_storage,
            double capacity_multiplier = 1.0);

  // Adds a single commodity to the network, with a given source and sink. If
  // the demand of the commodity is not specified it is assumed to be 0. The
  // unit of the demand should be in the same units as the link bandwidth * the
  // capacity multiplier used during construction.
  void AddCommodity(const std::string& source, const std::string& sink,
                    net::Bandwidth demand = net::Bandwidth::Zero());
  void AddCommodity(net::GraphNodeIndex source, net::GraphNodeIndex sink,
                    net::Bandwidth demand = net::Bandwidth::Zero());

  // Returns true if the MC problem is feasible -- if the commodities/demands
  // can fit in the network.
  bool IsFeasible();

  // If all commodities' demands are multiplied by the returned number the
  // problem will be close to being infeasible. Returns 0 if the problem is
  // currently infeasible or all commodities have 0 demands.
  double MaxCommodityScaleFactor();

  // If the returned demand is added to all commodities the problem will be very
  // close to being infeasible.
  net::Bandwidth MaxCommodityIncrement();

 protected:
  // Returns a map from a graph link to a list of one variable per commodity
  // destination.
  VarMap GetLinkToVariableMap(
      bool constrain_links, Problem* problem,
      std::vector<ProblemMatrixElement>* problem_matrix);

  // Adds flow conservation constraints to the problem.
  void AddFlowConservationConstraints(
      const VarMap& link_to_variables, Problem* problem,
      std::vector<ProblemMatrixElement>* problem_matrix);

  // Recovers the paths from an MC-flow problem. Returns for each commodity the
  // paths and fractions of commodity over each path.
  std::map<SrcAndDst, std::vector<FlowAndPath>> RecoverPaths(
      const VarMap& link_to_variables, const lp::Solution& solution) const;

  // Links that all operations will be performed on. This is the set of all
  // links minus the ones that are excluded during construction.
  net::GraphLinkSet all_links_;

  // Where the graph is stored. Not owned by this object.
  const net::GraphStorage* graph_storage_;

  // All links' capacity will be scaled by this number.
  double capacity_multiplier_;

  // The commodities, grouped by destination.
  net::GraphNodeMap<std::vector<SrcAndLoad>> commodities_;

  // For each node will keep a list of the edges going out of the node and the
  // edges coming into the node.
  net::GraphNodeMap<std::pair<std::vector<net::GraphLinkIndex>,
                              std::vector<net::GraphLinkIndex>>> adjacent_to_v_;

 private:
  // Returns the same problem, but with all commodities' demands multiplied by
  // the given scale factor and increased by 'increment'.
  MCProblem(const MCProblem& other, double scale_factor,
            net::Bandwidth increment);

  // Helper function for RecoverPaths.
  double RecoverPathsRecursive(const SrcAndLoad& commodity,
                               net::GraphNodeIndex dst_index,
                               net::GraphNodeIndex at_node, double overall_flow,
                               net::GraphLinkMap<double>* flow_over_links,
                               net::Links* links_so_far,
                               std::vector<FlowAndPath>* out) const;

  DISALLOW_COPY_AND_ASSIGN(MCProblem);
};

// Solves the multi-commodity max flow problem.
class MaxFlowMCProblem : public MCProblem {
 public:
  MaxFlowMCProblem(const net::GraphLinkSet& to_exclude,
                   const net::GraphStorage* graph_storage,
                   double capacity_multiplier = 1.0)
      : MCProblem(to_exclude, graph_storage, capacity_multiplier) {}

  // Populates the maximum flow (in the same units as edge bandwidth *
  // cpacity_multiplier_) for all commodities. If 'paths' is supplied will also
  // populate it with the actual paths for each commodity that will result in
  // the max flow value. If there are commodities that cannot satisfy their
  // demands false is returned and neither 'max_flow' nor 'paths' are modified.
  bool GetMaxFlow(
      net::Bandwidth* max_flow,
      std::map<SrcAndDst, std::vector<FlowAndPath>>* paths = nullptr);
};

}  // namespace lp
}  // namespace ncode
#endif
