#include <stddef.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "ncode_common/src/common.h"
#include "ncode_net/src/net_common.h"

namespace nc {
namespace lp {

// A single demand--source, destination and traffic volume.
struct DemandMatrixElement {
  DemandMatrixElement(net::GraphNodeIndex src, net::GraphNodeIndex dst,
                      net::Bandwidth load)
      : src(src), dst(dst), load(load) {}

  net::GraphNodeIndex src;
  net::GraphNodeIndex dst;
  net::Bandwidth load;
};

// A collection of demands.
class DemandMatrix {
 public:
  using NodePair = std::pair<net::GraphNodeIndex, net::GraphNodeIndex>;

  DemandMatrix(const std::vector<DemandMatrixElement>& elements,
               const net::GraphStorage* graph_storage)
      : elements_(elements), graph_storage_(graph_storage) {
    CHECK(graph_storage != nullptr);
  }

  DemandMatrix(std::vector<DemandMatrixElement>&& elements,
               const net::GraphStorage* graph_storage)
      : elements_(std::move(elements)), graph_storage_(graph_storage) {
    CHECK(graph_storage != nullptr);
  }

  // The elements of this demand matrix.
  const std::vector<DemandMatrixElement>& elements() const { return elements_; }

  // Per-link utilization, when everything is routed over the shortest path.
  net::GraphLinkMap<double> SPUtilization() const;

  // For each ingress-egress pair returns the number of hops on the SP and the
  // delay of the path.
  std::map<NodePair, std::pair<size_t, net::Delay>> SPStats() const;

  // Returns the load that the ingress sends to the egress.
  net::Bandwidth Load(const NodePair& node_pair) const;

  // Total load for all ingress-egress pairs.
  net::Bandwidth TotalLoad() const;

  // Returns the sum of the load over all links divided by the sum of all links'
  // capacity when all demands are routed over their shortest paths.
  double SPGlobalUtilization() const;

  // Returns <max_flow, scale_factor>, where max flow is the max flow through
  // the network, and scale_factor is a number by which all demands can be
  // multiplied to get to max flow.
  std::pair<net::Bandwidth, double> GetMaxFlow(
      const net::GraphLinkSet& to_exclude) const;

  // True if demand can be routed through the network.
  bool IsFeasible(const net::GraphLinkSet& to_exclude) const;

  // Returns the max commodity scale factor for this matrix.
  double MaxCommodityScaleFractor() const;

  // Returns true if the demand matrix is resilient to any single link failure
  // (the load can still fit).
  bool ResilientToFailures() const;

  bool empty() const { return elements_.empty(); }

  // Returns a demand matrix with the same pairs, but with load of all pairs
  // scaled by 'factor'.
  std::unique_ptr<DemandMatrix> Scale(double factor) const;

  // Returns a demand matrix that contains only the largest demand from this
  // demand matrix.
  std::unique_ptr<DemandMatrix> IsolateLargest() const;

  // Prints the matrix.
  std::string ToString() const;

  const net::GraphStorage* graph_storage() const { return graph_storage_; }

 private:
  std::vector<DemandMatrixElement> elements_;
  const net::GraphStorage* graph_storage_;

  DISALLOW_COPY_AND_ASSIGN(DemandMatrix);
};

// A class that generates demand matrices based on a set of constraints.
class DemandGenerator {
 public:
  DemandGenerator(uint64_t seed, net::GraphStorage* graph)
      : graph_(graph),
        max_global_utilization_(std::numeric_limits<double>::max()),
        rnd_(seed),
        min_scale_factor_(1.0) {}

  // Adds a constraint that makes 'fraction' of all links have utilization less
  // than or equal to 'utilization' when demands are routed over their shortest
  // paths.
  void AddUtilizationConstraint(double fraction, double utilization);

  // Adds a constraint that makes 'fraction' of all load go over ie pairs whose
  // shortest paths are hop count or longer.
  void AddHopCountLocalityConstraint(double fraction, size_t hop_count);

  // Adds a constraint that makes 'fraction' of all load go over ie pairs whose
  // shortest paths are distance delay or longer.
  void AddDistanceLocalityConstraint(double fraction,
                                     std::chrono::milliseconds delay);

  // Sets the max global utilization.
  void SetMaxGlobalUtilization(double fraction);

  void SetMinScaleFactor(double factor);

  // Adds a constraint that makes 'fraction' of all demands use up to
  // 'out_fraction' of their respective total outgoing capacity.
  void AddOutgoingFractionConstraint(double fraction, double out_fraction);

  // Generates a matrix. If the explore_alternatives argument is true will
  // generate kMaxTries matrices and pick the one with the highest global
  // utility. The matrix will be scaled by the scale argument after generation
  // (but this function guarantees that it will return a feasible matrix).
  std::unique_ptr<DemandMatrix> GenerateMatrix(
      bool explore_alternatives = false, double scale = 1.0);

 private:
  static constexpr size_t kMaxTries = 100;

  // Called by GenerateMatrix to generate a single matrix, if the matrix does
  // not satisfy the global utilization constraint it is discarded and this
  // function is called again.
  std::unique_ptr<DemandMatrix> GenerateMatrixPrivate();

  // The graph.
  net::GraphStorage* graph_;

  // Define the overall distribution of link utilizations.
  using FractionAndUtilization = std::pair<double, double>;
  std::vector<FractionAndUtilization> utilization_constraints_;

  using FractionAndDistance = std::pair<double, std::chrono::milliseconds>;
  std::vector<FractionAndDistance> locality_delay_constraints_;

  using FractionAndHopCount = std::pair<double, size_t>;
  std::vector<FractionAndHopCount> locality_hop_constraints_;

  using FractionAndOutgoingFraction = std::pair<double, double>;
  std::vector<FractionAndOutgoingFraction> outgoing_fraction_constraints_;

  // The sum of all load that crosses all links divided by the sum of all link
  // capacities will be less than this number.
  double max_global_utilization_;

  // Randomness is needed when links are shuffled before solving the problem.
  std::mt19937 rnd_;

  // All demands in the generated matrix should be scaleable by at least this
  // much, while keeping the matrix feasible.
  double min_scale_factor_;

  DISALLOW_COPY_AND_ASSIGN(DemandGenerator);
};

}  // namespace lp
}  // namespace nc
