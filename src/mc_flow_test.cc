#include "ncode_net/src/net_gen.h"
#include "gtest/gtest.h"
#include "mc_flow.h"

namespace nc {
namespace lp {
namespace {

using namespace std::chrono;

static constexpr net::Bandwidth kBw1 = net::Bandwidth::FromBitsPerSecond(10000);
static constexpr net::Bandwidth kBw2 =
    net::Bandwidth::FromBitsPerSecond(10000000000);

static std::unique_ptr<net::Walk> GetPath(
    const std::string& path, const net::GraphStorage& graph_storage) {
  return graph_storage.WalkFromStringOrDie(path);
}

// Shorthand for net::Bandwidth::FromBitsPerSecond(x)
static net::Bandwidth BW(uint64_t x) {
  return net::Bandwidth::FromBitsPerSecond(x);
}

static SrcAndDst SD(const std::string& src, const std::string& dst,
                    const net::GraphStorage& graph_storage) {
  return {graph_storage.NodeFromStringOrDie(src),
          graph_storage.NodeFromStringOrDie(dst)};
}

TEST(MCTest, UnidirectionalLink) {
  net::GraphBuilder builder = net::GenerateFullGraph(2, kBw1, microseconds(10));
  builder.AddLink({"N1", "N2", net::Bandwidth::FromBitsPerSecond(10000),
                   std::chrono::milliseconds(100)});

  net::GraphStorage graph_storage(builder);
  MaxFlowMCProblem max_flow_problem({}, &graph_storage);
  max_flow_problem.AddCommodity("N0", "N2");

  net::Bandwidth max_flow = net::Bandwidth::Zero();
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow));
  ASSERT_EQ(10000ul, max_flow.bps());

  // The path should be N0->N1->N2
  std::map<SrcAndDst, std::vector<FlowAndPath>> model_paths;
  std::vector<FlowAndPath>& fp = model_paths[SD("N0", "N2", graph_storage)];
  fp.emplace_back(BW(10000), GetPath("[N0->N1, N1->N2]", graph_storage));

  std::map<SrcAndDst, std::vector<FlowAndPath>> paths;
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow, &paths));
  ASSERT_EQ(10000ul, max_flow.bps());
  ASSERT_EQ(model_paths, paths);
}

TEST(MCTest, Simple) {
  net::GraphBuilder builder = net::GenerateFullGraph(2, kBw1, microseconds(10));
  net::GraphStorage graph_storage(builder);

  net::Bandwidth max_flow = net::Bandwidth::Zero();
  MaxFlowMCProblem max_flow_problem({}, &graph_storage);
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow));
  ASSERT_EQ(0ul, max_flow.bps());

  max_flow_problem.AddCommodity("N0", "N1");
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow));
  ASSERT_EQ(10000ul, max_flow.bps());

  std::map<SrcAndDst, std::vector<FlowAndPath>> model_paths;
  std::vector<FlowAndPath>& fp = model_paths[SD("N0", "N1", graph_storage)];
  fp.emplace_back(BW(10000), GetPath("[N0->N1]", graph_storage));

  std::map<SrcAndDst, std::vector<FlowAndPath>> paths;
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow, &paths));
  ASSERT_EQ(10000ul, max_flow.bps());
  ASSERT_EQ(model_paths, paths);
}

TEST(MCTest, SimpleTwoCommodities) {
  net::GraphBuilder builder = net::GenerateFullGraph(2, kBw1, microseconds(10));
  net::GraphStorage graph_storage(builder);

  MaxFlowMCProblem max_flow_problem({}, &graph_storage);
  max_flow_problem.AddCommodity("N0", "N1");
  max_flow_problem.AddCommodity("N1", "N0");

  net::Bandwidth max_flow = net::Bandwidth::Zero();
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow));
  ASSERT_EQ(20000ul, max_flow.bps());

  std::map<SrcAndDst, std::vector<FlowAndPath>> model_paths;
  std::vector<FlowAndPath>& fp_one = model_paths[SD("N1", "N0", graph_storage)];
  fp_one.emplace_back(BW(10000), GetPath("[N1->N0]", graph_storage));

  std::vector<FlowAndPath>& fp_two = model_paths[SD("N0", "N1", graph_storage)];
  fp_two.emplace_back(BW(10000), GetPath("[N0->N1]", graph_storage));

  std::map<SrcAndDst, std::vector<FlowAndPath>> paths;
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow, &paths));
  ASSERT_EQ(20000ul, max_flow.bps());
  ASSERT_EQ(model_paths, paths);
}

TEST(MCTest, Triangle) {
  net::GraphBuilder builder = net::GenerateFullGraph(3, kBw1, microseconds(10));
  net::GraphStorage graph_storage(builder);

  MaxFlowMCProblem max_flow_problem({}, &graph_storage);
  max_flow_problem.AddCommodity("N0", "N2");

  net::Bandwidth max_flow = net::Bandwidth::Zero();
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow));
  ASSERT_EQ(20000ul, max_flow.bps());

  std::map<SrcAndDst, std::vector<FlowAndPath>> model_paths;
  std::vector<FlowAndPath>& fp = model_paths[SD("N0", "N2", graph_storage)];
  fp.emplace_back(BW(10000), GetPath("[N0->N2]", graph_storage));
  fp.emplace_back(BW(10000), GetPath("[N0->N1, N1->N2]", graph_storage));

  std::map<SrcAndDst, std::vector<FlowAndPath>> paths;
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow, &paths));

  max_flow_problem.AddCommodity("N1", "N2");
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow));
  ASSERT_EQ(20000ul, max_flow.bps());
}

TEST(MCTest, TriangleSameDest) {
  net::GraphBuilder builder = net::GenerateFullGraph(3, kBw1, microseconds(10));
  net::GraphStorage graph_storage(builder);

  MaxFlowMCProblem max_flow_problem({}, &graph_storage);
  max_flow_problem.AddCommodity("N0", "N2");
  max_flow_problem.AddCommodity("N1", "N2");

  net::Bandwidth max_flow = net::Bandwidth::Zero();
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow));
  ASSERT_EQ(20000ul, max_flow.bps());

  std::map<SrcAndDst, std::vector<FlowAndPath>> model_paths;
  std::vector<FlowAndPath>& fp_one = model_paths[SD("N0", "N2", graph_storage)];
  fp_one.emplace_back(BW(10000), GetPath("[N0->N2]", graph_storage));

  std::vector<FlowAndPath>& fp_two = model_paths[SD("N1", "N2", graph_storage)];
  fp_two.emplace_back(BW(10000), GetPath("[N1->N2]", graph_storage));

  std::map<SrcAndDst, std::vector<FlowAndPath>> paths;
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow, &paths));
}

TEST(MCTest, TriangleOneExclude) {
  net::GraphBuilder builder = net::GenerateFullGraph(3, kBw1, microseconds(10));
  net::GraphStorage graph_storage(builder);
  net::GraphLinkIndex l1 = graph_storage.LinkOrDie("N0", "N2");
  net::GraphLinkIndex l2 = graph_storage.LinkOrDie("N2", "N0");

  MaxFlowMCProblem max_flow_problem({l1, l2}, &graph_storage);
  max_flow_problem.AddCommodity("N0", "N2");

  net::Bandwidth max_flow = net::Bandwidth::Zero();
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow));
  ASSERT_EQ(10000ul, max_flow.bps());

  std::map<SrcAndDst, std::vector<FlowAndPath>> model_paths;
  std::vector<FlowAndPath>& fp_one = model_paths[SD("N0", "N2", graph_storage)];
  fp_one.emplace_back(BW(10000), GetPath("[N0->N1, N1->N2]", graph_storage));

  std::map<SrcAndDst, std::vector<FlowAndPath>> paths;
  ASSERT_TRUE(max_flow_problem.GetMaxFlow(&max_flow, &paths));
}

TEST(MCTest, TriangleNoFit) {
  net::GraphBuilder builder = net::GenerateFullGraph(3, kBw1, microseconds(10));
  net::GraphStorage graph_storage(builder);

  MaxFlowMCProblem max_flow_problem({}, &graph_storage);
  max_flow_problem.AddCommodity("N0", "N2", BW(30000));

  net::Bandwidth max_flow = net::Bandwidth::Zero();
  ASSERT_FALSE(max_flow_problem.GetMaxFlow(&max_flow));
}

TEST(MCTest, SimpleFeasible) {
  net::GraphBuilder builder = net::GenerateFullGraph(2, kBw1, microseconds(10));
  net::GraphStorage graph_storage(builder);

  MCProblem mc_problem({}, &graph_storage);
  ASSERT_TRUE(mc_problem.IsFeasible());

  mc_problem.AddCommodity("N0", "N1", BW(10000));
  ASSERT_TRUE(mc_problem.IsFeasible());

  // The optimizer optimizes in Mbps, the default CPLEX feasibility tolerance is
  // not low enough to detect a 1 bit infeasibility (BW(10001) will not work).
  mc_problem.AddCommodity("N1", "N0", BW(11000));
  ASSERT_FALSE(mc_problem.IsFeasible());
}

TEST(MCTest, SimpleScaleFactor) {
  net::GraphBuilder builder = net::GenerateFullGraph(2, kBw2, microseconds(10));
  net::GraphStorage graph_storage(builder);

  MCProblem mc_problem({}, &graph_storage);
  ASSERT_EQ(0, mc_problem.MaxCommodityScaleFactor());

  mc_problem.AddCommodity("N0", "N1");
  ASSERT_EQ(0, mc_problem.MaxCommodityScaleFactor());

  mc_problem.AddCommodity("N1", "N0", BW(8000));
  ASSERT_NEAR(1250000, mc_problem.MaxCommodityScaleFactor(), 0.1);
}

TEST(MCTest, SimpleIncrement) {
  net::GraphBuilder builder = net::GenerateFullGraph(2, kBw2, microseconds(10));
  net::GraphStorage graph_storage(builder);

  MCProblem mc_problem({}, &graph_storage);
  ASSERT_EQ(net::Bandwidth::Zero(), mc_problem.MaxCommodityIncrement());

  mc_problem.AddCommodity("N0", "N1");
  ASSERT_NEAR(10000000000, mc_problem.MaxCommodityIncrement().bps(), 10);
}

}  // namespace
}  // namespace lp
}  // namespace ncode
