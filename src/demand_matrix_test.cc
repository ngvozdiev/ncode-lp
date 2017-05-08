#include "demand_matrix.h"

#include <gtest/gtest.h>
#include <set>

#include "ncode_net/src/net_gen.h"

namespace nc {
namespace lp {

using namespace nc::net;

static constexpr Bandwidth kBw = Bandwidth::FromBitsPerSecond(1000000);
static constexpr Delay kDelay = Delay(10);

static bool LinkLoadEq(const std::set<double>& utilizations,
                       const DemandMatrix& tm) {
  std::set<double> link_load;
  for (const auto& link_and_load : tm.SPUtilization()) {
    double load = *link_and_load.second;
    link_load.insert(static_cast<int>(load * 1000.0) / 1000.0);
  }
  return utilizations == link_load;
}

class TmGenTest : public ::testing::Test {
 protected:
  TmGenTest()
      : graph_storage_(GenerateFullGraph(2, kBw, kDelay)),
        generator_(1, &graph_storage_) {}

  GraphStorage graph_storage_;
  DemandGenerator generator_;
};

TEST_F(TmGenTest, Empty) { ASSERT_FALSE(generator_.GenerateMatrix()); }

TEST_F(TmGenTest, BadFraction) {
  ASSERT_DEATH(generator_.AddUtilizationConstraint(1.1, 0.5), ".*");
}

// 90% of the links should have utilization <= 0.5, but the other 10% are
// unbounded and can have arbitrariliy high utilization.
TEST_F(TmGenTest, Unbounded) {
  generator_.AddUtilizationConstraint(0.9, 0.5);
  ASSERT_FALSE(generator_.GenerateMatrix());
}

// 100% of the links have utilization 0.5 or less, all of them should be at 0.5.
TEST_F(TmGenTest, Full) {
  generator_.AddUtilizationConstraint(1.0, 0.5);

  auto matrix = generator_.GenerateMatrix();
  ASSERT_TRUE(matrix);
  ASSERT_TRUE(LinkLoadEq({0.5, 0.5}, *matrix));
}

// 50% of the links have utilization 0.6 or less, all links should have
// utilization of 0.1 or less.
TEST_F(TmGenTest, Half) {
  generator_.AddUtilizationConstraint(0.5, 0.6);
  generator_.AddUtilizationConstraint(1.0, 0.1);

  auto matrix = generator_.GenerateMatrix();
  ASSERT_TRUE(matrix);
  ASSERT_TRUE(LinkLoadEq({0.1, 0.1}, *matrix));
}

// 50% of the links have utilization 0.6 or less, all links should have
// utilization of 0.8 or less.
TEST_F(TmGenTest, HalfTwo) {
  generator_.AddUtilizationConstraint(0.5, 0.6);
  generator_.AddUtilizationConstraint(1.0, 0.8);

  auto matrix = generator_.GenerateMatrix();
  ASSERT_TRUE(matrix);
  ASSERT_TRUE(LinkLoadEq({0.6, 0.8}, *matrix));

  // 0.8 * 1.25 is 1.0
  ASSERT_NEAR(1.25, matrix->MaxCommodityScaleFractor(), 0.01);
}

// Same as before, but we only have 2 links, test to make sure 0.6 only affects
// one of them.
TEST_F(TmGenTest, HalfApprox) {
  generator_.AddUtilizationConstraint(0.6, 0.6);
  generator_.AddUtilizationConstraint(1.0, 0.8);

  auto matrix = generator_.GenerateMatrix();
  ASSERT_TRUE(matrix);
  ASSERT_TRUE(LinkLoadEq({0.6, 0.8}, *matrix));
}

}  // namespace lp
}  // naemspace nc
