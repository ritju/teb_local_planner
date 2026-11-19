#include "split_module_fixture.hpp"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/g2o_types/edge_obstacle.h"

using namespace teb_local_planner;

namespace
{
Point2dContainer rectangle(double x0, double y0, double x1, double y1)
{return {{x0,y0},{x1,y0},{x1,y1},{x0,y1}};}

std::shared_ptr<PolygonObstacle> polygon(const Point2dContainer & points)
{
  auto p = std::make_shared<PolygonObstacle>();
  for (const auto & v : points) {p->pushBackVertex(v);}
  p->finalizePolygon();
  return p;
}

TEST(SignedOptimizationDistance, pointPenetrationIncreasesCostAndLegacyDistanceIsUnchanged)
{
  PolygonRobotFootprint body(rectangle(-0.68,-0.65,1.15,0.65));
  const PoseSE2 pose(0,0,0);
  const double ys[] = {0.85,0.65,0.55,0.0};
  const double expected[] = {0.20,0.0,-0.10,-0.65};
  double previous = -1;
  for (size_t i=0; i<4; ++i) {
    PointObstacle point(0,ys[i]);
    const auto result = body.optimizationDistance(pose, &point);
    EXPECT_NEAR(result.signed_distance, expected[i], 1e-9);
    const double error = penaltyBoundFromBelow(result.signed_distance,0.15,0.05);
    EXPECT_GT(error, previous);
    previous = error;
  }
  PointObstacle center(0,0);
  EXPECT_NEAR(body.calculateDistance(pose,&center),0.65,1e-9);
}

TEST(SignedOptimizationDistance, containmentCrossingAndContactHaveUsableEscapeDirection)
{
  PolygonRobotFootprint body(rectangle(-1,-1,1,1));
  for (const auto & shape : {rectangle(-0.1,-2,0.1,2),rectangle(-0.1,-0.1,0.1,0.1),
    rectangle(-3,-3,3,3),rectangle(1,-0.5,2,0.5)})
  {
    const auto obstacle = polygon(shape);
    const auto result = body.optimizationDistance(PoseSE2(0,0,0),obstacle.get());
    EXPECT_LE(result.signed_distance,0.0);
    EXPECT_NEAR(result.escape_normal.norm(),1.0,1e-9);
    const auto moved = body.optimizationDistance(PoseSE2(
      result.escape_normal.x()*1e-4,result.escape_normal.y()*1e-4,0),obstacle.get());
    EXPECT_GT(moved.signed_distance,result.signed_distance);
  }
  const auto small = polygon(rectangle(-0.1,-0.1,0.1,0.1));
  EXPECT_NEAR(body.optimizationDistance(PoseSE2(0,0,0),small.get()).signed_distance,-1.1,1e-9);
  const auto large = polygon(rectangle(-3,-3,3,3));
  EXPECT_NEAR(body.optimizationDistance(PoseSE2(0,0,0),large.get()).signed_distance,-4.0,1e-9);
}

TEST(SignedOptimizationDistance, rotationVertexOrderAndDegenerateShapes)
{
  auto points = rectangle(-1,-0.5,1,0.5);
  PolygonRobotFootprint body(points);
  PointObstacle point(0,1.2);
  EXPECT_NEAR(body.optimizationDistance(PoseSE2(0,0,M_PI/2),&point).signed_distance,0.2,1e-9);
  std::reverse(points.begin(),points.end()); points.push_back(points.front());
  PolygonRobotFootprint reversed(points);
  EXPECT_NEAR(reversed.optimizationDistance(PoseSE2(0,0,M_PI/2),&point).signed_distance,0.2,1e-9);
  LineObstacle line(0,0,0,0);
  EXPECT_NEAR(body.optimizationDistance(PoseSE2(0,0,0),&line).signed_distance,-0.5,1e-9);
  CircularObstacle circle(0,0,0.1);
  EXPECT_NEAR(body.optimizationDistance(PoseSE2(0,0,0),&circle).signed_distance,-0.6,1e-9);
  PillObstacle pill(Eigen::Vector2d(-0.1,0),Eigen::Vector2d(0.1,0),0.1);
  EXPECT_NEAR(body.optimizationDistance(PoseSE2(0,0,0),&pill).signed_distance,-0.6,1e-9);
  EXPECT_THROW(optimization_distance::between({},{{0,0}}),std::invalid_argument);
  EXPECT_THROW(optimization_distance::between({{NAN,0}},{{0,0}}),std::invalid_argument);
  const auto concave = polygon({{-1,-1},{1,-1},{0,0},{1,1},{-1,1}});
  EXPECT_LT(body.optimizationDistance(PoseSE2(0,0,0),concave.get()).signed_distance,0.0);
}

TEST(SignedOptimizationDistance, distanceGradientMatchesFiniteDifferencesAwayFromTies)
{
  PolygonRobotFootprint body(rectangle(-0.68,-0.65,1.15,0.65));
  for (const Eigen::Vector2d p : {Eigen::Vector2d(1.6,0.8),Eigen::Vector2d(0.5,0.3)}) {
    PointObstacle obstacle(p);
    PoseSE2 pose(0.2,-0.1,0.3);
    const auto result = body.optimizationDistance(pose,&obstacle);
    const auto gradient = obstacleDistanceGradient(body,pose,&obstacle,result);
    for (int i=0;i<3;++i) {
      auto plus=pose,minus=pose;
      if (i==0) {plus.x()+=1e-6;minus.x()-=1e-6;}
      if (i==1) {plus.y()+=1e-6;minus.y()-=1e-6;}
      if (i==2) {plus.theta()+=1e-6;minus.theta()-=1e-6;}
      const double numeric=(body.optimizationDistance(plus,&obstacle).signed_distance-
        body.optimizationDistance(minus,&obstacle).signed_distance)/2e-6;
      EXPECT_NEAR(gradient[i],numeric,1e-4);
    }
  }
}

template<class Edge>
void optimizeCollision(const ObstaclePtr & obstacle, double exponent)
{
  TebConfig config;
  config.robot_model=std::make_shared<PolygonRobotFootprint>(rectangle(-1,-1,1,1));
  config.obstacles.min_obstacle_dist=0.15;
  config.obstacles.inflation_dist=0.4;
  config.optim.penalty_epsilon=0.05;
  config.optim.obstacle_cost_exponent=exponent;
  ObstContainer obstacles{obstacle};
  auto node=std::make_shared<nav2_util::LifecycleNode>("signed_distance_solver_test");
  TebOptimalPlanner planner(node,config,&obstacles);
  auto graph=planner.optimizer();
  auto * pose=new VertexPose(PoseSE2(0,0,0));pose->setId(0);
  ASSERT_TRUE(graph->addVertex(pose));
  auto * edge=new Edge;
  edge->setVertex(0,pose);
  edge->setParameters(config,config.robot_model.get(),obstacle.get());
  edge->information().setIdentity();edge->information()(0,0)=1000.0;
  ASSERT_TRUE(graph->addEdge(edge));
  ASSERT_LT(config.robot_model->optimizationDistance(pose->pose(),obstacle.get()).signed_distance,0.0);
  ASSERT_TRUE(graph->initializeOptimization());
  ASSERT_GT(graph->optimize(80),0);
  EXPECT_GE(config.robot_model->optimizationDistance(pose->pose(),obstacle.get()).signed_distance,0.199);
  graph->clear();  // This test graph owns its vertex; TEB normally owns vertices separately.
}

TEST_F(SplitModuleFixture, obstacleEdgesEscapeSymmetricPointAndPolygonPenetration)
{
  for (const auto & obstacle : ObstContainer{std::make_shared<PointObstacle>(0,0),
    polygon(rectangle(-0.1,-2,0.1,2)),polygon(rectangle(-0.1,-0.1,0.1,0.1))})
  {
    optimizeCollision<EdgeObstacle>(obstacle,1.0);
    optimizeCollision<EdgeInflatedObstacle>(obstacle,1.0);
    optimizeCollision<EdgeObstacle>(obstacle,2.0);
  }
}

TEST_F(SplitModuleFixture, tebPlanOptimizesPenetratingFreePosesAndAssociatesInsideObstacles)
{
  TebConfig config;
  config.robot_model=std::make_shared<PolygonRobotFootprint>(rectangle(-0.15,-0.15,0.15,0.15));
  config.obstacles.min_obstacle_dist=0.15;
  config.obstacles.inflation_dist=0.4;
  config.optim.weight_obstacle=1000.0;
  config.optim.weight_inflation=1.0;
  config.optim.weight_adapt_factor=2.0;
  config.optim.no_inner_iterations=5;config.optim.no_outer_iterations=4;
  ObstContainer obstacles{std::make_shared<PointObstacle>(0.0,0.13)};
  auto node=std::make_shared<nav2_util::LifecycleNode>("signed_distance_teb_test");
  TebOptimalPlanner planner(node,config,&obstacles);
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  for(int i=0;i<=20;++i) {plan.push_back(makePose(-2.0+i*0.2,0.0));}
  ASSERT_TRUE(planner.plan(plan));
  double minimum=1e9;
  for (int i=1;i+1<planner.teb().sizePoses();++i) {
    minimum=std::min(minimum,config.robot_model->optimizationDistance(
      planner.teb().Pose(i),obstacles.front().get()).signed_distance);
  }
  EXPECT_GE(minimum,-1e-6);
  RecordProperty("minimum_free_pose_signed_distance",std::to_string(minimum));
}

class InspectableSignedPlanner : public TebOptimalPlanner
{
public:
  using TebOptimalPlanner::TebOptimalPlanner;
  using TebOptimalPlanner::buildGraph;
};

TEST_F(SplitModuleFixture, penetratingObstacleIsForcedIntoGraphEvenWithZeroAssociationFactors)
{
  TebConfig config;
  config.robot_model=std::make_shared<PolygonRobotFootprint>(rectangle(-1,-1,1,1));
  config.obstacles.min_obstacle_dist=0.15;
  config.obstacles.inflation_dist=0.4;
  config.obstacles.obstacle_association_force_inclusion_factor=0.0;
  config.obstacles.obstacle_association_cutoff_factor=0.0;
  config.obstacles.legacy_obstacle_association=false;
  ObstContainer obstacles{std::make_shared<PointObstacle>(0,0)};
  auto node=std::make_shared<nav2_util::LifecycleNode>("signed_distance_association_test");
  InspectableSignedPlanner planner(node,config,&obstacles);
  std::vector<geometry_msgs::msg::PoseStamped> plan{
    makePose(-3,0),makePose(0,0),makePose(3,0)};
  ASSERT_TRUE(planner.teb().initTrajectoryToGoal(plan,0.5,0.4));
  ASSERT_TRUE(planner.buildGraph());
  bool found=false;
  for (const auto * edge : planner.optimizer()->edges()) {
    const auto * obstacle_edge=dynamic_cast<const EdgeInflatedObstacle *>(edge);
    if (obstacle_edge && obstacle_edge->vertices()[0]==planner.teb().PoseVertex(1)) {found=true;}
  }
  EXPECT_TRUE(found);
}
}  // namespace
