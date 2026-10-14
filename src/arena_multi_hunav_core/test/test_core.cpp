#include "arena_multi_hunav_core/core.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
using namespace arena_multi_hunav_core;
using Agent = hunav_msgs::msg::Agent;
Compute::Request fixture();

ComputeV2::Request v2_fixture() {
  const auto q = fixture();
  ComputeV2::Request v;
  v.epoch=q.epoch; v.step=q.step; v.dt=q.dt; v.current_agents=q.current_agents;
  v.robots=q.robots; v.modifiers=q.modifiers;
  arena_multi_hunav_msgs::msg::BehaviorIntent i; i.agent_id=1; i.allow_task_progress=true;
  v.intents.push_back(i); return v;
}

Compute::Request fixture() {
  Compute::Request q;
  q.epoch=7; q.step=1; q.dt=0.025;
  q.current_agents.header.frame_id="map"; q.robots.header.frame_id="map";
  Agent a; a.id=1; a.name="person"; a.type=Agent::PERSON; a.behavior.type=1;
  a.group_id=-1; a.radius=0.4; a.desired_velocity=1; a.goal_radius=0.3;
  a.behavior.goal_force_factor=2; a.behavior.obstacle_force_factor=10; a.behavior.social_force_factor=5;
  a.position.position.x=10; a.position.position.y=10; a.position.orientation.w=1;
  geometry_msgs::msg::Pose goal; goal.position.x=16; goal.position.y=10; goal.orientation.w=1;
  a.goals.push_back(goal); q.current_agents.agents.push_back(a);
  arena_multi_hunav_msgs::msg::BehaviorModifiers m; m.agent_id=1; q.modifiers.push_back(m);
  for(int i=1;i<=2;i++) {
    Agent r; r.id=-i; r.name="robot_"+std::to_string(i); r.type=Agent::ROBOT; r.group_id=-1;
    r.radius=0.35; r.position.position.x=11; r.position.position.y=10+(i==1 ? -0.8:0.8);
    r.position.orientation.w=1; q.robots.agents.push_back(r);
  }
  return q;
}

TEST(CoreV2, NeutralIntentMatchesV1OverMultipleSteps) {
  auto v=v2_fixture(); auto q=fixture();
  for (int step=0; step<1200; ++step) {
    q.robots.header.stamp=q.current_agents.header.stamp;
    v.robots.header.stamp=v.current_agents.header.stamp;
    auto a=integrate(q,{});auto b=integrate(v,{});
    ASSERT_TRUE(a.success)<<a.error;ASSERT_TRUE(b.success)<<b.error;
    EXPECT_EQ(a.updated_agents,b.updated_agents);EXPECT_EQ(a.influences,b.influences);
    q.current_agents=a.updated_agents;v.current_agents=b.updated_agents;
  }
}
TEST(CoreV2, TemporaryIntentPausesQueueAndRegularRestoresIt) {
  auto v=v2_fixture();auto & i=v.intents[0];auto & p=v.current_agents.agents[0];
  p.position.position.x=16;p.cyclic_goals=true;
  auto next=p.goals.front();next.position.x=10;p.goals.push_back(next);
  i.mode=i.CURIOUS;i.has_target_robot=true;i.target_robot_id=-1;i.override_desired_velocity=true;i.allow_task_progress=false;
  const auto queue=p.goals;auto special=integrate(v,{});ASSERT_TRUE(special.success)<<special.error;
  EXPECT_EQ(special.updated_agents.agents[0].goals,queue);
  v.current_agents=special.updated_agents;v.robots.header.stamp=v.current_agents.header.stamp;
  i=arena_multi_hunav_msgs::msg::BehaviorIntent();i.agent_id=1;i.allow_task_progress=true;
  auto regular=integrate(v,{});ASSERT_TRUE(regular.success)<<regular.error;
  Compute::Request baseline;baseline.epoch=v.epoch;baseline.step=v.step;baseline.dt=v.dt;
  baseline.current_agents=v.current_agents;baseline.robots=v.robots;baseline.modifiers=v.modifiers;
  EXPECT_EQ(regular.updated_agents,integrate(baseline,{}).updated_agents);
  EXPECT_NE(regular.updated_agents.agents[0].goals,queue);
}
TEST(CoreV2, ZeroDesiredVelocityDeceleratesAndNearStillMovesPeople) {
  auto v=v2_fixture();auto & i=v.intents[0];i.vigilant=true;i.override_desired_velocity=true;
  v.current_agents.agents[0].velocity.linear.x=.8;
  auto result=integrate(v,{});ASSERT_TRUE(result.success)<<result.error;
  EXPECT_GE(result.updated_agents.agents[0].velocity.linear.x,.8-3*.025-1e-9);
  v.current_agents.agents[0].velocity.linear.x=0;
  v.robots.agents.resize(1);v.robots.agents[0].position.position.x=10.8;v.robots.agents[0].position.position.y=10;
  v.modifiers[0].robot_scale=0;
  result=integrate(v,{});ASSERT_TRUE(result.success)<<result.error;
  EXPECT_LT(result.updated_agents.agents[0].velocity.linear.x,0);
  EXPECT_LT(result.motion[0].robot_near_force.x,0);
}
TEST(CoreV2, NearIsIndependentOfModeAndDiagnosticSumIsExact) {
  auto v=v2_fixture();auto a=integrate(v,{});ASSERT_TRUE(a.success);
  auto & i=v.intents[0];i.mode=i.SCARED;i.has_target_robot=true;i.target_robot_id=-1;
  i.override_desired_velocity=true;i.desired_velocity_x=-.8;i.allow_task_progress=false;
  v.modifiers[0].robot_scale=0;
  auto b=integrate(v,{});ASSERT_TRUE(b.success)<<b.error;
  EXPECT_EQ(a.motion[0].robot_near_force,b.motion[0].robot_near_force);
  const auto & d=b.motion[0];
  EXPECT_NEAR(d.total_before_limit.x,d.desired_force.x+d.obstacle_force.x+d.human_social_force.x+d.robot_social_force.x+d.robot_near_force.x,1e-12);
}
TEST(CoreV2, InvalidIntentsFailAtomically) {
  for (int error=0;error<5;++error) {
    auto v=v2_fixture();
    if(error==0)v.intents.clear();
    if(error==1)v.intents.push_back(v.intents[0]);
    if(error==2)v.intents[0].mode=99;
    if(error==3){v.intents[0].has_target_robot=true;v.intents[0].target_robot_id=-999;}
    if(error==4)v.intents[0].desired_velocity_x=NAN;
    auto r=integrate(v,{});EXPECT_FALSE(r.success);EXPECT_EQ(r.updated_agents,v.current_agents);
    EXPECT_TRUE(r.motion.empty());EXPECT_TRUE(r.influences.empty());
  }
}

TEST(Core, AllRobotsHaveIndependentContributions) {
  auto q=fixture(); auto both=integrate(q, {}); ASSERT_TRUE(both.success)<<both.error;
  ASSERT_EQ(both.influences.size(),2u);
  for(const auto & f:both.influences) EXPECT_GT(std::hypot(f.force.x,f.force.y),0.01);
  auto one=q; one.robots.agents.erase(one.robots.agents.begin());
  auto only=integrate(one, {}); ASSERT_TRUE(only.success)<<only.error;
  EXPECT_NE(both.updated_agents.agents[0].velocity.linear.y, only.updated_agents.agents[0].velocity.linear.y);
  for(const auto & f:both.influences) if(f.robot_name==only.influences[0].robot_name) {
    EXPECT_DOUBLE_EQ(f.force.x,only.influences[0].force.x);
    EXPECT_DOUBLE_EQ(f.force.y,only.influences[0].force.y);
  }
}
TEST(Core, RobotOrderAndNamesDoNotChangeMotion) {
  auto q=fixture(); auto a=integrate(q, {});
  std::reverse(q.robots.agents.begin(), q.robots.agents.end());
  std::swap(q.robots.agents[0].name,q.robots.agents[1].name);
  auto b=integrate(q, {}); ASSERT_TRUE(b.success);
  EXPECT_EQ(a.updated_agents,b.updated_agents);
}
TEST(Core, EmptyRobotsAndModifiers) {
  auto q=fixture(); q.robots.agents.clear();
  auto r=integrate(q, {}); ASSERT_TRUE(r.success)<<r.error;
  EXPECT_GT(r.updated_agents.agents[0].position.position.x,10);
  q.modifiers.clear(); EXPECT_FALSE(integrate(q, {}).success);
}
TEST(Core, GeometryAndInputFailuresAreAtomic) {
  auto q=fixture(); q.robots.agents[0].id=1; auto r=integrate(q, {});
  EXPECT_FALSE(r.success); EXPECT_EQ(r.updated_agents,q.current_agents);
  q=fixture(); q.dt=0.026; EXPECT_FALSE(integrate(q, {}).success);
  q=fixture(); q.robots.header.frame_id="odom"; EXPECT_FALSE(integrate(q, {}).success);
  q=fixture(); q.robots.agents[0].velocity.linear.x=NAN; EXPECT_FALSE(integrate(q, {}).success);
  q=fixture(); q.modifiers[0].space_scale=0.5; EXPECT_FALSE(integrate(q, {}).success);
}
TEST(Core, CoincidentEntitiesRemainFinite) {
  auto q=fixture(); q.robots.agents[0].position=q.current_agents.agents[0].position;
  auto r=integrate(q, {}); ASSERT_TRUE(r.success)<<r.error;
  EXPECT_TRUE(std::isfinite(r.updated_agents.agents[0].velocity.linear.x));
}
TEST(Core, PsychologyAppliesWithoutMutatingBaseSpeed) {
  auto q=fixture(); q.robots.agents.clear(); q.modifiers[0].speed_scale=0;
  auto r=integrate(q, {}); ASSERT_TRUE(r.success);
  EXPECT_DOUBLE_EQ(r.updated_agents.agents[0].velocity.linear.x,0);
  EXPECT_DOUBLE_EQ(r.updated_agents.agents[0].desired_velocity,1);
}
TEST(Core, GoalsAdvanceAndRobotsNeverIntegrated) {
  auto q=fixture(); q.robots.agents.clear();
  q.current_agents.agents[0].goals[0].position=q.current_agents.agents[0].position.position;
  auto r=integrate(q, {}); ASSERT_TRUE(r.success); EXPECT_TRUE(r.updated_agents.agents[0].goals.empty());
}

TEST(Core, MultiGoalCyclesAndFoldsAdvanceExactlyOneTargetPerStep) {
  for (const auto & order : std::vector<std::vector<int>>{
      {0, 1}, {0, 1, 2}, {0, 1, 2, 3}, {0, 1, 2, 1}, {0, 1, 2, 3, 2, 1}}) {
    auto q = fixture(); q.robots.agents.clear(); q.modifiers[0].speed_scale = 0;
    auto & person = q.current_agents.agents[0];
    person.cyclic_goals = true; person.goals.clear();
    for (int target : order) {
      geometry_msgs::msg::Pose goal;
      goal.position.x = 8 + 2 * target; goal.position.y = 10; goal.orientation.w = 1;
      person.goals.push_back(goal);
    }
    auto expected = person.goals;
    for (size_t step = 0; step < order.size() * 3; ++step) {
      q.current_agents.agents[0].position.position = expected.front().position;
      q.robots.header.stamp = q.current_agents.header.stamp;
      const auto original = q.current_agents;
      auto r = integrate(q, {}); ASSERT_TRUE(r.success) << r.error;
      EXPECT_EQ(q.current_agents, original);
      std::rotate(expected.begin(), expected.begin() + 1, expected.end());
      EXPECT_EQ(r.updated_agents.agents[0].goals, expected);
      q.current_agents = r.updated_agents;
    }
  }
}

TEST(Core, CyclicGoalAdvancesAtInclusiveRadiusOnly) {
  for (double offset : {.249, .25, .251}) {
    auto q = fixture(); q.robots.agents.clear(); q.modifiers[0].speed_scale = 0;
    auto & person = q.current_agents.agents[0];
    person.cyclic_goals = true; person.goal_radius = .25;
    auto next = person.goals[0]; next.position.x = 8; person.goals.push_back(next);
    person.position.position.x = 16 + offset;
    auto expected = person.goals;
    if (offset <= .25) std::rotate(expected.begin(), expected.begin() + 1, expected.end());
    const auto r = integrate(q, {}); ASSERT_TRUE(r.success) << r.error;
    EXPECT_EQ(r.updated_agents.agents[0].goals, expected);
  }
}

TEST(Core, FailureAfterCandidateAdvanceRestoresEveryGoalQueue) {
  auto q = fixture(); q.robots.agents.clear(); q.modifiers[0].speed_scale = 0;
  auto & person = q.current_agents.agents[0]; person.cyclic_goals = true;
  person.goals[0].position = person.position.position;
  auto next = person.goals[0]; next.position.x = 15; person.goals.push_back(next);
  auto invalid = person; invalid.id = 2; invalid.name = "outside";
  invalid.position.position.x = 31; q.current_agents.agents.push_back(invalid);
  auto modifier = q.modifiers[0]; modifier.agent_id = 2; q.modifiers.push_back(modifier);
  auto result = integrate(q, {});
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.updated_agents, q.current_agents);
  EXPECT_TRUE(result.influences.empty());
}
TEST(Core, ExactHeadOnStaticRobotDoesNotCausePermanentStop) {
  auto q=fixture(); q.robots.agents.resize(1); q.robots.agents[0].position.position.y=10;
  for(int i=0;i<4800 && !q.current_agents.agents[0].goals.empty();i++) {
    q.robots.header.stamp=q.current_agents.header.stamp;
    auto r=integrate(q,{}); ASSERT_TRUE(r.success)<<r.error;
    const auto & p=r.updated_agents.agents[0].position.position;
    EXPECT_GE(std::hypot(p.x-11,p.y-10)-.75,.1-1e-5);
    q.current_agents=r.updated_agents;
  }
  EXPECT_TRUE(q.current_agents.agents[0].goals.empty());
}
TEST(Core, AblatingEitherRobotChangesTrajectoryByTenCentimeters) {
  for(int omitted=0;omitted<2;omitted++) {
    auto full=fixture(), ablated=full;
    ablated.robots.agents.erase(ablated.robots.agents.begin()+omitted);
    double maximum=0;
    for(int i=0;i<320;i++) {
      full.robots.header.stamp=full.current_agents.header.stamp;
      ablated.robots.header.stamp=ablated.current_agents.header.stamp;
      auto a=integrate(full,{}), b=integrate(ablated,{});
      ASSERT_TRUE(a.success); ASSERT_TRUE(b.success);
      auto pa=a.updated_agents.agents[0].position.position, pb=b.updated_agents.agents[0].position.position;
      maximum=std::max(maximum,std::hypot(pa.x-pb.x,pa.y-pb.y));
      full.current_agents=a.updated_agents; ablated.current_agents=b.updated_agents;
    }
    EXPECT_GE(maximum,.10)<<"robot index="<<omitted;
  }
}

TEST(Core, ReservedSpaceScaleDoesNotChangeMotionOrNearRepulsion) {
  auto q = fixture();
  auto original = integrate(q, {}); ASSERT_TRUE(original.success);
  q.modifiers[0].space_scale = 4;
  auto changed = integrate(q, {}); ASSERT_TRUE(changed.success);
  EXPECT_EQ(original.updated_agents, changed.updated_agents);
  EXPECT_EQ(original.influences, changed.influences);
}

TEST(Core, NearRepulsionUsesPhysicalConfigAndIgnoresPsychology) {
  auto q = fixture(); q.robots.agents.resize(1);
  q.robots.agents[0].position.position.x = 11.2;
  q.robots.agents[0].position.position.y = 10;
  q.current_agents.agents[0].behavior.social_force_factor = 0;
  Config cfg; cfg.robot_clearance = .15; cfg.near_gain = 8; cfg.near_sigma = .1;
  auto original = integrate(q, cfg); ASSERT_TRUE(original.success);
  ASSERT_EQ(original.influences.size(), 1u);
  const double gap = 1.2 - q.current_agents.agents[0].radius - q.robots.agents[0].radius;
  EXPECT_NEAR(original.influences[0].force.x, -8 * std::exp((.15 - gap) / .1), 1e-10);
  EXPECT_DOUBLE_EQ(original.influences[0].force.y, 0);
  q.modifiers[0].speed_scale = 0;
  q.modifiers[0].social_scale = 4;
  q.modifiers[0].robot_scale = 0;
  q.modifiers[0].space_scale = 4;
  auto changed = integrate(q, cfg); ASSERT_TRUE(changed.success);
  EXPECT_EQ(original.influences, changed.influences);
}

Compute::Request robot_at(double relative_degrees, double distance = 1.2, double yaw = 0) {
  auto q = fixture(); q.robots.agents.resize(1);
  q.current_agents.agents[0].yaw = yaw;
  const double bearing = yaw + relative_degrees * std::acos(-1.0) / 180;
  q.robots.agents[0].position.position.x = 10 + distance * std::cos(bearing);
  q.robots.agents[0].position.position.y = 10 + distance * std::sin(bearing);
  return q;
}

TEST(Core, RobotSocialVisibilityHasFullFadeAndBlindRegions) {
  Config all; all.robot_fov_deg = 360;
  for (const auto & sample : std::vector<std::pair<double, double>>{
      {0, 1}, {80, 1}, {90, 1}, {95, .5}, {99, .1}, {100, 0}, {110, 0}, {180, 0},
      {-90, 1}, {-95, .5}, {-100, 0}, {-180, 0}}) {
    auto q = robot_at(sample.first);
    auto limited = integrate(q, {}), full = integrate(q, all);
    q.modifiers[0].robot_scale = 0;
    auto near = integrate(q, {});
    ASSERT_TRUE(limited.success); ASSERT_TRUE(full.success); ASSERT_TRUE(near.success);
    const auto a = limited.influences[0].force, b = full.influences[0].force, n = near.influences[0].force;
    EXPECT_GT(std::hypot(b.x-n.x, b.y-n.y), .01);
    EXPECT_NEAR(a.x-n.x, sample.second * (b.x-n.x), 1e-10) << sample.first;
    EXPECT_NEAR(a.y-n.y, sample.second * (b.y-n.y), 1e-10) << sample.first;
  }
}

TEST(Core, RobotFovCanBeNarrowHardOrOmnidirectional) {
  Config hard; hard.robot_fov_deg = 180; hard.robot_fov_fade_deg = 0;
  Config all; all.robot_fov_deg = 360;
  for (double angle : {90.0, 90.01, -90.0, -90.01}) {
    auto q = robot_at(angle);
    auto limited = integrate(q, hard), full = integrate(q, all);
    q.modifiers[0].robot_scale = 0; auto near = integrate(q, hard);
    ASSERT_TRUE(limited.success); ASSERT_TRUE(full.success); ASSERT_TRUE(near.success);
    const auto expected = std::abs(angle) <= 90 ? full.influences[0] : near.influences[0];
    EXPECT_EQ(limited.influences[0], expected);
  }
  all.robot_fov_fade_deg = 180; // A full circle has no visual edges.
  auto q = robot_at(180); auto full = integrate(q, all);
  all.robot_fov_fade_deg = 0; auto nofade = integrate(q, all);
  ASSERT_TRUE(full.success); ASSERT_TRUE(nofade.success);
  EXPECT_EQ(full.influences, nofade.influences);
  Config narrow; narrow.robot_fov_deg = 40; narrow.robot_fov_fade_deg = 5;
  q = robot_at(17.5); auto limited = integrate(q, narrow);
  auto reference = integrate(q, all);
  q.modifiers[0].robot_scale = 0; auto near = integrate(q, narrow);
  ASSERT_TRUE(limited.success); ASSERT_TRUE(reference.success); ASSERT_TRUE(near.success);
  EXPECT_NEAR(limited.influences[0].force.x-near.influences[0].force.x,
    .5*(reference.influences[0].force.x-near.influences[0].force.x), 1e-10);
}

TEST(Core, FovUsesWrappedPreStepYawEvenWhenVelocityDisagrees) {
  auto q = robot_at(0, 1.2, 3.13);
  q.current_agents.agents[0].velocity.linear.x = 1; // Velocity faces away from visual yaw.
  q.robots.agents[0].position.position.x = 10 + 1.2*std::cos(-3.13);
  q.robots.agents[0].position.position.y = 10 + 1.2*std::sin(-3.13);
  Config narrow; narrow.robot_fov_deg = 20; narrow.robot_fov_fade_deg = 0;
  Config all; all.robot_fov_deg = 360;
  auto limited = integrate(q, narrow), full = integrate(q, all);
  ASSERT_TRUE(limited.success); ASSERT_TRUE(full.success);
  EXPECT_EQ(limited.influences, full.influences);
}

TEST(Core, StoppedPedestrianRetainsYawAcrossComputeSteps) {
  auto q = robot_at(0, 1.2, 1.1); q.modifiers[0].speed_scale = 0;
  const auto before = q.current_agents.agents[0];
  for (int i = 0; i < 3; ++i) {
    auto result = integrate(q, {}); ASSERT_TRUE(result.success);
    const auto & person = result.updated_agents.agents[0];
    EXPECT_DOUBLE_EQ(person.yaw, before.yaw);
    EXPECT_DOUBLE_EQ(person.velocity.linear.x, 0);
    EXPECT_DOUBLE_EQ(person.velocity.linear.y, 0);
    EXPECT_DOUBLE_EQ(person.angular_vel, 0);
    EXPECT_NEAR(person.position.orientation.z, std::sin(before.yaw/2), 1e-10);
    q.current_agents = result.updated_agents;
    q.robots.header.stamp = q.current_agents.header.stamp;
  }
}

TEST(Core, RobotBehindStillHasNearRepulsionWhenSocialScaleIsZero) {
  auto q = robot_at(180, .9);
  auto original = integrate(q, {}); ASSERT_TRUE(original.success);
  const auto & f = original.influences[0];
  const double expected = 10 * std::exp((.1 - f.clearance) / .2);
  EXPECT_NEAR(f.force.x, expected, 1e-10);
  EXPECT_NEAR(f.force.y, 0, 1e-10);
  EXPECT_GT(f.force.x, 3);
  q.modifiers[0].robot_scale = 0; q.modifiers[0].space_scale = 4;
  auto disabled = integrate(q, {}); ASSERT_TRUE(disabled.success);
  EXPECT_EQ(original.influences, disabled.influences);
}

TEST(Core, RobotScaleChangesOnlyVisibleSocialContribution) {
  auto q = robot_at(95);
  auto normal = integrate(q, {}); ASSERT_TRUE(normal.success);
  q.modifiers[0].robot_scale = 0; auto near = integrate(q, {}); ASSERT_TRUE(near.success);
  q.modifiers[0].robot_scale = 4; auto amplified = integrate(q, {}); ASSERT_TRUE(amplified.success);
  EXPECT_NEAR(amplified.influences[0].force.x-near.influences[0].force.x,
    4*(normal.influences[0].force.x-near.influences[0].force.x), 1e-10);
  EXPECT_NEAR(amplified.influences[0].force.y-near.influences[0].force.y,
    4*(normal.influences[0].force.y-near.influences[0].force.y), 1e-10);
}

TEST(Core, VisibilityIsIndependentForEveryRobotPair) {
  auto q = robot_at(0);
  auto rear = q.robots.agents[0]; rear.id = -2; rear.name = "robot_2";
  rear.position.position.x = 8.8; q.robots.agents.push_back(rear);
  auto both = integrate(q, {}); ASSERT_TRUE(both.success); ASSERT_EQ(both.influences.size(), 2u);
  for (int i = 0; i < 2; ++i) {
    auto one = q; one.robots.agents = {q.robots.agents[i]};
    auto only = integrate(one, {}); ASSERT_TRUE(only.success);
    const auto found = std::find_if(both.influences.begin(), both.influences.end(),
      [&](const auto & influence) {return influence.robot_name == only.influences[0].robot_name;});
    ASSERT_NE(found, both.influences.end()); EXPECT_EQ(*found, only.influences[0]);
  }
}

TEST(Core, FovDoesNotChangeHumanHumanInteractionsOrEmptyRobotMotion) {
  auto q = fixture(); q.robots.agents.clear();
  auto other = q.current_agents.agents[0]; other.id = 2; other.name = "other";
  other.position.position.x = 8.8; q.current_agents.agents.push_back(other);
  auto modifier = q.modifiers[0]; modifier.agent_id = 2; q.modifiers.push_back(modifier);
  Config narrow; narrow.robot_fov_deg = 1; narrow.robot_fov_fade_deg = 0;
  Config all; all.robot_fov_deg = 360;
  auto a = integrate(q, narrow), b = integrate(q, all);
  ASSERT_TRUE(a.success); ASSERT_TRUE(b.success);
  EXPECT_EQ(a.updated_agents, b.updated_agents);
}

TEST(Core, CoincidentRobotRemainsFiniteAndVisible) {
  auto q = robot_at(0, 0, 2.0);
  Config narrow; narrow.robot_fov_deg = 1; narrow.robot_fov_fade_deg = 0;
  Config all; all.robot_fov_deg = 360;
  auto a = integrate(q, narrow), b = integrate(q, all);
  ASSERT_TRUE(a.success); ASSERT_TRUE(b.success);
  EXPECT_EQ(a.influences, b.influences);
}

TEST(Core, InvalidFovConfigurationFailsAtomically) {
  auto q = fixture();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto & values : std::vector<std::pair<double, double>>{
      {0, 0}, {-1, 0}, {361, 0}, {nan, 0}, {inf, 0}, {200, -1}, {200, 101}, {200, nan}, {200, inf}}) {
    Config cfg; cfg.robot_fov_deg = values.first; cfg.robot_fov_fade_deg = values.second;
    auto result = integrate(q, cfg);
    EXPECT_FALSE(result.success); EXPECT_FALSE(result.error.empty());
    EXPECT_EQ(result.updated_agents, q.current_agents); EXPECT_TRUE(result.influences.empty());
    EXPECT_EQ(result.epoch, q.epoch); EXPECT_EQ(result.step, q.step);
  }
}
