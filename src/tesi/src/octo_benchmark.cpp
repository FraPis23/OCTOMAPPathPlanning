#include <memory>

#include <rclcpp/rclcpp/rclcpp.hpp>
#include <geometry_msgs/geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <ompl/tools/benchmark/Benchmark.h>

#include <iostream>

//RRT
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

//EST
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>

//FMT
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>

//KPIECE1
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

//PRM
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>

//RLRT
#include <ompl/geometric/planners/rlrt/RLRT.h>
#include <ompl/geometric/planners/rlrt/BiRLRT.h>

//SBL
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>

//Other
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

using std::placeholders::_1;

bool unknownAsOccupied = false; // treat unknown space as occupied
float maxDist = 1; // the max distance at which distance computations are clamped

class OctoPlanner : public rclcpp::Node
{
  public:
    OctoPlanner()
    : Node("octo_planner")
    {
        octo_benchmark_ = this->create_subscription<octomap_msgs::msg::Octomap>("octomap_topic", 100, std::bind(&OctoPlanner::octo_callback, this, _1));
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path_topic", 100);
    }

  private:

      void octo_callback(const octomap_msgs::msg::Octomap & msg)
        {
            octomap::OcTree* tree_ = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(msg));

            // Log the received OctoMap message
            RCLCPP_INFO(this->get_logger(), "Received OctoMap message.");

            // Create an instance of the DynamicEDTOctomap class
            double minX, minY, minZ, maxX, maxY, maxZ;

            tree_->getMetricMin(minX, minY, minZ);
            octomap::point3d min = octomap::point3d(minX, minY, minZ);
            tree_->getMetricMax(maxX, maxY, maxZ);
            octomap::point3d max = octomap::point3d(maxX, maxY, maxZ);

            DynamicEDTOctomap distmap(maxDist, tree_, min, max, unknownAsOccupied);
            distmap.update();

            // construct the state space we are planning in
            auto space(std::make_shared<ob::RealVectorStateSpace>(3));
        
            // set the bounds for the R^3 part of SE(3)
            ob::RealVectorBounds bounds(3);
            bounds.setLow(0, min.x());
            bounds.setHigh(0, max.x());
            bounds.setLow(1, min.y());
            bounds.setHigh(1, 5);
            bounds.setLow(2, min.z());
            bounds.setHigh(2, 2);

            space->setBounds(bounds);
        
            og::SimpleSetup ss(space);

            // set state validity checking for this space
            ss.setStateValidityChecker([this, &distmap](const ob::State *state) -> bool {
                return isStateValid(state, distmap);
             });

            // create a random start state
            ob::ScopedState<> start(space);
            // start.random();
            start[0]=-3.0;
            start[1]=3.0;
            start[2]=1.0;
        
            // create a random goal state
            ob::ScopedState<> goal(space);
            // goal.random();
            goal[0]=15.0;
            goal[1]=1.0;
            goal[2]=2.0; 
        
            // set the start and goal states
            ss.setStartAndGoalStates(start, goal);
            ss.solve();

            // create a benchmark class:
            ot::Benchmark b(ss, "Benchmark");

            // Add a planner to the benchmark

            //auto rrtstar = ob::PlannerPtr(new og::RRTstar(ss.getSpaceInformation()));
            //rrtstar->as<og::RRTstar>()->setGoalBias(0.2);
            //rrtstar->as<og::RRTstar>()->setFocusSearch(false);
            //rrtstar->as<og::RRTstar>()->setSampleRejection(false);
            //b.addPlanner(rrtstar);
            //b.addPlanner(ob::PlannerPtr(new og::RRT(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::LazyRRT(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::RRTstar(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::RRTConnect(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::EST(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::LBTRRT(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::LazyLBTRRT(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::RRTsharp(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::SORRTstar(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::InformedRRTstar(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::TRRT(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::BiTRRT(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::pRRT(ss.getSpaceInformation())));

            //b.addPlanner(ob::PlannerPtr(new og::EST(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::BiEST(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::ProjEST(ss.getSpaceInformation())));

            //b.addPlanner(ob::PlannerPtr(new og::FMT(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::BFMT(ss.getSpaceInformation())));

            //b.addPlanner(ob::PlannerPtr(new og::KPIECE1(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::BKPIECE1(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::LBKPIECE1(ss.getSpaceInformation())));

            //b.addPlanner(ob::PlannerPtr(new og::PRM(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::LazyPRM(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::SPARS(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::SPARStwo(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::PRMstar(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::LazyPRMstar(ss.getSpaceInformation())));

            //b.addPlanner(ob::PlannerPtr(new og::RLRT(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::BiRLRT(ss.getSpaceInformation())));

            //b.addPlanner(ob::PlannerPtr(new og::SBL(ss.getSpaceInformation())));
            //b.addPlanner(ob::PlannerPtr(new og::pSBL(ss.getSpaceInformation())));

            b.addPlanner(ob::PlannerPtr(new og::CForest(ss.getSpaceInformation())));
            b.addPlanner(ob::PlannerPtr(new og::PDST(ss.getSpaceInformation())));
            b.addPlanner(ob::PlannerPtr(new og::STRIDE(ss.getSpaceInformation())));

            b.setPostRunEvent([this, &distmap](const ob::PlannerPtr &planner, const ot::Benchmark::RunProperties &run) {
              auto *path = planner->getProblemDefinition()->getSolutionPath()->as<og::PathGeometric>();
              double minClearance = std::numeric_limits<double>::max();
              double stepSize = 0.001;  // Passo per l'interpolazione lungo il segmento

              for (size_t i = 0; i < path->getStateCount() - 1; ++i) {
                  const ob::State *state1 = path->getState(i);
                  const ob::State *state2 = path->getState(i + 1);

                  // Interpolazione e calcolo clearance sui segmenti
                  for (double t = 0; t <= 1.0; t += stepSize) {
                      ob::State *interpolatedState = path->getSpaceInformation()->allocState();
                      path->getSpaceInformation()->getStateSpace()->interpolate(state1, state2, t, interpolatedState);
                      double clearance = this->calculateClearance(interpolatedState, distmap);
                      if (clearance < minClearance)
                          minClearance = clearance;
                      path->getSpaceInformation()->freeState(interpolatedState);
                  }
              }

              const_cast<ot::Benchmark::RunProperties&>(run).insert({"clearance REAL", std::to_string(minClearance)});
              });

            // b.addPlannerAllocator(std::bind(&myConfiguredPlanner, std::placeholders::_1));

            ot::Benchmark::Request req;
            req.maxTime = 0.5;
            req.maxMem = 300.0;
            req.runCount = 100;
            req.displayProgress = true;
            b.benchmark(req);

            // This will generate a file of the form ompl_host_time.log
            b.saveResultsToFile("./benchmark.log");
        };

        double calculateClearance(const ob::State *state, DynamicEDTOctomap &distmap)
        {
            const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
            return distmap.getDistance(octomap::point3d((*pos)[0], (*pos)[1], (*pos)[2]));
        };

         
        bool isStateValid(const ob::State *state, DynamicEDTOctomap &distmap)
            {
                // cast the abstract state type to the type we expect
                const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

                return distmap.getDistance(octomap::point3d((*pos)[0], (*pos)[1], (*pos)[2])) > 0.6;
            };
 
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octo_benchmark_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctoPlanner>());
  rclcpp::shutdown();
  return 0;
}