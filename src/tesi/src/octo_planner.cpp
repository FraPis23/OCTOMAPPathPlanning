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

#include <iostream>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using std::placeholders::_1;

bool unknownAsOccupied = false; // treat unknown space as occupied
float maxDist = 1; // the max distance at which distance computations are clamped

class RRTWithSmoothing : public ompl::geometric::RRT
{
public:
    RRTWithSmoothing(const ompl::base::SpaceInformationPtr &si) : ompl::geometric::RRT(si)
    {
    }

    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override
    {
        // Esegui la normale pianificazione con RRT
        ompl::base::PlannerStatus result = ompl::geometric::RRT::solve(ptc);

        // Controlla se è stata trovata una soluzione valida
        if (result == ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            // Ottieni il percorso generato
            auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef_->getSolutionPath());

            // Se il percorso è valido, applica il PathSimplifier
            if (path)
            {
                // Creiamo un PathSimplifier
                ompl::geometric::PathSimplifier simplifier(si_);

                simplifier.shortcutPath(*path); // Shortcutting
                // Applichiamo la semplificazione massima (shortcutting, etc.)
                simplifier.simplifyMax(*path);

                // Interpoliamo il percorso per renderlo più fluido
                path->interpolate(100);

                // Aggiorniamo il percorso semplificato nella problem definition
                pdef_->clearSolutionPaths(); // Rimuove il percorso originale
                pdef_->addSolutionPath(path); // Aggiunge il percorso semplificato
            }
        }

        return result;
    }
    
};

class OctoPlanner : public rclcpp::Node
{
  public:
    OctoPlanner()
    : Node("octo_planner")
    {
        octo_subs_ = this->create_subscription<octomap_msgs::msg::Octomap>("octomap_topic", 100, std::bind(&OctoPlanner::octo_callback, this, _1));
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path_topic", 100);
    }

  private:

      void octo_callback(const octomap_msgs::msg::Octomap & msg)
        {

            // // Convert the binary message back into an octomap::OcTree
            // std::vector<char> binaryData(msg.data.size());
            // std::memcpy(binaryData.data(), msg.data.data(), msg.data.size());

            // Read the binary data back into the existing OcTree
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
            bounds.setHigh(2, 3);

            space->setBounds(bounds);
        
            // construct an instance of  space information from this state space
            auto si(std::make_shared<ob::SpaceInformation>(space));

            // set state validity checking for this space
            si->setStateValidityChecker([this, &distmap](const ob::State *state) -> bool {
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
            goal[0]=14.95;
            goal[1]=1.0;
            goal[2]=2.0; 
        
            // create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        
            // set the start and goal states
            pdef->setStartAndGoalStates(start, goal);

            // create a planner for the defined space
            auto planner(std::make_shared<og::RRTstar>(si));
        
            // set the problem we are trying to solve for the planner
            planner->setProblemDefinition(pdef);
        
            // perform setup steps for the planner
            planner->setup();
        
            // print the settings for this space
            si->printSettings(std::cout);
        
            // print the problem settings
            pdef->print(std::cout);

            // attempt to solve the problem within one second of planning time
            ob::PlannerStatus solved = planner->ob::Planner::solve(1);

            
            if (solved) 
                { 
                    // Get the goal representation from the problem definition (not the same as the goal state) 
                    // and inquire about the found path 
                    og::PathGeometric *path = pdef->getSolutionPath()->as<og::PathGeometric>(); 
                
                    // Semplifica e interpola il percorso per renderlo più liscio 
                    // og::PathSimplifier simplifier(si); 
                    // simplifier.shortcutPath(*path);          // Semplifica il percorso
                    // simplifier.simplifyMax(*path);          // Semplifica il percorso
                    // path->interpolate(500);                 // Interpola il percorso con 100 punti 
                    auto& states = path->getStates(); 
                
                    // Convert the OMPL path to a sequence of geometry_msgs::PoseStamped messages 
                    std::vector<geometry_msgs::msg::Pose> poses; 
                    std::cout << "Solution found" << std::endl; 
                    for (const auto &state : states) 
                    {    
                        const auto *pos = state->as<ob::RealVectorStateSpace::StateType>(); 
                        // Extract the position values from the state 
                        double x = pos->values[0]; // x-coordinate 
                        double y = pos->values[1]; // y-coordinate 
                        double z = pos->values[2]; // z-coordinate 
                
                        std::cout << "Point: (" << x <<  ", " << y << ", " << z << ");" << std::endl; 
                
                        // Convert the OMPL state to a geometry_msgs::PoseStamped message 
                        geometry_msgs::msg::Pose pose; 
                
                        // Populate the position fields with the extracted values 
                        pose.position.x = x; 
                        pose.position.y = y; 
                        pose.position.z = z; 
                
                        pose.orientation.w = 1.0; // Assuming unit quaternion for simplicity 
                
                        poses.push_back(pose); 
                    } 
                
                    // Publish the PoseArray message 
                    geometry_msgs::msg::PoseArray poseArray; 
                    poseArray.poses = poses; 
                    path_pub_->publish(poseArray); 
                } 
                else 
                { 
                    std::cout << "No solution found" << std::endl; 
                }
        };
       

         bool isStateValid(const ob::State *state, DynamicEDTOctomap &distmap)
            {
                // cast the abstract state type to the type we expect
                const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
                
                return distmap.getDistance(octomap::point3d((*pos)[0], (*pos)[1], (*pos)[2])) > 0.6;
            };
        
        
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octo_subs_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctoPlanner>());
  rclcpp::shutdown();
  return 0;
}