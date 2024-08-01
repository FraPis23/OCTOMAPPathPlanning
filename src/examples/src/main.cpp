 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/spaces/RealVectorStateSpace.h>
 #include <ompl/geometric/planners/rrt/RRTConnect.h>
 #include <ompl/geometric/SimpleSetup.h>

 #include <ompl/config.h>
 #include <iostream>
  
 namespace ob = ompl::base;
 namespace og = ompl::geometric;
  
 bool isStateValid(const ob::State *state)
 {
     // cast the abstract state type to the type we expect
     const auto *se3state = state->as<ob::RealVectorStateSpace::StateType>();
  
     // check validity of state defined by pos & rot
  
  
     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
     return true;
 }
  
 void plan()
 {
     // construct the state space we are planning in
     auto space(std::make_shared<ob::RealVectorStateSpace>(3));
  
     // set the bounds for the R^3 part of SE(3)
     ob::RealVectorBounds bounds(3);
     bounds.setLow(-1);
     bounds.setHigh(1);
  
     space->setBounds(bounds);
  
     // construct an instance of  space information from this state space
     auto si(std::make_shared<ob::SpaceInformation>(space));
  
     // set state validity checking for this space
     si->setStateValidityChecker(isStateValid);
  
     // create a random start state
     ob::ScopedState<> start(space);
     start.random();
  
     // create a random goal state
     ob::ScopedState<> goal(space);
     goal.random();
  
     // create a problem instance
     auto pdef(std::make_shared<ob::ProblemDefinition>(si));
  
     // set the start and goal states
     pdef->setStartAndGoalStates(start, goal);
  
     // create a planner for the defined space
     auto planner(std::make_shared<og::RRTConnect>(si));
  
     // set the problem we are trying to solve for the planner
     planner->setProblemDefinition(pdef);
  
     // perform setup steps for the planner
     planner->setup();
  
  
     // print the settings for this space
     si->printSettings(std::cout);
  
     // print the problem settings
     pdef->print(std::cout);
  
     // attempt to solve the problem within one second of planning time
     ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
  
     if (solved)
     {
         // get the goal representation from the problem definition (not the same as the goal state)
         // and inquire about the found path
         ob::PathPtr path = pdef->getSolutionPath();
         std::cout << "Found solution:" << std::endl;
         auto states = path->getSpaceInformation();

         for (const auto &state: states){

         }
  
         // print the path to screen
         path->print(std::cout);
     }
     else
         std::cout << "No solution found" << std::endl;
 }
  
 int main(int /*argc*/, char ** /*argv*/)
 {
     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
  
     plan();
  
     return 0;
 }