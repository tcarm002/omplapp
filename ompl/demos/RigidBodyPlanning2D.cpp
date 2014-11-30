#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

// assume the following namespace definitions
namespace ob = ompl::base;
namespace og = ompl::geometric;

// state validity checking function
bool isStateValid(const ob::State *state)
{
	// const ob::CompoundStateSpace::StateType *space = state->as<ob::CompoundStateSpace::StateType()>;

    // cast the abstract state type to the type we expect
    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(1);

    // extract the second component of the state and cast it to what we expect
    const ob::SO2StateSpace::StateType *rot = se2state->as<ob::SO2StateSpace::StateType>(2);

	// check validity of state definied by time
	// const ob::TimeStateSpace::StateType *time = state->as<ob::TimeStateSpace::StateType();
	const ob::TimeStateSpace::StateType *time = se2state->as<ob::TimeStateSpace::StateType>(0);
	bool isValid = true;
	// check against previous state
	

    // check validity of state defined by pos & rot
	// double x = se2state->getX();
	// double y = se2state->getY();
	
	// check clearance to obstacles
	// if (this->clearance(state) > 0.0;

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return (const void*)rot != (const void*)pos;
}

void planWithSimpleSetup(void)
{
    // construct the state space we are planning in
    // ob::StateSpacePtr space(new ob::SE2StateSpace());
	// create a compund state space with both SE(2) and Time, each with equal weight 1.0

	ob::StateSpacePtr space = ob::StateSpacePtr(new ob::CompoundStateSpace());
    space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::TimeStateSpace()), 1.0);
    space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::SE2StateSpace()), 1.0);
    space->as<ob::CompoundStateSpace>()->lock();

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

	// space->as<ob::SE2StateSpace>()->setBounds(bounds);
	space->as<ob::CompoundStateSpace>()->as<ob::SE2StateSpace>(1)->setBounds(bounds);

	// set the bounds for Time
	ob::RealVectorBounds tBounds(2);
	tBounds.setLow(0);
	tBounds.setHigh(1);
    
	space->as<ob::CompoundStateSpace>()->as<ob::TimeStateSpace>(0)->setBounds(0, 1);

    // define a simple setup class
	// create an instance of ompl::geometric::SimpleSetup
	// instances of ompl::base::SpaceInformation and ompl::base::ProblemDefinition are created internally
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, _1));

    // create a random start state
    ob::ScopedState<> start(space);
	
    start.random();

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
     ss.setup();
     ss.print();

	// now try to solve the problem
	// trigger a call to ompl::geomatric::SimpleSetup::setup() and create a default instance of a planner (since we have not specified one)
	// ompl::base::Planner::setup() is called, which calls ompl::base::SpaceInformation::setup()
	// chain of calls leads to computation of runtime params
	// This call returns a value from ompl::base::PlannerStatus which describes whether a solution has been found within the specified amount of time
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
		// solution was found
        std::cout << "Found solution:" << std::endl;
      
		// optional, simplify
        // ss.simplifySolution();
		// print the path to screen
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
	std::cout << std::endl << std::endl;

    planWithSimpleSetup();

    return 0;
}

