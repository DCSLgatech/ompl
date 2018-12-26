/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Georgia Institute of Technology
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Florian Hauer */

#include <ompl/base/spaces/TestSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/DRRT.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/format.hpp>
#include <boost/math/constants/constants.hpp>
#include <fstream>


using namespace ompl::base;

class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }

    bool isValid(const ompl::base::State *state) const override
    {
    	return true;
//        const ompl::base::RealVectorStateSpace::StateType *state2D =
//            state->as<ompl::base::RealVectorStateSpace::StateType>();
//
//        double sum = 0;
//        for (unsigned i = 0; i < si_->getStateSpace()->as<ompl::base::TestSpace>()->getDimension(); i+=2)
//            sum += state2D->values[i] * state2D->values[i];
//
//        return sqrt(sum) > 3;
    }
};

class MotionValidityChecker : public MotionValidator
        {
        public:
			MotionValidityChecker(SpaceInformation *si) : MotionValidator(si)
            {
            }
			MotionValidityChecker(const SpaceInformationPtr &si) : MotionValidator(si)
            {
            }
            bool checkMotion(const State *s1, const State *s2) const override
			{
            	return true;
			}
            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override
			{
            	return true;
			}
        };

inline bool exists_file (const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

std::string uniqueName(const std::string &stem, const std::string &ext) {

    std::ostringstream possibleName;
    possibleName.clear();
    possibleName << stem << "." << ext;
    for (int i=1; exists_file(possibleName.str()); ++i) {
        possibleName.str(std::string());
        possibleName << stem << "_" << i << "_." << ext;
    }
    return possibleName.str();
}

int main(int argc, char **argv)
{
    int dim = 3;
    auto space(std::make_shared<ompl::base::TestSpace>(dim));

    space->sanityChecks();
    space->setBounds(50, 5, 1);

//    return 0;

    ompl::base::SpaceInformationPtr si(std::make_shared<ompl::base::SpaceInformation>(space));

    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));
    si->setMotionValidator(std::make_shared<MotionValidityChecker>(si));

    int j = 0;
    ompl::base::ScopedState<> start(space), goal(space);
	for (int i = 0; i < dim; ++i)
	{
		start[2*i] = -10 + j++;
		goal[2*i] = 10;
		start[2*i+1] = 0;
		goal[2*i+1] = 0;
	}
	start[0] = -13.1;
	start[1] = -0.1;
	start[2] = -22.3;
	start[3] = -0.2;
	start[4] = -13.7;
	start[5] =  0.1;

	goal[0] = 15.1;
	goal[1] = 0.1;
	goal[2] = 12.1;
	goal[3] = 0.2;
	goal[4] = 23.7;
	goal[5] = 0.1;

//	goal[6] = -13.1;
//	goal[7] = -0.1;
//	goal[8] = -22.3;
//	goal[9] = -0.2;
//	goal[10] = -13.7;
//	goal[11] =  0.1;
//
//	start[6] = 15.1;
//	start[7] = 0.1;
//	start[8] = 12.1;
//	start[9] = 0.2;
//	start[10] = 23.7;
//	start[11] = 0.1;


    auto opt_(std::make_shared<PathLengthOptimizationObjective>(si));
    ompl::base::ScopedState<> middle(space);
    ompl::base::ScopedState<> temp(space);
    middle[0] = 2;
    middle[1] = 3;
    middle[2] = 1;
    middle[3] = 1;
    middle[2] = 1;
    middle[3] = 5;

    State *s,*g,*m;
    s = start.get();
    g= goal.get();
    m = middle.get();

    std::cout << "gradient descent" << std::endl;
    std::cout << "initial cost " << opt_->combineCosts(opt_->motionCost(s, m), opt_->motionCost(m, g)) << std::endl;
    Cost c = opt_->motionCost(s, g);
    std::cout << "start goal cost opt " << c << std::endl;
    std::cout << "start goal cost space " << space->distance(s, g) << std::endl;

//    for(int i=0;i<=100;++i)
//    {
//    	space->interpolate(s,g,0.01*i,m);
//    }

//    return 0;

    int it =0;
    double delta = 0.1;
    while(it < 20)
    {
    	++it;
        std::vector<double> grad1,grad2;
        grad1=opt_->gradientwrts2(s,m);
        grad2=opt_->gradientwrts1(m,g);
        for(unsigned int i=0;i<grad1.size();++i){
            grad1[i]+=grad2[i];
        }
        for(unsigned int i=0;i<grad1.size();++i){
        	temp[i]=middle[i] - delta*grad1[i];
		}
        if( opt_->isCostBetterThan(
				opt_->combineCosts(opt_->motionCost(s, temp.get()), opt_->motionCost(temp.get(), g)),
        		opt_->combineCosts(opt_->motionCost(s, m), opt_->motionCost(m, g))
						))
        {
            for(unsigned int i=0;i<grad1.size();++i){
            	middle[i]=middle[i] - delta*grad1[i];
    		}
        }else{
        	delta *= 0.9;
        }

        std::cout << "cost at iteration " << it << " : " << opt_->combineCosts(opt_->motionCost(s, m), opt_->motionCost(m, g)) << std::endl;
    }

    std::cout << "gradient descent over" << std::endl;

    std::cout << "start goal distance " << space->distance(start.get(),goal.get()) << std::endl;

//    return 0;

	ompl::geometric::SimpleSetup ss(si);
	ss.setStartAndGoalStates(start, goal);

	// by default, use the Benchmark class
	double runtime_limit = 40, memory_limit = 1024;
	int run_count = 1;
	ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.05, true, true, false, false);
	ompl::tools::Benchmark b(ss, "testSpaceDemo");

	double range = 3.5;

	bool knn = true;

	auto rrtstar(std::make_shared<ompl::geometric::RRTstar>(si));
	rrtstar->setName("RRT*");
	rrtstar->setDelayCC(false);
	rrtstar->setFocusSearch(false);
	rrtstar->setRange(range);
	rrtstar->setKNearest(knn);
	b.addPlanner(rrtstar);
	auto rrtsh(std::make_shared<ompl::geometric::RRTsharp>(si));
	rrtsh->setRange(range);
	rrtsh->setKNearest(knn);
	b.addPlanner(rrtsh);
//	auto rrtX2(std::make_shared<ompl::geometric::RRTXstatic>(si));
//	rrtX2->setName("RRTX0.01");
//	rrtX2->setEpsilon(0.01);
//	rrtX2->setRange(range);
//	rrtX2->setKNearest(knn);
//	b.addPlanner(rrtX2);
//	auto drrtg(std::make_shared<ompl::geometric::DRRT>(si));
//	drrtg->setName("DRRT0.01g");
//	drrtg->setEpsilon(0.0001);
//	drrtg->setRange(range);
//	drrtg->setVariant(ompl::geometric::DRRT::Variant::GOAL);
//	drrtg->as<ompl::geometric::DRRT>()->setDelayOptimizationUntilSolution(true);
//	drrtg->setKNearest(knn);
//	b.addPlanner(drrtg);
//	auto drrtb(std::make_shared<ompl::geometric::DRRT>(si));
//	drrtb->setName("DRRT0.01b");
//	drrtb->setEpsilon(0.0001);
//	drrtb->setRange(range);
//	drrtb->setVariant(ompl::geometric::DRRT::Variant::BRANCH);
//	drrtb->as<ompl::geometric::DRRT>()->setDelayOptimizationUntilSolution(true);
//	drrtb->setKNearest(knn);
//	b.addPlanner(drrtb);
//	auto drrtt1(std::make_shared<ompl::geometric::DRRT>(si));
//	drrtt1->setName("DRRT0.01t");
//	drrtt1->setRange(range);
//	drrtt1->setEpsilon(0.01);
//	drrtt1->setVariant(ompl::geometric::DRRT::Variant::TREE);
////	drrtt1->as<ompl::geometric::DRRT>()->setDelayOptimizationUntilSolution(true);
//	drrtt1->setKNearest(knn);
//	b.addPlanner(drrtt1);
//	auto drrtt(std::make_shared<ompl::geometric::DRRT>(si));
//	drrtt->setName("DRRT0t");
//	drrtt->setRange(range);
//	drrtt->setVariant(ompl::geometric::DRRT::Variant::TREE);
//	drrtt->as<ompl::geometric::DRRT>()->setDelayOptimizationUntilSolution(true);
//	drrtt->setKNearest(knn);
//	b.addPlanner(drrtt);
    auto drrttsn(std::make_shared<ompl::geometric::DRRT>(si));
    drrttsn->setName("drrttsn");
    drrttsn->setRange(range);
    drrttsn->setVariant(ompl::geometric::DRRT::Variant::TREE);
    drrttsn->setKNearest(knn);
    drrttsn->setSingleNodeUpdate(true);
    drrttsn->as<ompl::geometric::DRRT>()->setDelayOptimizationUntilSolution(true);
    drrttsn->setGDFlags(GD_IF_EVERY_N | GD_MAX_N_DEPTH | GD_APPROX_DESCENDANT_N);
    drrttsn->setGDMaxDepthDescendantApprox(0u);
    drrttsn->setGDMaxDepth(2u);
    b.addPlanner(drrttsn);

	b.benchmark(request);
	b.saveResultsToFile(uniqueName("testSpaceDemo", "log").c_str());

//	ss.setPlanner(rrtstar);
//	ss.solve(30);

    exit(0);
}
