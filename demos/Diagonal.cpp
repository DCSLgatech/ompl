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

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/DRRT.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/format.hpp>
#include <boost/math/constants/constants.hpp>
#include <fstream>

class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }

    bool isValid(const ompl::base::State *state) const override
    {
	return true;
        /*const ompl::base::RealVectorStateSpace::StateType *state2D =
            state->as<ompl::base::RealVectorStateSpace::StateType>();

        double sum = 0;
        for (unsigned i = 0; i < si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getDimension(); ++i)
            sum += state2D->values[i] * state2D->values[i];

        return sqrt(sum) > 0.1;*/
    }
};

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " dimensionOfTheProblem" << std::endl;
        exit(0);
    }
    int dim = atoi(argv[1]);

    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(dim));
    ompl::geometric::SimpleSetup ss(space);
    const ompl::base::SpaceInformationPtr &si = ss.getSpaceInformation();
    space->setBounds(-1, 1);

    ss.setStateValidityChecker(std::make_shared<ValidityChecker>(si));

    ompl::base::ScopedState<> start(space), goal(space);
    for (int i = 0; i < dim; ++i)
    {
        start[i] = -1;
        goal[i] = 1;
    }

    ss.setStartAndGoalStates(start, goal);

    // by default, use the Benchmark class
    double runtime_limit = 5, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.05, true, true, false, false);
    ompl::tools::Benchmark b(ss, "Diagonal");

    double range = 0.1 * sqrt(dim);

    auto lengthObj(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
    ompl::base::OptimizationObjectivePtr oop((0.5 / sqrt(dim)) * lengthObj);

    ss.setOptimizationObjective(oop);

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
    auto rrtX2(std::make_shared<ompl::geometric::RRTXstatic>(si));
    rrtX2->setName("RRTX0.01");
    rrtX2->setEpsilon(0.01);
    rrtX2->setRange(range);
    rrtX2->setKNearest(knn);
    b.addPlanner(rrtX2);
    auto drrtt(std::make_shared<ompl::geometric::DRRT>(si));
    drrtt->setName("DRRT0t");
    drrtt->setRange(range);
    drrtt->setVariant(ompl::geometric::DRRT::Variant::TREE);
    drrtt->setKNearest(knn);
    b.addPlanner(drrtt);
    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("Diagonal.log")).c_str());

    exit(0);
}
