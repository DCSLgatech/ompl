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
#include <ompl/geometric/planners/bitstar/BITstar.h>
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
    double runtime_limit = 20, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.05, true, true, false, false);
    ompl::tools::Benchmark b(ss, "gradientDescentApprox");

    double range = 0.05 * sqrt(dim);

    auto lengthObj(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
    ompl::base::OptimizationObjectivePtr oop((0.5 / sqrt(dim)) * lengthObj);

    ss.setOptimizationObjective(oop);

    bool knn = true;

//    auto drrtt(std::make_shared<ompl::geometric::DRRT>(si));
//    drrtt->setName("DRRT");
//    drrtt->setRange(range);
//    drrtt->setVariant(ompl::geometric::DRRT::Variant::TREE);
//    drrtt->setKNearest(knn);
//    b.addPlanner(drrtt);
//    auto drrtt2(std::make_shared<ompl::geometric::DRRT>(si));
//    drrtt2->setName("DRRT_branch");
//    drrtt2->setRange(range);
//    drrtt2->setVariant(ompl::geometric::DRRT::Variant::BRANCH);
//    drrtt2->setKNearest(knn);
//    drrtt2->setGDFlags(GD_APPROX_BRANCH);
//    b.addPlanner(drrtt2);
    auto drrtt3(std::make_shared<ompl::geometric::DRRT>(si));
    drrtt3->setName("DRRT_approx0");
    drrtt3->setRange(range);
    drrtt3->setVariant(ompl::geometric::DRRT::Variant::TREE);
    drrtt3->setKNearest(knn);
    drrtt3->setGDFlags(GD_APPROX_DESCENDANT_N);
    drrtt3->setGDMaxDepthDescendantApprox(0u);
    b.addPlanner(drrtt3);
    auto drrtt4(std::make_shared<ompl::geometric::DRRT>(si));
    drrtt4->setName("DRRT_approx1");
    drrtt4->setRange(range);
    drrtt4->setVariant(ompl::geometric::DRRT::Variant::TREE);
    drrtt4->setKNearest(knn);
    drrtt4->setGDFlags(GD_APPROX_DESCENDANT_N);
    drrtt4->setGDMaxDepthDescendantApprox(1u);
    b.addPlanner(drrtt4);
    auto drrtt5(std::make_shared<ompl::geometric::DRRT>(si));
    drrtt5->setName("DRRT_approx5");
    drrtt5->setRange(range);
    drrtt5->setVariant(ompl::geometric::DRRT::Variant::TREE);
    drrtt5->setKNearest(knn);
    drrtt5->setGDFlags(GD_APPROX_DESCENDANT_N);
    drrtt5->setGDMaxDepthDescendantApprox(5u);
    b.addPlanner(drrtt5);
    auto drrtt6(std::make_shared<ompl::geometric::DRRT>(si));
    drrtt6->setName("DRRT_approx10");
    drrtt6->setRange(range);
    drrtt6->setVariant(ompl::geometric::DRRT::Variant::TREE);
    drrtt6->setKNearest(knn);
    drrtt6->setGDFlags(GD_APPROX_DESCENDANT_N);
    drrtt6->setGDMaxDepthDescendantApprox(10u);
    b.addPlanner(drrtt6);
    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("gradient_descent_approx.log")).c_str());

    exit(0);
}
