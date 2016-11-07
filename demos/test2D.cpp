/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

/* Author: Bryant Gipson, Mark Moll */

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/DRRT.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <ompl/tools/debug/Profiler.h>


class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr& si,std::vector< std::vector<double> > obstacles,ompl::base::StateSpacePtr space) :
        ompl::base::StateValidityChecker(si),obstacles_(obstacles),space_(space) {}

    bool isValid(const ompl::base::State* s1) const
    {
        ompl::base::ScopedState<> s(space_,s1);
        for(std::vector< std::vector<double> >::const_iterator it=obstacles_.begin(),end=obstacles_.end();it!=end;++it){
            std::vector<double> o=*it;
            if(o[0]-0.5*o[2]< (s[0])
                && (s[0])<o[0]+0.5*o[2]
                && o[1]-0.5*o[3]< (s[1])
                && (s[1])<o[1]+0.5*o[3]){
                return false;
            }
        }
        return true;
    }

    std::vector< std::vector<double> > obstacles_;
    ompl::base::StateSpacePtr space_;
};

int main(int argc, char **argv)
{
    std::vector< std::vector<double> > obstacles;
    double mo1[] = {0.,2.0,8.5,1.0};
    std::vector<double> o1(mo1, mo1 + sizeof(mo1) / sizeof(double) );
    obstacles.push_back(o1);
    double mo2[] = {-4.0,8.0,1.0,2.5};
    std::vector<double> o2(mo2, mo2 + sizeof(mo2) / sizeof(double) );
    obstacles.push_back(o2);
    double mo3[] = {-4.0,4.5,4.0,2.0};
    std::vector<double> o3(mo3, mo3 + sizeof(mo3) / sizeof(double) );
    obstacles.push_back(o3);
    double mo4[] = {7.0,3.5,2.5,11.0};
    std::vector<double> o4(mo4, mo4 + sizeof(mo4) / sizeof(double) );
    obstacles.push_back(o4);
    double mo5[] = {3.0,5.0,2.5,2.0};
    std::vector<double> o5(mo5, mo5 + sizeof(mo5) / sizeof(double) );
    obstacles.push_back(o5);
    double mo6[] = {-6.0,-4.0,3.0,5.0};
    std::vector<double> o6(mo6, mo6 + sizeof(mo6) / sizeof(double) );
    obstacles.push_back(o6);
    double mo7[] = {3.0,-5.0,2.5,5.0};
    std::vector<double> o7(mo7, mo7 + sizeof(mo7) / sizeof(double) );
    obstacles.push_back(o7);

    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
    ompl::geometric::SimpleSetup ss(space);
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(-20,20);

    ss.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
        new ValidityChecker(ss.getSpaceInformation(),obstacles,space)));

    ompl::base::ScopedState<> start(space), goal(space);
    start[0]=0;
    start[1]=0;
    goal[0]=0;
    goal[1]=8;

    ss.setStartAndGoalStates(start, goal);

    // by default, use the Benchmark class
    double runtime_limit = 2, memory_limit = 1024;
    int run_count = 30;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.05,true,true,false,false);
    ompl::tools::Benchmark b(ss, "Test2D");

    double range=1.0;

    ompl::base::PlannerPtr rrtstar(new ompl::geometric::RRTstar(ss.getSpaceInformation()));
    rrtstar->as<ompl::geometric::RRTstar>()->setName("RRT*");
    rrtstar->as<ompl::geometric::RRTstar>()->setDelayCC(false);
    rrtstar->as<ompl::geometric::RRTstar>()->setPrunedMeasure(false);
    rrtstar->as<ompl::geometric::RRTstar>()->setInformedSampling(false);
    rrtstar->as<ompl::geometric::RRTstar>()->setSampleRejection(false);
    rrtstar->as<ompl::geometric::RRTstar>()->setNewStateRejection(false);
    rrtstar->as<ompl::geometric::RRTstar>()->setAdmissibleCostToCome(false);
    rrtstar->as<ompl::geometric::RRTstar>()->setFocusSearch(false);
    rrtstar->as<ompl::geometric::RRTstar>()->setRange(range);
    b.addPlanner(rrtstar);
    ompl::base::PlannerPtr rrtsharpepsilon0c(new ompl::geometric::RRTsharp(ss.getSpaceInformation()));
    rrtsharpepsilon0c->as<ompl::geometric::RRTsharp>()->setName("RRTsharp");
    rrtsharpepsilon0c->as<ompl::geometric::RRTsharp>()->setRange(range);
    b.addPlanner(rrtsharpepsilon0c);
    ompl::base::PlannerPtr rrtsharpepsilon1c(new ompl::geometric::RRTXstatic(ss.getSpaceInformation()));
    rrtsharpepsilon1c->as<ompl::geometric::RRTXstatic>()->setName("RRTs0.01c");
    rrtsharpepsilon1c->as<ompl::geometric::RRTXstatic>()->setEpsilon(0.01);
    rrtsharpepsilon1c->as<ompl::geometric::RRTXstatic>()->setRange(range);
    b.addPlanner(rrtsharpepsilon1c);
	ompl::base::PlannerPtr drrttd(new ompl::geometric::DRRT(ss.getSpaceInformation()));
	drrttd->as<ompl::geometric::DRRT>()->setName("DRRTtd");
	drrttd->as<ompl::geometric::DRRT>()->setRange(range);
	drrttd->as<ompl::geometric::DRRT>()->setVariant(ompl::geometric::DRRT::TREE);
	drrttd->as<ompl::geometric::DRRT>()->setDelayOptimizationUntilSolution(true);
	b.addPlanner(drrttd);
	ompl::base::PlannerPtr drrtt(new ompl::geometric::DRRT(ss.getSpaceInformation()));
	drrtt->as<ompl::geometric::DRRT>()->setName("DRRTt");
	drrtt->as<ompl::geometric::DRRT>()->setRange(range);
	drrtt->as<ompl::geometric::DRRT>()->setVariant(ompl::geometric::DRRT::TREE);
	b.addPlanner(drrtt);//*
	ompl::base::PlannerPtr drrttf(new ompl::geometric::DRRT(ss.getSpaceInformation()));
	drrttf->as<ompl::geometric::DRRT>()->setName("DRRTtf0.3");
	drrttf->as<ompl::geometric::DRRT>()->setRange(range);
	drrttf->as<ompl::geometric::DRRT>()->setVariant(ompl::geometric::DRRT::TREE);
	drrttf->as<ompl::geometric::DRRT>()->setDeformationFrequency(0.3);
	b.addPlanner(drrttf);//*/
    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("test2D.log")).c_str());

    exit(0);
}
