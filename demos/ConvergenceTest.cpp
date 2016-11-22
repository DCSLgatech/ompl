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

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/DRRT.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/MotionValidator.h>
#include "ompl/util/Time.h"

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>



class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si): ompl::base::StateValidityChecker(si.get()) {}

    bool isValid(const ompl::base::State* s1) const
    {
        return true;
    }
};

class myMotionValidator : public ompl::base::MotionValidator
{
public:
    myMotionValidator(const ompl::base::SpaceInformationPtr &si): ompl::base::MotionValidator(si.get()){}
    virtual bool 	checkMotion (const ompl::base::State *s1, const ompl::base::State *s2) const {return true;}
    virtual bool 	checkMotion (const ompl::base::State *s1, const ompl::base::State *s2, std::pair< ompl::base::State *, double > &lastValid) const {return true;}
};

int main(int argc, char **argv)
{
    if(argc!=3){
        std::cout << "Usage: " << argv[0] << " maxDim nbRun" <<std::endl;
        exit(0);
    }
    int maxDim = atoi(argv[1]);
    int nbRun  = atoi(argv[2]);
    double maxTime=1000.0;
    double threshold=1.025;

    ompl::msg::noOutputHandler();

    for(int p=0;p<9;++p){

        switch(p){
            case 0 : std::cout << "RRT*" << std::endl;break;
            case 1 : std::cout << "RRT#" << std::endl;break;
            case 2 : std::cout << "DRRT" << std::endl;break;
            case 3 : std::cout << "RRT*RS" << std::endl;break;
            case 4 : std::cout << "RRT#RS" << std::endl;break;
            case 5 : std::cout << "DRRTRS" << std::endl;break;
            case 6 : std::cout << "RRT*IS" << std::endl;break;
            case 7 : std::cout << "RRT#IS" << std::endl;break;
            case 8 : std::cout << "DRRTIS" << std::endl;break;
        }

        for(int dim=1;dim<=maxDim;++dim){
            std::vector<double> times;
            std::vector<int> iterations;
            for(int r=0;r<nbRun;++r){
                //Setup space
                ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dim));
                ompl::geometric::SimpleSetup ss(space);
                space->as<ompl::base::RealVectorStateSpace>()->setBounds(-1,1);

                //Setup Validity checker for nodes and edges
                ss.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new ValidityChecker(ss.getSpaceInformation())));
                ss.getSpaceInformation()->setMotionValidator(ompl::base::MotionValidatorPtr(new myMotionValidator(ss.getSpaceInformation())));
                ss.getSpaceInformation()->setup();

                //Define start and goal
                ompl::base::ScopedState<> start(space), goal(space);
                for(int i=0;i<dim;++i){
                    start[i]=-1;
                    goal[i]=1;
                }

                ss.setStartAndGoalStates(start, goal);

                //Define Optimization function such that optimal cost is 1
                double range=0.1*sqrt(dim);

                ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::PathLengthOptimizationObjective(ss.getSpaceInformation()));
                ompl::base::OptimizationObjectivePtr oop;
                oop=(0.5/sqrt(dim))*lengthObj;
                oop->setCostThreshold(ompl::base::Cost(threshold));

                ss.setOptimizationObjective(oop);

                ompl::base::PlannerPtr planner;

                switch(p){
                    case 0 : planner=ompl::base::PlannerPtr(new ompl::geometric::RRTstar(ss.getSpaceInformation()));
                             planner->as<ompl::geometric::RRTstar>()->setRange(range);
                             break;
                    case 1 : planner=ompl::base::PlannerPtr(new ompl::geometric::RRTsharp(ss.getSpaceInformation()));
                             planner->as<ompl::geometric::RRTsharp>()->setRange(range);
                             break;
                    case 2 : planner=ompl::base::PlannerPtr(new ompl::geometric::DRRT(ss.getSpaceInformation()));
                             planner->as<ompl::geometric::DRRT>()->setRange(range);
                             break;
                    case 3 : planner=ompl::base::PlannerPtr(new ompl::geometric::RRTstar(ss.getSpaceInformation()));
                             planner->as<ompl::geometric::RRTstar>()->setRange(range);
                             planner->as<ompl::geometric::RRTstar>()->setSampleRejection(true);
                             break;
                    case 4 : planner=ompl::base::PlannerPtr(new ompl::geometric::RRTsharp(ss.getSpaceInformation()));
                             planner->as<ompl::geometric::RRTsharp>()->setRange(range);
                             planner->as<ompl::geometric::RRTsharp>()->setSampleRejection(true);
                             break;
                    case 5 : planner=ompl::base::PlannerPtr(new ompl::geometric::DRRT(ss.getSpaceInformation()));
                             planner->as<ompl::geometric::DRRT>()->setRange(range);
                             planner->as<ompl::geometric::DRRT>()->setSampleRejection(true);
                             break;
                    case 6 : planner=ompl::base::PlannerPtr(new ompl::geometric::RRTstar(ss.getSpaceInformation()));
                             planner->as<ompl::geometric::RRTstar>()->setRange(range);
                             planner->as<ompl::geometric::RRTstar>()->setInformedSampling(true);
                             break;
                    case 7 : planner=ompl::base::PlannerPtr(new ompl::geometric::RRTsharp(ss.getSpaceInformation()));
                             planner->as<ompl::geometric::RRTsharp>()->setRange(range);
                             planner->as<ompl::geometric::RRTsharp>()->setInformedSampling(true);
                             break;
                    case 8 : planner=ompl::base::PlannerPtr(new ompl::geometric::DRRT(ss.getSpaceInformation()));
                             planner->as<ompl::geometric::DRRT>()->setRange(range);
                             planner->as<ompl::geometric::DRRT>()->setInformedSampling(true);
                             break;
                }

                ss.setPlanner(planner);

                ompl::time::point timeStart = ompl::time::now();
                ss.solve(maxTime);
                double timeUsed = ompl::time::seconds(ompl::time::now() - timeStart);
                times.push_back(timeUsed);
                if(timeUsed>maxTime)
                    break;

                switch(p){
                    case 0 : iterations.push_back(planner->as<ompl::geometric::RRTstar>()->numIterations());
                             break;
                    case 1 : iterations.push_back(planner->as<ompl::geometric::RRTsharp>()->numIterations());
                             break;
                    case 2 : iterations.push_back(planner->as<ompl::geometric::DRRT>()->numIterations());
                             break;
                    case 3 : iterations.push_back(planner->as<ompl::geometric::RRTstar>()->numIterations());
                             break;
                    case 4 : iterations.push_back(planner->as<ompl::geometric::RRTsharp>()->numIterations());
                             break;
                    case 5 : iterations.push_back(planner->as<ompl::geometric::DRRT>()->numIterations());
                             break;
                    case 6 : iterations.push_back(planner->as<ompl::geometric::RRTstar>()->numIterations());
                             break;
                    case 7 : iterations.push_back(planner->as<ompl::geometric::RRTsharp>()->numIterations());
                             break;
                    case 8 : iterations.push_back(planner->as<ompl::geometric::DRRT>()->numIterations());
                             break;
                }
            }

            if(times.empty() || times.back()>maxTime)
                break;
            // Compute average and standard deviation.
            double sum = 0;
            double sum_it = 0;
            for (unsigned int i = 0; i < times.size(); ++ i) {
                sum += times[i];
                sum_it += iterations[i];
            }
            double average = double(sum) / times.size();
            double it_avg = double(sum_it) / times.size();
            double sum_sq_diff = 0.0 , std_dev = 0.0;
            double sum_sq_diff_it = 0.0 , it_std = 0.0;
            if(times.size()>1){
                for (unsigned int i=0; i<times.size(); ++i) {
                    double diff = times[i] - average;
                    sum_sq_diff += diff*diff;
                    double diff_it = iterations[i] - it_avg;
                    sum_sq_diff_it += diff_it*diff_it;
                }
                std_dev = sqrt(sum_sq_diff / (times.size()-1));
                it_std = sqrt(sum_sq_diff_it / (times.size()-1));
            }

            std::cout << dim << " " << average << " " << std_dev << " " << it_avg << " " << it_std << std::endl;
        }

    }
    exit(0);
}
