/*
 * boundedAccelSpeedStateSpace.h
 *
 *  Created on: Feb 28, 2018
 *      Author: florian
 */

#ifndef SRC_OMPL_BASE_SPACES_BOUNDEDACCELSPEEDSTATESPACE_H_
#define SRC_OMPL_BASE_SPACES_BOUNDEDACCELSPEEDSTATESPACE_H_

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>
#include <functional>

namespace ompl
{
    namespace base
    {
        class BoundedAccelSpeedStateSpace : public RealVectorStateSpace
        {
        public:
        	class StateType : public State
			{
			public:
				StateType() : State()
				{
				}

				/** \brief Access element i of values.  This does not
					check whether the index is within bounds */
				double operator[](unsigned int i) const
				{
					return values[i];
				}

				/** \brief Access element i of values.  This does not
					check whether the index is within bounds */
				double &operator[](unsigned int i)
				{
					return values[i];
				}

				/** \brief The value of the actual vector in R<sup>n</sup> */
				double *values;
			};

        	BoundedAccelSpeedStateSpace(unsigned int dim = 0, double v = 1, double a = 1)
              : RealVectorStateSpace(dim), vMax(v), aMax(a)
            {
        		/* TODO check that dim is even */
        		/* TODO set bounds with vMax */
                type_ = STATE_SPACE_REAL_VECTOR;
                setName("BoundedAccelSpeedStateSpace" + getName());
            }

            bool isMetricSpace() const override
            {
                return false;
            }

            double distance(const State *state1, const State *state2) const override;

            void interpolate(const State *from, const State *to, const double t, State *state) const override;

            void sanityChecks() const override
            {
                double zero = std::numeric_limits<double>::epsilon();
                double eps = std::numeric_limits<float>::epsilon();
                int flags = ~(STATESPACE_INTERPOLATION | STATESPACE_TRIANGLE_INEQUALITY | STATESPACE_DISTANCE_BOUND | STATESPACE_DISTANCE_SYMMETRIC);
                StateSpace::sanityChecks(zero, eps, flags);
            }
            ompl::base::State *allocState() const;
            void freeState(State *state) const;

            StateSamplerPtr allocDefaultStateSampler() const override
			{
				return std::make_shared<ompl::base::RealVectorStateSampler>(this);
			}


        protected:

            double vMax;
            double aMax;

            class oneDimSolution
            {
			public:
				std::vector<double> time;
				std::vector<double> accel;
				bool solved;

				oneDimSolution():solved(false){}

				double getTime()
				{
					if(solved && time.size()>0 && std::all_of(time.cbegin(), time.cend(), [](double i){ return i >= 0.0; }))
					{
						return std::accumulate(time.begin(), time.end(), 0.0);
					}
					else
					{
						return std::numeric_limits<double>::max();
					}
				}
			};

			class multiDimSolution
			{
			public:
				std::vector<oneDimSolution> components;
				bool solved;

				multiDimSolution(): solved(false){}

				double getTime()
				{
					if(solved)
					{
						return components.front().getTime();
					}
					else
					{
						return std::numeric_limits<double>::max();
					}
				}
			};
			oneDimSolution oneDimOptimalSolver(double x0, double x1, double v0, double v1, double vMax, double aMax) const;
			oneDimSolution oneDimFixedTimeSolver(double T, double x0, double x1, double v0, double v1, double vMax, double aMax) const;
			multiDimSolution multiDimSolver(const State *state1, const State *state2) const;


        };

//        class BoundedASMotionValidator : public MotionValidator
//        {
//        public:
//            BoundedASMotionValidator(SpaceInformation *si) : MotionValidator(si)
//            {
//                defaultSettings();
//            }
//            BoundedASMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
//            {
//                defaultSettings();
//            }
//            ~BoundedASMotionValidator() override = default;
//            bool checkMotion(const State *s1, const State *s2) const override;
//            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;
//
//        private:
//            BoundedAccelSpeedStateSpace *stateSpace_;
//            void defaultSettings();
//        };
    }
}



#endif /* SRC_OMPL_BASE_SPACES_BOUNDEDACCELSPEEDSTATESPACE_H_ */
