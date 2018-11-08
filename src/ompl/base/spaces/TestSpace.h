/*
 * testSpace.h
 *
 *  Created on: Mar 28, 2018
 *      Author: florian
 */

#ifndef SRC_OMPL_BASE_SPACES_TESTSPACE_H_
#define SRC_OMPL_BASE_SPACES_TESTSPACE_H_


#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/Solvers.h"

#define EPSILON 1e-6

namespace ompl
{
	namespace base
	{
		class TestSpace : public RealVectorStateSpace
		{
		public:
			static State* savedS1;
			static State* savedS2;
			static MultiDimSolution savedSol;

			TestSpace(unsigned int dim = 0, double xMax = 50, double vMax = 5, double aMax = 1);

			void sanityChecks() const override;

            bool isMetricSpace() const override;

			void setBounds(double xMax, double vMax, double aMax);

            double distance(const State *state1, const State *state2) const override;

            void interpolate(const State *from, const State *to, const double t, State *state) const override;

            MultiDimSolution solve(const State *state1, const State *state2) const;

            bool equalStates(const State *state1, const State *state2) const;
		private:
			double vMax_;
			double aMax_;
		};
	}
}


#endif /* SRC_OMPL_BASE_SPACES_TESTSPACE_H_ */
