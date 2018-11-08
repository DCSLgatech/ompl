/*
 * TestSpace.cpp
 *
 *  Created on: Jul 18, 2018
 *      Author: florian
 */

#include "ompl/base/spaces/TestSpace.h"

//#define FLO_DEBUG

ompl::base::State* ompl::base::TestSpace::savedS1 = NULL;
ompl::base::State* ompl::base::TestSpace::savedS2 = NULL;
ompl::base::MultiDimSolution ompl::base::TestSpace::savedSol;

ompl::base::TestSpace::TestSpace(unsigned int dim, double xMax, double vMax, double aMax)
	: RealVectorStateSpace(2*dim)
{
	setName("TestSpace" + getName());
	setBounds(xMax, vMax, aMax);
	savedS1 = allocState();
	savedS2 = allocState();
	StateType *tos = static_cast<StateType *>(savedS1);
	tos->values[0]=1000;
}

void ompl::base::TestSpace::sanityChecks() const
{
	unsigned int flags = ~(STATESPACE_DISTANCE_SYMMETRIC | STATESPACE_DISTANCE_BOUND | STATESPACE_SERIALIZATION | STATESPACE_TRIANGLE_INEQUALITY);
	StateSpace::sanityChecks(EPSILON, EPSILON, flags);
}

bool ompl::base::TestSpace::isMetricSpace() const
{
	return false;
}

void ompl::base::TestSpace::setBounds(double xMax, double vMax, double aMax)
{
	vMax_ = vMax;
	aMax_ = aMax;
	RealVectorBounds bounds(dimension_);
	for(unsigned int i = 0; i < dimension_; i+=2)
	{
		bounds.setLow(i,   -xMax);
		bounds.setLow(i+1, -vMax);
		bounds.setHigh(i,   xMax);
		bounds.setHigh(i+1, vMax);
	}
	RealVectorStateSpace::setBounds(bounds);
}

double ompl::base::TestSpace::distance(const State *state1, const State *state2) const
{
	double d = 0.0;
	if(equalStates(state1, state2))
	{
		d = 0.0;
	}
	else
	{
		MultiDimSolution sol = solve(state1, state2);
		if(sol.solved)
		{
			d = sol.distance;
		}
		else
		{
			d = std::numeric_limits<double>::max();
		}
	}
#ifdef FLO_DEBUG
//            	std::cout << "distance between ";
//            	printState(state1, std::cout);
//            	printState(state2, std::cout);
//            	std::cout << "distance : " << d<< std::endl;
#endif
	return d;
}

void ompl::base::TestSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
	MultiDimSolution sol = solve(from, to);
#ifdef FLO_DEBUG
	std::cout << "Interpolation at t=" << t << " between " << std::endl;
	printState(from, std::cout);
	printState(to, std::cout);
#endif
	const StateType *f = static_cast<const StateType *>(from);
	const StateType *tos = static_cast<const StateType *>(to);
	StateType *s = static_cast<StateType *>(state);
	sol.interpolate(dimension_, f->values, tos->values, t, s->values, vMax_, aMax_);
#ifdef FLO_DEBUG
	std::cout << "Result : ";
	printState(state, std::cout);
#endif
//	printState(state, std::cout);
//	std::cout << aMax_ << std::endl;
//	std::cout << sol.distance << std::endl;

}

ompl::base::MultiDimSolution ompl::base::TestSpace::solve(const State *state1, const State *state2) const
{
	if(!equalStates(state1, savedS1) || !equalStates(state2, savedS2))
	{
		ompl::base::RealVectorStateSpace::copyState(savedS1, state1);
		ompl::base::RealVectorStateSpace::copyState(savedS2, state2);
		const StateType *s1 = static_cast<const StateType *>(state1);
		const StateType *s2 = static_cast<const StateType *>(state2);
		savedSol = MultiDimSolution(dimension_, s1->values, s2->values, vMax_, aMax_);
	}
	return savedSol;
}

bool ompl::base::TestSpace::equalStates(const State *state1, const State *state2) const
{
	const double *s1 = static_cast<const StateType *>(state1)->values;
	const double *s2 = static_cast<const StateType *>(state2)->values;
	for (unsigned int i = 0; i < dimension_; ++i)
	{
		double diff = (*s1++) - (*s2++);
		if (fabs(diff) > EPSILON * 2.0)
			return false;
	}
	return true;
}
