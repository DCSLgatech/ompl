
#include "ompl/base/spaces/BoundedAccelSpeedStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <boost/math/constants/constants.hpp>
#include <vector>
#include <limits>

using namespace ompl::base;

static double sign(double x)
{
	if(x>0.0)
		return 1.0;
	if(x<0.0)
		return -1.0;
	return 0.0;
}

const double BAS_EPS = 1e-6;


ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution ompl::base::BoundedAccelSpeedStateSpace::oneDimOptimalSolver(double x0, double x1, double v0, double v1, double vMax, double aMax) const
{
	ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol;

	double deltaX = x1-x0;
	double deltaV = v1-v0;

	{
		double u=aMax;
		double delta = u*deltaX +0.5*(v0*v0+v1*v1);
		if(delta>=0)
		{
			{
				double t2 = (-v1+sqrt(delta))/u;
				double t1 = t2 + deltaV/u;
				if(fabs(v0+t1*u)<=vMax)
				{
					ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol1;
					sol1.time.push_back(t1);
					sol1.time.push_back(t2);
					sol1.accel.push_back(u);
					sol1.accel.push_back(-u);
					sol1.solved = true;
					if(sol1.getTime()<sol.getTime())
					{
						sol=sol1;
					}
				}
				else
				{
					t1 = (sign(u)*vMax - v0) / u;
					double t3 = -(v1-sign(u)*vMax) / u;
					t2 = 1 / sign(u)/vMax * (deltaX - v0 *t1 -u *t1*t1/2 -sign(u)*vMax*t3 +u*t3*t3/2);
					ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol1;
					sol1.time.push_back(t1);
					sol1.time.push_back(t2);
					sol1.time.push_back(t3);
					sol1.accel.push_back(u);
					sol1.accel.push_back(0.0);
					sol1.accel.push_back(-u);
					sol1.solved = true;
					if(sol1.getTime()<sol.getTime())
					{
						sol=sol1;
					}
				}
			}
			{
				double t2 = (-v1-sqrt(delta))/u;
				double t1 = t2 + deltaV/u;
				if(fabs(v0+t1*u)<=vMax)
				{
					ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol1;
					sol1.time.push_back(t1);
					sol1.time.push_back(t2);
					sol1.accel.push_back(u);
					sol1.accel.push_back(-u);
					sol1.solved = true;
					if(sol1.getTime()<sol.getTime())
					{
						sol=sol1;
					}
				}
				else
				{
					t1 = (sign(u)*vMax - v0) / u;
					double t3 = -(v1-sign(u)*vMax) / u;
					t2 = 1 / sign(u)/vMax * (deltaX - v0 *t1 -u *t1*t1/2 -sign(u)*vMax*t3 +u*t3*t3/2);
					ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol1;
					sol1.time.push_back(t1);
					sol1.time.push_back(t2);
					sol1.time.push_back(t3);
					sol1.accel.push_back(u);
					sol1.accel.push_back(0.0);
					sol1.accel.push_back(-u);
					sol1.solved = true;
					if(sol1.getTime()<sol.getTime())
					{
						sol=sol1;
					}
				}
			}
		}
	}
	{
		double u=-aMax;
		double delta = u*deltaX +0.5*(v0*v0+v1*v1);
		if(delta>=0)
		{
			{
				double t2 = (-v1+sqrt(delta))/u;
				double t1 = t2 + deltaV/u;
				if(fabs(v0+t1*u)<=vMax)
				{
					ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol1;
					sol1.time.push_back(t1);
					sol1.time.push_back(t2);
					sol1.accel.push_back(u);
					sol1.accel.push_back(-u);
					sol1.solved = true;
					if(sol1.getTime()<sol.getTime())
					{
						sol=sol1;
					}
				}
				else
				{
					t1 = (sign(u)*vMax - v0) / u;
					double t3 = -(v1-sign(u)*vMax) / u;
					t2 = 1 / sign(u)/vMax * (deltaX - v0 *t1 -u *t1*t1/2 -sign(u)*vMax*t3 +u*t3*t3/2);
					ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol1;
					sol1.time.push_back(t1);
					sol1.time.push_back(t2);
					sol1.time.push_back(t3);
					sol1.accel.push_back(u);
					sol1.accel.push_back(0.0);
					sol1.accel.push_back(-u);
					sol1.solved = true;
					if(sol1.getTime()<sol.getTime())
					{
						sol=sol1;
					}
				}
			}
			{
				double t2 = (-v1-sqrt(delta))/u;
				double t1 = t2 + deltaV/u;
				if(fabs(v0+t1*u)<=vMax)
				{
					ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol1;
					sol1.time.push_back(t1);
					sol1.time.push_back(t2);
					sol1.accel.push_back(u);
					sol1.accel.push_back(-u);
					sol1.solved = true;
					if(sol1.getTime()<sol.getTime())
					{
						sol=sol1;
					}
				}
				else
				{
					t1 = (sign(u)*vMax - v0) / u;
					double t3 = -(v1-sign(u)*vMax) / u;
					t2 = 1 / sign(u)/vMax * (deltaX - v0 *t1 -u *t1*t1/2 -sign(u)*vMax*t3 +u*t3*t3/2);
					ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol1;
					sol1.time.push_back(t1);
					sol1.time.push_back(t2);
					sol1.time.push_back(t3);
					sol1.accel.push_back(u);
					sol1.accel.push_back(0.0);
					sol1.accel.push_back(-u);
					sol1.solved = true;
					if(sol1.getTime()<sol.getTime())
					{
						sol=sol1;
					}
				}
			}
		}
	}

	if(sol.getTime() < std::numeric_limits<double>::max())
	{
		sol.solved = true;
	}

	return sol;
}

ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution ompl::base::BoundedAccelSpeedStateSpace::oneDimFixedTimeSolver(double T, double x0, double x1, double v0, double v1, double vMax, double aMax) const
{
	ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol;
	ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sStar = oneDimOptimalSolver(x0,x1,v0,v1,vMax,aMax);

	if(sStar.getTime() > T)
	{
		return sol;
	}

	// case 1: optimal velocity goes through 0, pause then for the right amount of time
	double v=v0;
	for(unsigned int i=0;i<sStar.time.size();++i)
	{
	  double vNew = v + sStar.accel[i]*sStar.time[i];
	  if (vNew * v <= 0)
	  {
		// Velocity goes through 0
		double newT = -v/sStar.accel[i];
		// Construct the solution
		sStar.time[i] = newT;
		sStar.time.insert(sStar.time.begin()+i+1, T-sStar.getTime());
		sStar.accel.insert(sStar.accel.begin()+i+1, 0);
		return sStar;
	  }
	  v = vNew;
	}

	// case 2: it is possible to find a stopping point
	// subcase a: stop as fast as possible
	double tStop = fabs(v0/aMax);
	double xStop = x0 + sign(v0)*v0*v0/2/aMax;

	ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution sol2 = oneDimOptimalSolver(xStop, x1, 0.0, v1, vMax, aMax);

	if( tStop + sol2.getTime() <=T)
	{
		// solution found
		sol2.time.insert(sol2.time.begin(),T-tStop-sol2.getTime());
		sol2.time.insert(sol2.time.begin(),tStop);
		sol2.accel.insert(sol2.accel.begin(), 0.0);
		sol2.accel.insert(sol2.accel.begin(), -sign(v0)*aMax);
		sol2.solved = true;
		return sol2;
	}

	// subcase b: finish as fast as possible
	tStop = fabs(v1/aMax);
	xStop = x1 - sign(v1)*v1*v1/2/aMax;

	sol2 = oneDimOptimalSolver (x0, xStop, v0, 0.0, vMax, aMax);

	if( tStop + sol2.getTime() <=T)
	{
	  // solution found
	  sol2.time.push_back(T-tStop-sol2.getTime());
	  sol2.time.push_back(tStop);
	  sol2.accel.push_back(0.0);
	  sol2.accel.push_back(sign(v1)*aMax);
	  sol2.solved = true;
	  return sol2;
	}

	// last case: dichotomy : v0, v1, deltaX have the same sign. There is not enough time to reach a stopping point and wait
	double t1Min = 0;
	double t1Max = fabs(v0/aMax);
	double tSol = -1;
	int iterationMax = 100;
	int iteration = 0;
	double tGuess;
	while(fabs(tSol-T)>0.001 && iteration < iterationMax)
	{
	  iteration = iteration + 1;
	  tGuess = 0.5*(t1Min+t1Max);
	  double xGuess = x0 + v0*tGuess - sign(v0)*aMax*tGuess*tGuess/2;
	  double vGuess = v0 - sign(v0)*aMax*tGuess;
	  sol2 = oneDimOptimalSolver (xGuess, x1, vGuess, v1, vMax, aMax);
	  tSol = tGuess + sol2.getTime();
	  if( tSol > T)
	  {
		t1Max = tGuess;
	  }
	  else
	  {
		t1Min = tGuess;
	  }
	}

	if( iteration < iterationMax)
	{
		sol2.time.insert(sol2.time.begin(),tGuess);
		sol2.accel.insert(sol2.accel.begin(),-sign(v0)*aMax);
		sol2.solved = true;
		return sol2;
	}

	return sol;
}


ompl::base::BoundedAccelSpeedStateSpace::multiDimSolution ompl::base::BoundedAccelSpeedStateSpace::multiDimSolver(const State *state1, const State *state2) const
{
	const StateType *s1 = static_cast<const StateType *>(state1);
	const StateType *s2 = static_cast<const StateType *>(state2);
	ompl::base::BoundedAccelSpeedStateSpace::multiDimSolution sol;
	sol.components.insert(sol.components.begin(), this->dimension_, oneDimSolution());
	double maxTime = -1;
	unsigned int maxIndex = 0;
	/* Compute the optimal trajectory on each dimension */
	for(unsigned int i=0; i < this->dimension_; i += 2)
	{
		ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution s = oneDimOptimalSolver(s1->values[i], s1->values[i+1], s2->values[i], s2->values[i+1], this->vMax, this->aMax);
		double cost = s.getTime();
		if(cost>maxTime)
		{
			sol.components[i] = s;
			maxTime = cost;
			maxIndex = i;
		}
	}
	/* Recompute the trajectories such that they all take maxTime */
	for(unsigned int i=0; i < this->dimension_; i += 2)
	{
		if(i == maxIndex)
		{
			continue;
		}
		ompl::base::BoundedAccelSpeedStateSpace::oneDimSolution s = oneDimFixedTimeSolver(maxTime, s1->values[i], s1->values[i+1], s2->values[i], s2->values[i+1], this->vMax, this->aMax);
		sol.components[i] = s;
		if(!s.solved)
		{
			return sol;
		}
	}
	sol.solved = true;
	return sol;
}


double ompl::base::BoundedAccelSpeedStateSpace::distance(const State *state1, const State *state2) const
{
	std::cout << "state 1 : ";
	printState(state1, std::cout);
	std::cout << "state 2 : ";
	printState(state2, std::cout);
	double time =  multiDimSolver(state1, state2).getTime();
	std::cout << "distance : " << time << std::endl << std::endl;
	return time;
}

void ompl::base::BoundedAccelSpeedStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{

}

ompl::base::State *ompl::base::BoundedAccelSpeedStateSpace::allocState() const
{
    auto *rstate = new StateType();
    rstate->values = new double[dimension_];
    return rstate;
}

void ompl::base::BoundedAccelSpeedStateSpace::freeState(State *state) const
{
    StateType *rstate = static_cast<StateType *>(state);
    delete[] rstate->values;
    delete rstate;
}
