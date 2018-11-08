/*
 * Solvers.h
 *
 *  Created on: Mar 28, 2018
 *      Author: florian
 */

#ifndef SRC_OMPL_BASE_SPACES_SOLVERS_H_
#define SRC_OMPL_BASE_SPACES_SOLVERS_H_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <algorithm>    // std::min

static inline double sign(double x)
{
	if(x>0.0)
		return 1.0;
	if(x<0.0)
		return -1.0;
	return 0.0;
}

static inline double clip(double x, double xMin, double xMax)
{
	return std::min(std::max(xMin,x),xMax);
}

static inline double clip(double x, double xMax)
{
	return std::min(std::max(-xMax,x),xMax);
}

namespace ompl
{
	namespace base
	{
		struct oneDimSolution{
			bool solved;
			double bestTime;
			std::vector<double> time;
			std::vector<double> accel;
			double x0;
			double x1;
			double v0;
			double v1;
			double vMax;
			double aMax;

			oneDimSolution() : solved(false), bestTime(std::numeric_limits<double>::max()) {}

			oneDimSolution(double _x0, double _x1, double _v0, double _v1, double _vMax, double _aMax)
				: solved(false), bestTime(std::numeric_limits<double>::max()),
				  x0(_x0), x1(_x1), v0(_v0), v1(_v1), vMax(_vMax), aMax(_aMax)
			{
				v0 = clip(v0, vMax);
				v1 = clip(v1, vMax);
			}

			void solve()
			{
				double deltaX = x1-x0;
				double deltaV = v1-v0;
				for(double s1 = -1; s1 < 2; s1+=2)
				{
					double u=s1*aMax;
					double delta = u*deltaX +0.5*(v0*v0+v1*v1);
					if(delta>=0)
					{
						for(double s2 = -1; s2 < 2; s2+=2)
						{
							double t2 = (-v1+s2*sqrt(delta))/u;
							double t1 = t2 + deltaV/u;
							if(t1>=0 && t2>=0 && fabs(v0+t1*u)<=vMax)
							{
								if(t1+t2 < bestTime)
								{
									time.clear();
									accel.clear();

									time.push_back(t1);
									time.push_back(t2);
									accel.push_back(u);
									accel.push_back(-u);

									bestTime = t1+t2;
									solved = true;
								}
							}
							else
							{
								t1 = (sign(u)*vMax - v0) / u;
								double t3 = -(v1-sign(u)*vMax) / u;
								t2 = 1 / sign(u)/vMax * (deltaX - v0 *t1 -u *t1*t1/2 -sign(u)*vMax*t3 +u*t3*t3/2);
								if(t1>=0 && t2>=0 && t3>=0 && t1+t2+t3 < bestTime)
								{
									time.clear();
									accel.clear();

									time.push_back(t1);
									time.push_back(t2);
									time.push_back(t3);
									accel.push_back(u);
									accel.push_back(0.0);
									accel.push_back(-u);

									bestTime = t1+t2+t3;
									solved = true;
								}
							}
						}
					}
				}
//				assert(solved);
				if(!solved)
					bestTime = std::numeric_limits<double>::max();
			}

			void interpolate(double alpha, double *xo, double *vo)
			{
				if(!solved)
				{
					solve();
				}
				alpha = clip(alpha, 0., 1.);
				double t = alpha * bestTime;
				double x = x0;
				double v = v0;
				double teps = 1e-8;

				for(unsigned int i=0; i<time.size(); ++i)
				{
					if(t <= time[i])
					{
						double tt = std::max(t-teps, 0.0);
						x += accel[i] *tt*tt/2 + v*tt;
						v += accel[i] *tt;

						*xo = x;
						*vo = v;
						return;
					}else{
						double tt = std::max(time[i]-teps, 0.0);
						x += accel[i] *tt*tt/2 + v*tt;
						v += accel[i] *tt;
						t -= time[i];
					}
				}

				*xo = x;
				*vo = v;
			}

			bool solveFixedTime(double fixedTime)
			{
				bool success = false;

				// solve optimal time if not solved
				if(!solved)
				{
					solve();
				}

				// it cannot be done faster than the optimal time
				if(fixedTime < bestTime)
				{
					return false;
				}

				// case 1: optimal velocity goes through 0, pause then for the right amount of time
				double v=v0;
				for(unsigned int i=0;i<time.size();++i)
				{
				  double vNew = v + accel[i]*time[i];
				  if (vNew * v <= 0)
				  {
					// Velocity goes through 0
					double newT = -v/accel[i];
					double oldT = time[i] - newT;
					// Construct the solution
					// split the current step
					time[i] = newT;
					time.insert(time.begin()+i+1, oldT);
					accel.insert(accel.begin()+i+1, accel[i]);
					// insert time at v=0
					time.insert(time.begin()+i+1, fixedTime-bestTime);
					accel.insert(accel.begin()+i+1, 0);
					bestTime = fixedTime;
//					std::cout << "case 1" << std::endl;
					return true;
				  }
				  v = vNew;
				}

				// case 2: it is possible to find a stopping point
				// subcase a: stop as fast as possible
				double tStop = fabs(v0/aMax);
				double xStop = x0 + sign(v0)*v0*v0/2/aMax;

				oneDimSolution sol2(xStop, x1, 0.0, v1, vMax, aMax);
				sol2.solve();

				if( tStop + sol2.bestTime <= fixedTime)
				{
					// solution found
					time.clear();
					accel.clear();

					time.push_back(tStop);
					time.push_back(fixedTime-tStop-sol2.bestTime);
					time.insert(time.end(), sol2.time.begin(), sol2.time.end());

					accel.push_back(-sign(v0)*aMax);
					accel.push_back(0.0);
					accel.insert(accel.end(), sol2.accel.begin(), sol2.accel.end());

					bestTime = fixedTime;
//					std::cout << "case 2a" << std::endl;
					return true;
				}

				// subcase b: finish as fast as possible
				tStop = fabs(v1/aMax);
				xStop = x1 - sign(v1)*v1*v1/2/aMax;

				sol2 = oneDimSolution(x0, xStop, v0, 0.0, vMax, aMax);
				sol2.solve();

				if( tStop + sol2.bestTime <= fixedTime)
				{
					// solution found
					time.clear();
					accel.clear();

					time.insert(time.end(), sol2.time.begin(), sol2.time.end());
					time.push_back(fixedTime-tStop-sol2.bestTime);
					time.push_back(tStop);

					accel.insert(accel.end(), sol2.accel.begin(), sol2.accel.end());
					accel.push_back(0.0);
					accel.push_back(sign(v1)*aMax);

					bestTime = fixedTime;
//					std::cout << "case 2b" << std::endl;
					return true;
				}

				/* TODO last case: v0,v1,deltax same sign, no time to stop */
				/* 2 cases:
				 * - deltaX < deltaXmin(T) : impossible
				 * - deltaX > deltaXmin(T) : a solution exists
				 */
//				double deltaXmin = 1/8/aMax*(8*fabs(v0)*(fabs(v1)-fabs(v0))  -2 * (v1-v0)*(v1-v0)
//											 + 4 * (fabs(v1)-fabs(v0)) *aMax * fixedTime
//											 + 2*aMax*aMax*fixedTime*fixedTime);
				double t = (-fabs(v1)+fabs(v0)+aMax*fixedTime)/2/aMax;
				double deltaXmin = -fabs(v0)*t - aMax * t * t / 2
									+ (fabs(v0) - aMax * t) * (fixedTime - t) + aMax * (fixedTime -t) * (fixedTime -t) / 2;

//				assert(deltaXmin == deltaXmin2);

				if(fabs(x0-x1) > deltaXmin)
				{
					/* a solution exists, dichotomy could be used to find it */
					double t1Min = 0;
					double t1Max = -(fabs(v1) - fabs(v0) - aMax * fixedTime)/2/aMax;
					double tSol = -1;
					int iterationMax = 30;
					int iteration = 0;
					double tGuess;
					while(fabs(tSol-fixedTime)>0.00001 && iteration < iterationMax)
					{
					  iteration = iteration + 1;
					  tGuess = 0.5*(t1Min+t1Max);
					  double xGuess = x0 + v0*tGuess - sign(v0)*aMax*tGuess*tGuess/2;
					  double vGuess = v0 - sign(v0)*aMax*tGuess;
					  sol2 = oneDimSolution(xGuess, x1, vGuess, v1, vMax, aMax);
					  sol2.solve();
					  tSol = tGuess + sol2.bestTime;
					  if( tSol > fixedTime)
					  {
						t1Max = tGuess;
					  }
					  else
					  {
						t1Min = tGuess;
					  }
//					  std::cout << "dichotomy to find " << fixedTime << ", current guess : " << tGuess << ", current solution : " << tSol << std::endl;
					}

					if( iteration < iterationMax)
					{
						// solution found
						time.clear();
						accel.clear();

						time.push_back(tGuess);
						time.insert(time.end(), sol2.time.begin(), sol2.time.end());

						accel.push_back(-sign(v0)*aMax);
						accel.insert(accel.end(), sol2.accel.begin(), sol2.accel.end());

						bestTime = fixedTime;
//						std::cout << "case 3" << std::endl;
						return true;
					}
//					assert(true == false);
				}
//				else
				{
					/* no solution exists, the position would go to far in the given time. Slow the speed to 0 and solve from there */
					/* same as case 2b */
					tStop = fabs(v1/aMax);
					xStop = x1 - sign(v1)*v1*v1/2/aMax;

					sol2 = oneDimSolution(x0, xStop, v0, 0.0, vMax, aMax);
					sol2.solve();

					// solution found
					time.clear();
					accel.clear();

					time.insert(time.end(), sol2.time.begin(), sol2.time.end());
					time.push_back(fixedTime-tStop-sol2.bestTime);
					time.push_back(tStop);

					accel.insert(accel.end(), sol2.accel.begin(), sol2.accel.end());
					accel.push_back(0.0);
					accel.push_back(sign(v1)*aMax);

					bestTime = fixedTime;
//					std::cout << "case 4" << std::endl;
					return false;
				}
				/* we shouldn't get here */
				assert(true == false);
				return success;
			}
		};

		struct MultiDimSolution{
			bool solved;
			double distance;
			std::vector<oneDimSolution> sols;

			MultiDimSolution() : solved(false), distance(std::numeric_limits<double>::max()) { sols.clear();}
			MultiDimSolution(unsigned int dim, double *state1, double *state2, double vMax, double aMax) : MultiDimSolution()
			{
				solve(dim, state1, state2, vMax, aMax);
			}
			void solve(unsigned int dim, double *state1, double *state2, double vMax, double aMax)
			{
//				assert(dim >= 2);
//				oneDimSolution s(state1[0], state2[0], state1[1], state2[1], vMax, aMax);
//				s.solve();
//				solved = s.solved;
//				distance = s.bestTime;
				/* solve each dimension
				 * find slowest dimension
				 * while no solution found
				 * try solving all dimensions with fix time
				 * if it fails, increase the time to the minimum time for the failed dimension
				 * it will eventually succeed
				 * return solution */
				assert(dim % 2 == 0);
				sols = std::vector<oneDimSolution>(dim/2);
				double maxTime = 0;
				for(unsigned int i=0; i < dim; i = i+2)
				{
					sols[i/2] = oneDimSolution(state1[i], state2[i], state1[i+1], state2[i+1], vMax, aMax);
					sols[i/2].solve();
					maxTime = std::max(maxTime, sols[i/2].bestTime);
				}
				solved = false;
				while(!solved)
				{
					bool allGood = true;
					for(unsigned int i=0; i < dim; i = i+2)
					{
						if(maxTime == sols[i/2].bestTime)
						{
							continue;
						}
						if(!sols[i/2].solveFixedTime(maxTime))
						{
							maxTime = sols[i/2].bestTime;
							allGood = false;
							break;
						}

					}
					if(allGood)
					{
						solved = true;
						distance = maxTime;
					}
				}
			}

			void interpolate(unsigned int dim, double *state1, double *state2, double t, double *s, double vMax, double aMax)
			{
//				assert(dim >= 2);
//				oneDimSolution ods(state1[0], state2[0], state1[1], state2[1], vMax, aMax);
//				ods.interpolate(t, &s[0], &s[1]);

				// solve optimal time if not solved
				if(!solved)
				{
					solve(dim, state1, state2, vMax, aMax);
				}
				for(unsigned int i=0; i < dim; i+=2)
				{
					sols[i/2].interpolate(t, &s[i], &s[i+1]);
				}
			}
		};
	}
}


#endif /* SRC_OMPL_BASE_SPACES_SOLVERS_H_ */
