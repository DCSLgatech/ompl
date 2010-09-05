
/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_LAZY_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_LAZY_RRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <vector>

namespace ompl
{

    namespace geometric
    {
	
	/**
	   @anchor gLazyRRT
	   
	   @par Short description
	   
	   The basic idea of RRT is that it samples a random state @b qr
	   in the state space, then finds the state @b qc among the
	   previously seen states that is closest to @b qr and expands
	   from @b qc towards @b qr, until a state @b qm is reached and @b
	   qm is the new state to be visited. The difference between RRT
	   and LazyRRT is that when moving towards the new state @b qm,
	   LazyRRT does not check to make sure the path is valid. Instead,
	   it is optimistic and attempts to find a path as soon as
	   possible. Once a path is found, it is checked for collision. If
	   collisions are found, the invalid path segments are removed and
	   the search process is continued.       
	   
	   @par External documentation
	   - J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844730">10.1109/ROBOT.2000.844730</a><br>
	   <a href="http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246">[PDF]</a>
	   <a href="http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree">[Wikipedia]</a>
	   <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">[more]</a>
	   - R. Bohlin and L.E. Kavraki, Path planning using lazy PRM, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 521–528, 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844107">10.1109/ROBOT.2000.844107</a><br>
	   <a href="http://ieeexplore.ieee.org/ielx5/6794/18235/00844107.pdf?tp=&arnumber=844107&isnumber=18235">[PDF]

	*/

	/** \brief Lazy RRT */
	class LazyRRT : public base::Planner
	{
	public:

	    /** \brief Constructor */	    
	    LazyRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "LazyRRT")
	    {
		type_ = base::PLAN_TO_GOAL_ANY;
		
		goalBias_ = 0.05;
		maxDistance_ = 0.0;
	    }
	    
	    virtual ~LazyRRT(void)
	    {
		freeMemory();
	    }

	    virtual void getPlannerData(base::PlannerData &data) const;

	    virtual bool solve(double solveTime);
	    
	    virtual void clear(void);
	    
	    /** \brief Set the goal biasing.

		In the process of randomly selecting states in the state
		space to attempt to go towards, the algorithm may in fact
		choose the actual goal state, if it knows it, with some
		probability. This probability is a real number between 0.0
		and 1.0; its value should usually be around 0.05 and
		should not be too large. It is probably a good idea to use
		the default value. */
	    void setGoalBias(double goalBias)
	    {
		goalBias_ = goalBias;
	    }
	    
	    /** \brief Get the goal bias the planner is using */
	    double getGoalBias(void) const
	    {
		return goalBias_;
	    }
	    
	    /** \brief Set the range the planner is supposed to use.

		This parameter greatly influences the runtime of the
		algorithm. It represents the maximum length of a
		motion to be added in the tree of motions. */
	    void setRange(double distance)
	    {
		maxDistance_ = distance;
	    }
	    
	    /** \brief Get the range the planner is using */
	    double getRange(void) const
	    {
		return maxDistance_;
	    }

	    /** \brief Set a different nearest neighbors datastructure */
	    template<template<typename T> class NN>
	    void setNearestNeighbors(void)
	    {
		nn_.reset(new NN<Motion*>());
	    }
	    
	    virtual void setup(void);

	protected:

	    /** \brief Representation of a motion */
	    class Motion
	    {
	    public:
		
		Motion(void) : state(NULL), parent(NULL), valid(false)
		{
		}
		
		/** \brief Constructor that allocates memory for the state */
		Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL), valid(false)
		{
		}
		
		~Motion(void)
		{
		}

		/** \brief The state contained by the motion */
		base::State          *state;

		/** \brief The parent motion in the exploration tree */
		Motion               *parent;

		/** \brief Flag indicating whether this motion has been validated */
		bool                  valid;

		/** \brief The set of motions that descend from this one */
		std::vector<Motion*>  children;
	    };

	    /** \brief Free the memory allocated by this planner */
	    void freeMemory(void);

	    /** \brief Remove a motion from the tree datastructure */
	    void removeMotion(Motion *motion);	

	    /** \brief Compute distance between motions (actually distance between contained states) */	    
	    double distanceFunction(const Motion* a, const Motion* b) const
	    {
		return si_->distance(a->state, b->state);
	    }
	    
	    /** \brief State sampler */	    
	    base::ManifoldStateSamplerPtr                  sampler_;

	    /** \brief A nearest-neighbors datastructure containing the tree of motions */
	    boost::shared_ptr< NearestNeighbors<Motion*> > nn_;	    

	    /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */	    
	    double                                         goalBias_;

	    /** \brief The maximum length of a motion to be added to a tree */
	    double                                         maxDistance_;	

	    /** \brief The random number generator */
	    RNG                                            rng_;
	
	};
	
    }
}

#endif
    
