#include "YourPlanner.h"
#include "TutorialPlanSystem.h"
#include <iostream>
#include <boost/make_shared.hpp>
#include "rl/plan/GaussianSampler.h"

YourPlanner::YourPlanner() :
	RrtCon(),
	lastConnected(6),
	beforeLastConnected(6),
	failCounter(0.6), // Some initial standard deviation. doesn't matter. we overwrite in solve anyways
	sigma(6) // 6 DOF
{
	delta = 0.10;
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
	return "Your Planner";
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
	if (this->rand() > this->probability)
	{
		// Set chosen to mean of gaussian (which is the extension of the last connected vector)
		chosen = lastConnected + (lastConnected-beforeLastConnected);
		// Set sigma according to failCounter
		sigma << failCounter, failCounter, failCounter, failCounter, failCounter, failCounter;
		//(GaussianSampler*)(this->sampler)->sigma = &sigma;


		((MyGaussian*)this->sampler)->generateGaussian(chosen);
		//std::cout << "chosen: \n" << chosen << std::endl;

		// why?
		if(chosen.maxCoeff() > 6) {
			//std::cout << chosen << std::endl;
		}
		else if(chosen.minCoeff() < -6) {
			//std::cout << chosen << std::endl;
		} else {
			//std::cout << "Cool, alright" << std::endl;
		}
		//std::cout << "Sigma: \n" <<  *((GaussianSampler*)this->sampler)->sigma << "Mean: \n" << lastConnected + (lastConnected-beforeLastConnected) << "Chosen: " << chosen << std::endl;
	}
	else
	{
		chosen = *this->goal;
	}
}

Rrt::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
	//your modifications here
	//check 

	Rrt::Vertex res = RrtCon::connect(tree, nearest, chosen);
	if(res == NULL) {
		if (this->areEqual(chosen, *this->goal)) {
			std::cout << "must not happen..." << std::endl;
			// It's alright. We just couldn't connect to the goal
		} else {
			// Fail!!
			failCounter += 0.05;
			if(failCounter > 1) {
				
				failCounter = 0.3;
				Rrt::Vertex v;
				//int x = int(this->rand() * ::boost::num_vertices(tree));

				//std::cout << lastConnected << std::endl << std::endl << beforeLastConnected << std::endl;
				//std::cout << "Failed too much. Try closest vertex" <<  std::endl;
					
				double minDistance = 10000;
				for(VertexIteratorPair i=::boost::vertices(tree); i.first != i.second; ++i.first) {
					double distance = this->model->distance(*tree[*i.first].q, *this->goal);
					if(distance < minDistance && this->rand() > 0.5) {// 50 % chance to skip the closest one
						// Find another node 
						minDistance = distance;
						lastConnected = *tree[*i.first].q;
						beforeLastConnected = *tree[*i.first].q;
					}
				}

			}
				
		}
	} else {
		//std::cout << failCounter << std::endl;
		// Success: Reduce failCounter to reduce variance
		failCounter /= 2.5;
		if(failCounter < 0.001) {
			failCounter = 30;
		}
		// Update the vectors where we sample
		beforeLastConnected = (*tree[nearest.first].q);
		//std::cout << chosen << std::endl;
		lastConnected = chosen;

	}
	return res;
}
	
// We only extend, dont connect
/*Rrt::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
	//your modifications here
	return RrtCon::extend(tree, nearest, chosen);
}*/

bool
YourPlanner::solve()
{
	failCounter = 0.3;
	((GaussianSampler*)this->sampler)->sigma = &sigma;
	lastConnected = *this->start;
	beforeLastConnected = *this->start;
	//your modifications here
	this->begin[0] = this->addVertex(this->tree[0], ::boost::make_shared< ::rl::math::Vector >(*this->start));
	
	::rl::math::Vector chosen(this->model->getDof());
	
	timer.start();
	timer.stop();
	
	while (timer.elapsed() < this->duration)
	{
		this->choose(chosen);
		
		Neighbor nearest = this->nearest(this->tree[0], chosen);
		Vertex connected;
		
		if (this->areEqual(chosen, *this->goal))
			connected = this->connect(this->tree[0], nearest, chosen);
		else 
			connected = this->extend(this->tree[0], nearest, chosen);
		
		if (NULL != connected)
		{
			if (this->areEqual(*this->tree[0][connected].q, *this->goal))
			{
				this->end[0] = connected;
				return true;
			}
		}
		
		timer.stop();
	}
	
	return false;
}
