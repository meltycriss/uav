/*
 * Sphere.cpp
 * RVO2-3D Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

/* Example file showing a demo with 812 agents initially positioned evenly distributed on a sphere attempting to move to the antipodal position on the sphere. */

#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#include <cmath>
#include <cstddef>
#include <vector>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#include <RVO.h>

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

/* Store the goals of the agents. */
std::vector<RVO::Vector3> goals;

void setupScenario(RVO::RVOSimulator *sim)
{
  //   \param timeStep      The time step of the simulation. Must be positive.

  /* Specify the global time step of the simulation. */
  sim->setTimeStep(0.125f);

  //   \param neighborDist  The default maximum distance (center point to center point) to other agents a new agent takes into 
  //                        account in the navigation. The larger this number, the longer he running time of the simulation. If
  //                        the number is too low, the simulation will not be safe. Must be non-negative.
  //   \param maxNeighbors  The default maximum number of other agents a new agent takes into account in the navigation. The 
  //                        larger this number, the longer the running time of the simulation. If the number is too low, the 
  //                        simulation will not be safe.
  //   \param timeHorizon   The default minimum amount of time for which a new agent's velocities that are computed by the 
  //                        simulation are safe with respect to other agents. The larger this number, the s    ooner an agent 
  //                        will respond to the presence of other agents, but the less freedom the agent has in choosing its 
  //                        velocities. Must be positive.
  //   \param radius        The default radius of a new agent. Must be non-negative.
  //   \param maxSpeed      The default maximum speed of a new agent. Must be non-negative.
  //   \param velocity      The default initial three-dimensional linear velocity of a new agent (optional).
  
  /* Specify the default parameters for agents that are subsequently added. */
  sim->setAgentDefaults(15.0f, 10, 10.0f, 1.5f, 2.0f);

  /* Add agents, specifying their start position, and store their goals on the opposite side of the environment. */
  for (float a = 0; a < M_PI; a += 0.1f) {
    const float z = 100.0f * std::cos(a);
    const float r = 100.0f * std::sin(a);

    for (size_t i = 0; i < r / 2.5f; ++i) {
      const float x = r * std::cos(i * 2.0f * M_PI / (r / 2.5f));
      const float y = r * std::sin(i * 2.0f * M_PI / (r / 2.5f));

      sim->addAgent(RVO::Vector3(x, y, z));
      goals.push_back(-sim->getAgentPosition(sim->getNumAgents() - 1));
    }
  }
}

#if RVO_OUTPUT_TIME_AND_POSITIONS
void updateVisualization(RVO::RVOSimulator *sim)
{
  /* Output the current global time. */
  std::cout << sim->getGlobalTime();

  /* Output the position for all the agents. */
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    std::cout << " " << sim->getAgentPosition(i);
  }

  std::cout << std::endl;
}
#endif

void setPreferredVelocities(RVO::RVOSimulator *sim)
{
  /* Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal. */
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    RVO::Vector3 goalVector = goals[i] - sim->getAgentPosition(i);

    if (RVO::absSq(goalVector) > 1.0f) {
      goalVector = RVO::normalize(goalVector);
    }

    sim->setAgentPrefVelocity(i, goalVector);
  }
}

bool reachedGoal(RVO::RVOSimulator *sim)
{
  /* Check if all agents have reached their goals. */
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > 4.0f * sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
      return false;
    }
  }

  return true;
}

int main()
{
  /* Create a new simulator instance. */
  RVO::RVOSimulator *sim = new RVO::RVOSimulator();

  /* Set up the scenario. */
  // specify currTime, agents' pos and goals
  setupScenario(sim);

  /* Perform (and manipulate) the simulation. */
  do {
#if RVO_OUTPUT_TIME_AND_POSITIONS
    updateVisualization(sim);
#endif
    // set v for each agent
    setPreferredVelocities(sim);
    sim->doStep();
  }
  while (!reachedGoal(sim));

  delete sim;

  return 0;
}
