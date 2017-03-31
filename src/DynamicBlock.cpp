/*
 * Blocks.cpp
 * RVO2 Library
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

/*
 * Example file showing a demo with 100 agents split in four groups initially
 * positioned in four corners of the environment. Each agent attempts to move to
 * other side of the environment through a narrow passage generated by four
 * obstacles. There is no roadmap to guide the agents around the obstacles.
 */

#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#ifndef RVO_SEED_RANDOM_NUMBER_GENERATOR
#define RVO_SEED_RANDOM_NUMBER_GENERATOR 1
#endif

#include <cmath>
#include <cstdlib>

#include <vector>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#if RVO_SEED_RANDOM_NUMBER_GENERATOR
#include <ctime>
#endif

#if _OPENMP
#include <omp.h>
#endif

#include <RVO.h>

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

double currTime = 0;
double timeStep = 0.25f;

double width = 3;
double length = 4;
double speed = 1;

/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals;

std::vector<RVO::Vector2> agents;

RVO::Vector2 obs;

void setupScenario(RVO::RVOSimulator *sim)
{
#if RVO_SEED_RANDOM_NUMBER_GENERATOR
  std::srand(static_cast<unsigned int>(std::time(NULL)));
#endif

  sim->setTimeStep(timeStep);

  sim->setAgentDefaults(15.0f, 10, 5.0f, 50.0f, 2.0f, 2.0f);

  for(int i=0; i<agents.size(); ++i){
    sim->addAgent(agents[i]);
  }


  std::vector<RVO::Vector2> obstacle1, obstacle2;

  obstacle2.push_back(obs+RVO::Vector2(-width, -length));
  obstacle1.push_back(obs+RVO::Vector2(width, -length));
  obstacle1.push_back(obs+RVO::Vector2(width, length)+RVO::Vector2(0,speed)*timeStep);
  obstacle1.push_back(obs+RVO::Vector2(-width, length)+RVO::Vector2(0,speed)*timeStep);

  sim->addObstacle(obstacle1);

  sim->processObstacles();
}

void updateVisualization()
{
  std::cout << currTime;

  std::cout << " " << obs-RVO::Vector2(width, length);

  for (size_t i = 0; i < agents.size(); ++i) {
    std::cout << " " << agents[i];
  }

  std::cout << std::endl;
}

void setPreferredVelocities(RVO::RVOSimulator *sim)
{
  for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
    RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

    if (RVO::absSq(goalVector) > 1.0f) {
      goalVector = RVO::normalize(goalVector);
    }

    sim->setAgentPrefVelocity(i, goalVector);

    float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
    float dist = std::rand() * 0.0001f / RAND_MAX;

    sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +
        dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
  }
}

void updateData(RVO::RVOSimulator *sim){
  for(int i=0; i<agents.size(); ++i){
    agents[i] = sim->getAgentPosition(i);
  }
  currTime += timeStep;
  obs += RVO::Vector2(0,speed)*timeStep;
}

bool reachedGoal()
{
  /* Check if all agents have reached their goals. */
  for (size_t i = 0; i < agents.size(); ++i) {
    if (RVO::absSq(agents[i] - goals[i]) > 2.0f) {
      return false;
    }
  }

  return true;
}

int main()
{
  double beginX = -45;
  double beginY = 50;
  int numAgent = 10;
  double endX = 50;

  for(int i=0; i<numAgent; ++i){
    agents.push_back(RVO::Vector2(beginX, beginY - double(i) / (double(numAgent)-1) * 2 * beginY));
    goals.push_back(agents[agents.size()-1]+RVO::Vector2(endX - beginX, 0.0f));
  }

  double obsX = 0;
  double obsY = -50;
  obs = RVO::Vector2(obsX, obsY);

  do {
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();
    setupScenario(sim);
    updateVisualization();
    setPreferredVelocities(sim);
    sim->doStep();
    updateData(sim);
    delete sim;
  }
  while (!reachedGoal());


  return 0;
}