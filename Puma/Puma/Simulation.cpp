#include "Simulation.h"

void Simulation::Init()
{
	paused = false;
	loop = false;
	animationTime = 2.0f;

	robot.Init();

	Reset();
}

void Simulation::Reset()
{
	time = 0;
}

void Simulation::Update(float dt)
{
	if (!paused)
		time += dt / 1000.0f;

	if (time > animationTime)
	{
		if (loop)
			time -= animationTime;
		else
			time = min(time, animationTime);
	}
}
