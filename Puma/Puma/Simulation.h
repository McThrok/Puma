#pragma once
#include <d3d11.h>
#include <vector>
#include <math.h> 
#include <DirectXMath.h>
#include <SimpleMath.h>
#include <algorithm>

#include "Graphics/Vertex.h"
#include "Robot.h"

using namespace std;
using namespace DirectX;
using namespace DirectX::SimpleMath;

class Simulation
{
public:
	float time;
	bool paused;
	float animationTime;
	bool loop;

	Robot robot;

	void Init();
	void Reset();
	void Update(float dt);
};

