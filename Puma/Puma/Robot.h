#pragma once
#include <d3d11.h>
#include <vector>
#include <math.h> 
#include <DirectXMath.h>
#include <SimpleMath.h>
#include <algorithm>
#include <random>
#include <queue>
#include <memory>

#include "Graphics/Vertex.h"

using namespace std;
using namespace DirectX;
using namespace DirectX::SimpleMath;

class State
{
public:
	Vector3 Position;
	Quaternion Rotation;

	Matrix GetMatrix(){return Matrix::CreateFromQuaternion(Rotation) * Matrix::CreateTranslation(Position);}
};

class InnerState
{
public:
	vector<float> angles;
	float q;
};

class Robot
{
public:
	vector<float> l;

	State startState;
	State endState;
	InnerState startInner;
	InnerState endInner;

	void Init();
	InnerState GetState(float animationProgress, bool angleInterpolation);
	float NormalizeAngle(float angle);
	State GetInterpolatedState(float animationProgress);
	InnerState GetInterpolatedInnerState(float animationProgress);
	InnerState InverseKinematics(State state);

	Quaternion EtoQ(Vector3 v);
	Vector3 QtoE(Quaternion q);
	Vector3 ToRad(Vector3 v);
	Vector3 ToDeg(Vector3 v);
};
