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

	Matrix GetMatrix() { return Matrix::CreateFromQuaternion(Rotation) * Matrix::CreateTranslation(Position); }
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
	bool showInnerCS;
	vector<float> l;

	State startState;
	State endState;
	InnerState prev;

	void Init();
	InnerState GetState(float animationProgress, bool angleInterpolation);
	State GetInterpolatedState(float animationProgress);
	InnerState GetInterpolatedInnerState(float animationProgress);
	InnerState InverseKinematics(State state);
	vector<Matrix> GetMatrices(InnerState state);

	Quaternion EtoQ(Vector3 v);
	Vector3 QtoE(Quaternion q);
	Vector3 ToRad(Vector3 v);
	Vector3 ToDeg(Vector3 v);

	float NormalizeDeg(float angle);
	float NormalizeRad(float angle);
	float DiffDeg(float a, float b);
	float DiffRad(float a, float b);

	Vector3 Fix(Vector3 c)
	{
		return{ Fix(c.x), Fix(c.y), Fix(c.z) };
	}
	float Fix(float f)
	{
		float eps = 10e-6;
		if (abs(f - 1) < eps) f = 1;
		if (abs(f) < eps) f = 0;
		if (abs(f+1 ) < eps) f = -1;
			
		return f;
	}
};

