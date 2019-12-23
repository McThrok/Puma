#include "Robot.h"

void Robot::Init()
{
	showInnerCS = true;

	l.push_back(3);
	l.push_back(2);
	l.push_back(1);

	startInner.q = 3;
	startInner.angles.push_back(30);
	startInner.angles.push_back(-10);
	startInner.angles.push_back(-10);
	startInner.angles.push_back(30);
	startInner.angles.push_back(60);

	endInner.q = 5;
	endInner.angles.push_back(70);
	endInner.angles.push_back(10);
	endInner.angles.push_back(30);
	endInner.angles.push_back(-30);
	endInner.angles.push_back(130);
}
InnerState Robot::GetState(float animationProgress, bool angleInterpolation)
{
	if (angleInterpolation)
		return GetInterpolatedInnerState(animationProgress);
	else
		return InverseKinematics(GetInterpolatedState(animationProgress));
}
State Robot::GetInterpolatedState(float animationProgress)
{
	State state;
	state.Position = (endState.Position - startState.Position) * animationProgress + startState.Position;
	state.Rotation = Quaternion::Slerp(startState.Rotation, endState.Rotation, animationProgress);

	return state;
}
InnerState Robot::GetInterpolatedInnerState(float animationProgress)
{
	InnerState inner;

	for (int i = 0; i < startInner.angles.size(); i++)
	{
		float s = startInner.angles[i];
		float e = endInner.angles[i];
		if (e - s > XM_PI) e -= XM_2PI;
		if (s - e > XM_PI) e += XM_2PI;

		inner.angles.push_back((e - s) * animationProgress + s);
	}

	inner.q = (endInner.q - startInner.q) * animationProgress + startInner.q;

	return inner;
}
Quaternion Robot::EtoQ(Vector3 v)
{
	float cy = cosf(v.z * 0.5);
	float sy = sinf(v.z * 0.5);
	float cp = cosf(v.y * 0.5);
	float sp = sinf(v.y * 0.5);
	float cr = cosf(v.x * 0.5);
	float sr = sinf(v.x * 0.5);

	Quaternion q;
	q.w = cy * cp * cr + sy * sp * sr;
	q.x = cy * cp * sr - sy * sp * cr;
	q.y = sy * cp * sr + cy * sp * cr;
	q.z = sy * cp * cr - cy * sp * sr;

	return q;
}
Vector3 Robot::QtoE(Quaternion q)
{
	q.Normalize();
	Vector3 angles;

	float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	angles.x = atan2f(sinr_cosp, cosr_cosp);

	float sinp = 2 * (q.w * q.y - q.z * q.x);
	if (abs(sinp) >= 1)
		angles.y = copysignf(XM_PIDIV2, sinp);
	else
		angles.y = asinf(sinp);

	float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	angles.z = atan2(siny_cosp, cosy_cosp);

	return angles;
}
Vector3 Robot::ToRad(Vector3 v)
{
	return v * XM_PI / 180;
}
Vector3 Robot::ToDeg(Vector3 v)
{
	return v * 180 / XM_PI;
}
float Robot::NormalizeAngle(float angle)
{
	while (angle < 0) angle += 360;
	while (angle >= 360) angle -= 360;

	return angle;
}
InnerState Robot::InverseKinematics(State state)
{
	return InnerState();
}
vector<Matrix> Robot::GetMatrixes(InnerState inner)
{
	vector<Matrix> result;
	Matrix m = Matrix::CreateRotationZ(XMConvertToRadians(inner.angles[0]));
	result.push_back(m);

	m = Matrix::CreateRotationY(XMConvertToRadians(inner.angles[1])) * Matrix::CreateTranslation(0, 0, l[0]) * m;
	result.push_back(m);

	m = Matrix::CreateRotationY(XMConvertToRadians(inner.angles[2])) * Matrix::CreateTranslation(inner.q, 0, 0) * m;
	result.push_back(m);

	m = Matrix::CreateRotationZ(XMConvertToRadians(inner.angles[3])) * Matrix::CreateTranslation(0, 0, -l[1]) * m;
	result.push_back(m);

	m = Matrix::CreateRotationX(XMConvertToRadians(inner.angles[4])) * Matrix::CreateTranslation(l[2], 0, 0) * m;
	result.push_back(m);

	return result;
}

