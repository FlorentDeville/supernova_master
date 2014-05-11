#include "snMath.h"
#include "snMatrix44f.h"

using namespace Supernova::Vector;

namespace Supernova
{

	float clamp(float _value, float _min, float _max)
	{
		if (_value < _min)
			return _min;
		if (_value > _max)
			return _max;

		return _value;
	}

	snVec clampComponents(const snVec& _v, float _min, float _max)
	{
		return snVec4Set(clamp(snVec4GetX(_v), _min, _max), clamp(snVec4GetY(_v), _min, _max), clamp(snVec4GetZ(_v), _min, _max), clamp(snVec4GetW(_v), _min, _max));
	}

	bool isInRange(float _value, float _min, float _max)
	{
		return _value >= _min && _value <= _max;
	}

	int sign(float _value)
	{
		return _value < 0 ? -1 : 1;
	}

	void computeBasis(const snVec& _a, snVec& _b, snVec& _c)
	{
		// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
		// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component of a
		// unit vector must be greater or equal to 0.57735.

		if (snVec4GetX(_a) >= 0.57735f)
			_b = snVec4Set(snVec4GetY(_a), -snVec4GetX(_a), 0.0f, 0.0f);
		else
			_b = snVec4Set(0.0f, snVec4GetZ(_a), -snVec4GetY(_a), 0.0f);

		snVec3Normalize(_b);
		_c = snVec3Cross(_a, _b);
	}

	//compute a frenet matrix from a catmull rom interpolation
	void computeFrenetFromCatmullRom(const snVec& _a0, const snVec& _a1, const snVec& _a2, const snVec& _a3,
		float _t, snMatrix44f& _frenet)
	{
		snVec c0 = (_a3 - _a0) * 0.5f + (_a1 - _a2) * 1.5f;//_y0 * -0.5f + _y1 * 1.5f - _y2 * 1.5f + _y3 * 0.5f;
		snVec c1 = _a0 - _a1 * 2.5f + _a2 * 2 - _a3 * 0.5f;
		snVec c2 = (_a2 - _a0) * 0.5f;

		float t2 = _t * _t;

		//compute position which is the catmull rom interpolation
		_frenet[3] = c0 * t2 * _t + c1 * t2 + c2 * _t + _a1;

		//compute the forward (Z axis) vector which is the first derivative of the interpolation
		_frenet[2] = c0 * 3 * t2 + c1 * _t * 2 + c2;
		snVec3Normalize(_frenet[2]);

		//compute the left (X axis) vector which is the second derivative of the interpolation
		//_frenet[0] = c0 * 6 * _t + c1 * 2;
		//_frenet[0].normalize();

		////compute the up vector (Y axis) which is X cross Z
		//_frenet[1] = snVec::cross(_frenet[0], _frenet[2]);

		_frenet[1] = snVec4Set(0, 1, 0, 0);
		_frenet[0] = snVec3Cross(_frenet[2], _frenet[1]);
	}

	snVec cosInterpolation(const snVec& _start, const snVec& _end, float _t)
	{
		float cosineT = (1.f - cosf(_t * SN_PI)) * 0.5f;
		return (_start * (1.f - cosineT) + _end * cosineT);
	}
}