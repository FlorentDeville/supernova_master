#include "snMath.h"
#include "snVector4f-inl.h"
#include "snMatrix44f.h"

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

	snVector4f clampComponents(const snVector4f& _v, float _min, float _max)
	{
		return snVector4f(clamp(_v.getX(), _min, _max), clamp(_v.getY(), _min, _max), clamp(_v.getZ(), _min, _max), clamp(_v.getW(), _min, _max));
	}

	bool isInRange(float _value, float _min, float _max)
	{
		return _value >= _min && _value <= _max;
	}

	int sign(float _value)
	{
		return _value < 0 ? -1 : 1;
	}

	void computeBasis(const snVector4f& _a, snVector4f& _b, snVector4f& _c)
	{
		// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
		// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component of a
		// unit vector must be greater or equal to 0.57735.

		if (_a.getX() >= 0.57735f)
			_b.set(_a.getY(), -_a.getX(), 0.0f, 0.0f);
		else
			_b.set(0.0f, _a.getZ(), -_a.getY(), 0.0f);

		_b.normalize();
		_c = _a.cross(_b);
	}

	//compute a frenet matrix from a catmull rom interpolation
	void computeFrenetFromCatmullRom(const snVector4f& _a0, const snVector4f& _a1, const snVector4f& _a2, const snVector4f& _a3,
		float _t, snMatrix44f& _frenet)
	{
		snVector4f c0 = (_a3 - _a0) * 0.5f + (_a1 - _a2) * 1.5f;//_y0 * -0.5f + _y1 * 1.5f - _y2 * 1.5f + _y3 * 0.5f;
		snVector4f c1 = _a0 - _a1 * 2.5f + _a2 * 2 - _a3 * 0.5f;
		snVector4f c2 = (_a2 - _a0) * 0.5f;

		float t2 = _t * _t;

		//compute position which is the catmull rom interpolation
		_frenet[3] = c0 * t2 * _t + c1 * t2 + c2 * _t + _a1;

		//compute the forward (Z axis) vector which is the first derivative of the interpolation
		_frenet[2] = c0 * 3 * t2 + c1 * _t * 2 + c2;
		_frenet[2].normalize();

		//compute the left (X axis) vector which is the second derivative of the interpolation
		//_frenet[0] = c0 * 6 * _t + c1 * 2;
		//_frenet[0].normalize();

		////compute the up vector (Y axis) which is X cross Z
		//_frenet[1] = snVector4f::cross(_frenet[0], _frenet[2]);

		_frenet[1] = snVector4f(0, 1, 0, 0);
		_frenet[0] = snVector4f::cross(_frenet[2], _frenet[1]);
	}
}