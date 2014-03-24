#include "snMath.h"
#include "snVector4f.h"

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
}