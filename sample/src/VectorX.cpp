#include "VectorX.h"
#include <assert.h>

namespace Pengine
{
	VectorX::VectorX()
	{
		m_size = 0;
	}

	VectorX::VectorX(int _size, float _defaultValue)
	{
		m_array.resize(_size);
		m_size = _size;
		for (int i = 0; i < m_size; ++i)
			m_array[i] = _defaultValue;
	}

	VectorX::VectorX(int _size)
	{
		m_array.resize(_size);
		m_size = _size;
	}

	VectorX::~VectorX(){}

	void VectorX::setSize(int _size)
	{
		m_array.resize(_size);
		m_size = _size;
	}

	int VectorX::getSize() const
	{
		return m_size;
	}

	float VectorX::operator()(int _id) const
	{
		return m_array[_id];
	}

	void VectorX::operator()(int _id, float value)
	{
		m_array[_id] = value;
	}

	void VectorX::setSubVector(const VectorX& _v, int start)
	{
		assert(_v.getSize() + start <= m_size);

		for (int i = 0; i < _v.getSize(); ++i)
		{
			m_array[start + i] = _v(i);
		}
	}

	void VectorX::set(float _value)
	{
		for (int i = 0; i < m_size; ++i)
		{
			m_array[i] = _value;
		}
	}

	void VectorX::multiply(const VectorX& _v1, const VectorX& _v2, VectorX& _res)
	{
		assert(_v1.getSize() == _v2.getSize());

		int size = _v1.getSize();
		_res.setSize(size);

		for (int i = 0; i < size; ++i)
			_res(i, _v1(i) * _v2(i));
	}

	void VectorX::multiply(float _f, const VectorX& _v, VectorX& _res)
	{
		int size = _v.getSize();
		_res.setSize(size);

		for (int i = 0; i < size; ++i)
			_res(i, _v(i) * _f);
	}

	void VectorX::add(const VectorX& _v1, const VectorX& _v2, VectorX& _res)
	{
		assert(_v1.getSize() == _v2.getSize());
		int size = _v1.getSize();
		_res.setSize(size);

		for (int i = 0; i < size; ++i)
			_res(i, _v1(i) + _v2(i));
	}

	void VectorX::substract(const VectorX& _v1, const VectorX& _v2, VectorX& _res)
	{
		assert(_v1.getSize() == _v2.getSize());
		int size = _v1.getSize();
		_res.setSize(size);

		for (int i = 0; i < size; ++i)
			_res(i, _v1(i) - _v2(i));
	}

	float VectorX::dot(const VectorX& _v1, const VectorX& _v2)
	{
		assert(_v1.getSize() == _v2.getSize());

		int size = _v1.getSize();

		float res = 0;
		for (int i = 0; i < size; ++i)
			res += _v1(i) * _v2(i);

		return res;
	}
}