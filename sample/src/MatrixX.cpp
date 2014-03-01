#include "MatrixX.h"
#include <assert.h>
#include "VectorX.h"

namespace Pengine
{
	MatrixX::MatrixX()
	{
		m_columnCount = 0;
		m_rowCount = 0;
	}

	MatrixX::MatrixX(int _rowCount, int _columnCount, float defaultValue)
	{
		m_array.resize(_rowCount * _columnCount);
		m_rowCount = _rowCount;
		m_columnCount = _columnCount;
		for (int i = 0; i < _rowCount * _columnCount; ++i)
			m_array[i] = defaultValue;
	}

	MatrixX::MatrixX(int _rowCount, int _columnCount)
	{
		m_array.resize(_rowCount * _columnCount);
		m_rowCount = _rowCount;
		m_columnCount = _columnCount;
	}

	MatrixX::~MatrixX()
	{
	}

	void MatrixX::setSize(int _rowCount, int _columnCount)
	{
		m_array.resize(_rowCount * _columnCount);
		m_rowCount = _rowCount;
		m_columnCount = _columnCount;
	}

	int MatrixX::getRowCount() const
	{
		return m_rowCount;
	}

	int MatrixX::getColumnCount() const
	{
		return m_columnCount;
	}

	float MatrixX::operator ()(int _rowCount, int _columnCount) const
	{
		return m_array[m_columnCount * _rowCount + _columnCount];
	}

	void MatrixX::operator ()(int _rowCount, int _columnCount, float _value)
	{
		m_array[m_columnCount * _rowCount + _columnCount] = _value;
	}

	void MatrixX::setSubMatrix(int _rowStart, int _columnStart, const MatrixX& _m)
	{
		assert(_m.getRowCount() <= m_rowCount - _rowStart);
		assert(_m.getColumnCount() <= m_columnCount - _columnStart);

		int innerRow = _rowStart;
		for (int rowId = 0; rowId < _m.getRowCount(); rowId++)
		{
			int innerColumn = _columnStart;
			for (int columnId = 0; columnId < _m.getColumnCount(); columnId++)	
			{
				(*this)(innerRow, innerColumn, _m(rowId, columnId));
				++innerColumn;
			}
			++innerRow;
		}
	}

	void MatrixX::setColumn(int _rowStart, int _columnId, const VectorX& _v)
	{
		assert(_v.getSize() <= m_rowCount - _rowStart);

		int innerRow = 0;
		for (int rowId = _rowStart; rowId < getRowCount() + _rowStart; rowId++)
		{
			(*this)(rowId, _columnId, _v(innerRow));
			++innerRow;
		}
	}

	void MatrixX::multiply(const MatrixX& _m1, const MatrixX& _m2, MatrixX& _res)
	{
		assert(_m1.getColumnCount() == _m2.getRowCount());
		int size = _m1.getColumnCount();

		_res.setSize(_m1.getRowCount(), _m2.getColumnCount());

		for (int rowId = 0; rowId < _m1.getRowCount(); ++rowId)
		{
			for (int columnId = 0; columnId < _m2.getColumnCount(); ++columnId)
			{
				float value = 0;
				for (int i = 0; i < size; ++i)
					value += _m1(rowId, i) * _m2(i, columnId);

				_res(rowId, columnId, value);
			}
		}
	}

	void MatrixX::multiply(const VectorX& _v, const MatrixX& _m, VectorX& _res)
	{
		assert(_v.getSize() == _m.getRowCount());

		int size = _v.getSize();
		_res.setSize(size);

		for (int columnId = 0; columnId < _m.getColumnCount(); ++columnId)
		{
			float value = 0;
			for (int i = 0; i < size; ++i)
				value += _v(i) * _m(i, columnId);

			_res(columnId, value);
		}
		
	}

	void MatrixX::multiply(float _f, const MatrixX& _m, MatrixX& _res)
	{
		_res.setSize(_m.getRowCount(), _m.getColumnCount());

		for (int i = 0; i < _m.getColumnCount(); ++i)
		{
			for (int j = 0; j < _m.getRowCount(); ++j)
				_res(j, i, _f * _m(j, i));
		}
	}

	void MatrixX::transpose(const MatrixX& _m, MatrixX& _t)
	{
		_t.setSize(_m.getColumnCount(), _m.getRowCount());

		for (int rowId = 0; rowId < _m.getRowCount(); ++rowId)
		{
			for (int columnId = 0; columnId < _m.getColumnCount(); ++columnId)
			{
				_t(columnId, rowId, _m(rowId, columnId));
			}
		}
	}

	MatrixX MatrixX::getLower() const
	{
		assert(m_rowCount == m_columnCount);
		MatrixX out(m_rowCount, m_columnCount, 0);

		for (int rowId = 0; rowId < m_rowCount; ++rowId)
		{
			for (int columnId = 0; columnId <= rowId; ++columnId)
			{
				out(rowId, columnId, (*this)(rowId, columnId));
			}
		}

		return out;
	}

	MatrixX MatrixX::getUpper() const
	{
		assert(m_rowCount == m_columnCount);
		MatrixX out(m_rowCount, m_columnCount, 0);

		for (int rowId = 0; rowId < m_rowCount; ++rowId)
		{
			for (int columnId = rowId; columnId < m_columnCount; ++columnId)
			{
				out(rowId, columnId, (*this)(rowId, columnId));
			}
		}

		return out;
	}

	MatrixX MatrixX::getStrictlyLower() const
	{
		assert(m_rowCount == m_columnCount);
		MatrixX out(m_rowCount, m_columnCount, 0);

		for (int rowId = 0; rowId < m_rowCount; ++rowId)
		{
			for (int columnId = 0; columnId < rowId; ++columnId)
			{
				out(rowId, columnId, (*this)(rowId, columnId));
			}
		}

		return out;
	}

	MatrixX MatrixX::getStrictlyUpper() const
	{
		assert(m_rowCount == m_columnCount);
		MatrixX out(m_rowCount, m_columnCount, 0);

		for (int rowId = 0; rowId < m_rowCount; ++rowId)
		{
			for (int columnId = rowId + 1; columnId < m_columnCount; ++columnId)
			{
				out(rowId, columnId, (*this)(rowId, columnId));
			}
		}

		return out;
	}
}