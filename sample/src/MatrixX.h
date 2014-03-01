#ifndef MATRIX_X_H
#define MATRIX_X_H

#include <vector>

namespace Pengine
{
	class VectorX;

	class MatrixX
	{
	private:
		int m_rowCount;
		int m_columnCount;

		std::vector<float> m_array;

	public:
		MatrixX();
		MatrixX(int _rowCount, int _columnCount, float defaultValue);
		MatrixX(int _rowCount, int _columnCount);

		~MatrixX();

		void setSize(int _rowCount, int _columnCount);

		int getRowCount() const;

		int getColumnCount() const;

		float operator ()(int _rowCount, int _columnCount) const;

		void operator ()(int _rowCount, int _columnCount, float _value);

		void setSubMatrix(int _rowStart, int _columnStart, const MatrixX& _m);

		void setColumn(int _rowStart, int _columnId, const VectorX& _v);

		static void multiply(const MatrixX& _m1, const MatrixX& _m2, MatrixX& _res);

		static void multiply(const VectorX& _v, const MatrixX& _m, VectorX& _res);

		static void multiply(float _f, const MatrixX& _m, MatrixX& _res);

		static void transpose(const MatrixX& _m, MatrixX& _t);

		MatrixX getLower() const;

		MatrixX getUpper() const;

		MatrixX getStrictlyLower() const;

		MatrixX getStrictlyUpper() const;
	};
}
#endif
