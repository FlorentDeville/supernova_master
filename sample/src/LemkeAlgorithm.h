#ifndef LEMKE_ALGORITHM_H
#define LEMKE_ALGORITHM_H

//#include "MatrixX.h"

namespace Pengine
{
	class VectorX;
	class MatrixX;

	class LemkeAlgorithm
	{
	public:
		LemkeAlgorithm();
		~LemkeAlgorithm();

		static bool solve(const MatrixX& _M, const VectorX& _q, VectorX& _w, VectorX& _z);

	private:
		static void pivot(MatrixX& _M, MatrixX& _q, int _pivotRowId, int _pivotColumnId);

		static void swap(VectorX& _v, int _id1, int _id2);

		static int findIdOfMinimumConstant(const MatrixX& _constant);

		static double computeConstant(const MatrixX& _constant, int _rowId);
	};
}
#endif
