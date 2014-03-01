#include "PGSSolver.h"

#include "MatrixX.h"
#include "VectorX.h"

namespace Pengine
{

	PGSSolver::PGSSolver()
	{
	}


	PGSSolver::~PGSSolver()
	{
	}

	void PGSSolver::solve(const MatrixX& _A, const VectorX& _B, VectorX& _x, int _interationCount) const
	{
		int size = _B.getSize();

		//initialize x
		_x.setSize(size);
		_x.set(0);

		//make the required number of iteration
		for (int iteration = 0; iteration < _interationCount; ++iteration)
		{
			//loop through each element of x
			for (int i = 0; i < size; ++i)
			{
				//compute a(i, j) * x(j)
				float ax = 0;
				for (int j = 0; j < size; ++j)
					ax += _A(i, j) * _x(j);

				//compute x(i) = (b(i) - ax) / a(i, i)
				float res = (_B(i) - ax) / _A(i, i);
				_x(i, res);
			}
		}

	}
}