#ifndef PGS_SOLVER_H
#define PGS_SOLVER_H

namespace Pengine
{

	class MatrixX;
	class VectorX;

	class PGSSolver
	{
	public:
		PGSSolver();
		~PGSSolver();

		void solve(const MatrixX& _A, const VectorX& _B, VectorX& _x, int _interationCount) const;
	};

}
#endif //PGS_SOLVER_H