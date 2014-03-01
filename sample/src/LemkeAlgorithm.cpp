#include "LemkeAlgorithm.h"

#include "MatrixX.h"
#include "VectorX.h"
#include <assert.h>

namespace Pengine
{

	LemkeAlgorithm::LemkeAlgorithm()
	{
	}

	LemkeAlgorithm::~LemkeAlgorithm()
	{
	}

	bool LemkeAlgorithm::solve(const MatrixX& _M, const VectorX& _q, VectorX& _w, VectorX& _z)
	{
		//check if q is positive
		bool qPositive = true;
		for (int i = 0; i < _q.getSize(); ++i)
		{
			if (_q(i) < 0)
			{
				qPositive = false;
				break;
			}
		}

		//if q is positive then the solution is z = 0 and w = q
		if (qPositive)
		{
			for (int i = 0; i < _q.getSize(); ++i)
			{
				_w(i, _q(i));
				_z(i, 0);
			}
			return true;
		}

		int dim = _q.getSize();
		int Z0Id = dim * 2;

		//create the working matrix. It is the _M matrix with one more column set to 1 for Z0.
		MatrixX M;
		M.setSize(_M.getRowCount(), _M.getColumnCount() + 1);
		VectorX oneVector(_M.getRowCount(), 1);
		M.setSubMatrix(0, 0, _M);
		M.setColumn(0, _M.getColumnCount(), oneVector);

		//create the working constant vector using perturbations.
		//Each line in the matrix correspond to an element of q. Each column is the coefficient of a power of epsilon.
		MatrixX constant(dim, dim + 1);
		constant.setColumn(0, 0, _q);
		for (int i = 0; i < dim; ++i)
			constant(i, i + 1, 1);
		
		//Create the dictionary vector containing basis (w) and non basis (z) elements. 
		//Each id is a position in the w and z vector and the value is the element (w1, w2, z1, z2, etc).
		//From 0 to dim-1, it is the w vector. From dim to 2*dim-1 is the z vector. 2*dim is Z0.
		VectorX dictionary;
		dictionary.setSize(dim * 2 + 1);
		for (int i = 0; i < dictionary.getSize(); ++i)
			dictionary(i, i);

		//first step, find the minium constant
		int pivotRowId = findIdOfMinimumConstant(constant);//constant.findMinimum(); //r
		int pivotColumnId = M.getColumnCount() - 1; //we start by swaping z0 which means using the last column as pivot//s

		//swap in matrix
		pivot(M, constant, pivotRowId, pivotColumnId);

		//swap position of element in the basis vector
		swap(dictionary, pivotRowId, pivotColumnId + dim);

		//the pivot column becomes the pivot row. This is because wi went out of the dictionary so zi must enter.
		pivotColumnId = pivotRowId;

		bool exit = false;
		while (!exit)
		{
			// check if the column with id pivotRowId is < 0
			//in the same time find the next line to swap
			bool isValid = false;
			int minId = -1;
			double minValue = INFINITY;
			for (int i = 0; i < M.getRowCount(); ++i)
			{
				if (M(i, pivotColumnId) < 0)
				{
					isValid = true;
					double arg = -computeConstant(constant, i) / (double)M(i, pivotColumnId);
					if (arg < minValue)
					{
						minValue = arg;
						minId = i;
					}
				}
			}

			//no variable can leave the dictionary so there is no solution.
			if (!isValid)
				return false;

			//store the new pivot row id
			pivotRowId = minId;
			
			//if this iteration will put Z0 out of the dictionary then it must be the last iteration.
			exit = dictionary(pivotRowId) == Z0Id;

			//swap in matrix
			pivot(M, constant, pivotRowId, pivotColumnId);

			//swap position of element in the basis vector
			int elementToSwap = pivotColumnId + dim;
			swap(dictionary, pivotRowId, elementToSwap);

			//element in pivotRowId got out of the dictionary, we need to find the complementary element that will enter the dictionary.
			int elementOut = dictionary(elementToSwap);
			int elementIn = -1;
			if (elementOut < dim)
				elementIn = elementOut + dim;
			else
				elementIn = elementOut - dim;

			//find the position of the elementIn in the basis vector. It corresponds to the column id for the next pivot.
			//look only from dim to the end cause the element must be in the non basic part of the dictionary.
			for (int i = dim; i < dictionary.getSize(); ++i)
			{
				if (dictionary(i) == elementIn)
				{
					pivotColumnId = i - dim;
					break;
				}
			}
		}

		//assign result
		for (int i = 0; i < dim; ++i)
		{
			int element = dictionary(i);
			if (element == Z0Id)
				continue;

			if (element < dim) //wi
				_w(element, constant(i, 0));
			else //zi
				_z(element % dim, constant(i, 0));
		}

		for (int i = dim; i < dictionary.getSize(); ++i)
		{
			int element = dictionary(i);
			if (element == Z0Id)
				continue;

			if (element < dim) //wi
				_w(element, 0);
			else //zi
				_z(element % dim, 0);
		}
		return true;
	}

	void LemkeAlgorithm::pivot(MatrixX& _M, MatrixX& _q, int _pivotRowId, int _pivotColumnId)
	{
		MatrixX res;
		res.setSize(_M.getRowCount(), _M.getColumnCount());

		float coeff = 1.f / _M(_pivotRowId, _pivotColumnId);

		for (int row = 0; row < _M.getRowCount(); ++row)
		{
			for (int column = 0; column < _M.getColumnCount(); ++column)
			{
				if (row == _pivotRowId) //same line as pivot
				{
					if (column == _pivotColumnId) //pivot
						res(row, column, coeff);
					else //same row as pivot
						res(row, column, -_M(row, column) * coeff);
				}
				else //other line
				{
					if (column == _pivotColumnId) //same column as pivot
						res(row, column, _M(row, column) * coeff);
					else //other column
					{
						float value = _M(row, column) - _M(row, _pivotColumnId) * _M(_pivotRowId, column) * coeff;
						res(row, column, value);
					}
				}
			}
		}
		
		//float coeffVector = _q(_pivotRowId) * coeff;

		MatrixX newConstant(_q.getRowCount(), _q.getColumnCount());

		for (int row = 0; row < _q.getRowCount(); ++row)
		{
			if (row == _pivotRowId) //same row as pivot
			{
				for (int column = 0; column < _q.getColumnCount(); ++column)
				{
					float value = -_q(_pivotRowId, column) * coeff;
					newConstant(row, column, value);
				}
			}
				//_q(row, -coeffVector);
			else //other row
			{
				for (int column = 0; column < _q.getColumnCount(); ++column)
				{
					float value = _q(row, column) - _M(row, _pivotColumnId) * _q(_pivotRowId, column) * coeff;
					newConstant(row, column, value);
				}
			}
				//_q(row, _q(row) - _M(row, _pivotColumnId) * coeffVector);
		}
		_q = newConstant;
		_M = res;
	}

	void LemkeAlgorithm::swap(VectorX& _v, int _id1, int _id2)
	{
		float temp = _v(_id1);
		_v(_id1, _v(_id2));
		_v(_id2, temp);
	}

	int LemkeAlgorithm::findIdOfMinimumConstant(const MatrixX& _constant)
	{
		const double SMALL_EPSILON = 0.001f;
		double minValue = INFINITY;
		int minId = -1;

		//////
		/////
		///
		//
		//might be improve by going through the column first, computing the constant and storing the result for each row then finding the minimum value.
		//
		///
		////
		/////

		//for each row
		for (int rowId = 0; rowId < _constant.getRowCount(); ++rowId)
		{
			//compute the constant
			double total = _constant(rowId, 0);

			double currentEpsilon = 1;
			for (int columnId = 1; columnId < _constant.getColumnCount(); ++columnId)
			{
				currentEpsilon *= SMALL_EPSILON;
				total += _constant(rowId, columnId) * currentEpsilon;
			}

			//if the constant is the minimum found so far, save it
			if (total < minValue)
			{
				minValue = total;
				minId = rowId;
			}
		}

		return minId;
	}

	double LemkeAlgorithm::computeConstant(const MatrixX& _constant, int _rowId)
	{
		const double SMALL_EPSILON = 0.001f;

		//compute the constant
		double total = _constant(_rowId, 0);

		double currentEpsilon = 1;
		for (int columnId = 1; columnId < _constant.getColumnCount(); ++columnId)
		{
			currentEpsilon *= SMALL_EPSILON;
			total += _constant(_rowId, columnId) * currentEpsilon;
		}

		return total;
	}
}