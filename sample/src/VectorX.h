#ifndef VECTOR_X_H
#define VECTOR_X_H

#include <vector>

namespace Pengine
{
	class VectorX
	{
	private:
		int m_size;
		std::vector<float> m_array;

	public:
		VectorX();
		VectorX(int _size, float _defaultValue);
		VectorX(int _size);

		~VectorX();

		void setSize(int _size);
		int getSize() const;
		float operator()(int _id) const;
		void operator()(int _id, float value);

		void setSubVector(const VectorX& _v, int start);

		void set(float _value);

		static void multiply(const VectorX& _v1, const VectorX& _v2, VectorX& _res);

		static void multiply(float _f, const VectorX& _v, VectorX& _res);

		static void add(const VectorX& _v1, const VectorX& _v2, VectorX& _res);

		static void substract(const VectorX& _v1, const VectorX& _v2, VectorX& _res);

		static float dot(const VectorX& _v1, const VectorX& _v2);
	};

}

#endif