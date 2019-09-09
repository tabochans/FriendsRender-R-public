#pragma once
#include "Vec.h"
#include <immintrin.h>
#include <vector>
#include <array>
#include <algorithm>
#include "Primitive.h"

namespace Isc{

	struct IntersectInfo {
		float t;
		float u;
		float v;
		PTUtility::Vec3 normal;
		IntersectInfo() : t(0),u(0), v(0), normal(0, 1, 0){}
		IntersectInfo(float _t, float _u, float _v, const PTUtility::Vec3& _n) : t(_t), u(_u), v(_v), normal(_n) {}
	};

	// Primitive�Œ�`���ꂽ���̓��m�̌�������
	// Intersect.cpp �Ńe���v���[�g���ꉻ�����Ďg��
	template<typename R, typename T, typename S>
	bool Intersect(const T& primitive1, const S& primitive2, R& result);

	// intersection test using SIMD
	// intersectDistance store FLT_MAX if ray does not intersect AABB
	bool Intersect_AABB4_RAY(
		const Primitive::AABB4& aabb4, 
		const Primitive::Ray& ray, 
		std::array<float, 4>& intersectDistance
	);

	// intersection test using SIMD
	// intersectDistance store FLT_MAX if ray does not intersect AABB
	bool Intersect_AABB8_RAY(
		const Primitive::AABB8& aabb4,
		const Primitive::Ray& ray,
		std::array<float, 8>& intersectDistance
	);

	// simd���g����AABB��RAY�̌�������
	bool AABB_RAY_SIMD(
		const __m256 value[2][3],
		const __m256* org,
		const __m256* idir,
		const int* sign,
		__m256& dist,
		int& mask);

	// simd���g����AABB��RAY�̌�������
	bool AABB_RAY_SIMD_MASK(
		const __m256 value[2][3],
		const __m256* org,
		const __m256* idir,
		const __m256 farMask,
		const __m256 nearMask,
		const int* sign,
		int& mask);
}