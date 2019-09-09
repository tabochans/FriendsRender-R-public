#include"Intersect.h"

namespace Isc{

	template<>
	bool Intersect(const Primitive::Sphere& A, const Primitive::Sphere& B, IntersectInfo& result){
		if ((A.m_Pos - B.m_Pos).norm2() < (A.m_Radius + B.m_Radius)*(A.m_Radius + B.m_Radius)){
			return true;
		}
		return false;
	}

	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::Sphere& s, IntersectInfo& result){
		PTUtility::Vec3 ss = ray.m_Org - s.m_Pos;
		double A = 1.0f;
		double B = 2.0 * ss.dot(ray.m_Dir);
		double C = ss.norm2() - s.m_Radius*s.m_Radius;
		double D = B*B - 4.0 * A*C;

		if (D > 0.00001){
			if ((s.m_Pos - ray.m_Org).norm2() < s.m_Radius*s.m_Radius){
				result.t = 0.5 * (-B + sqrt(D));
			}
			else{
				result.t = 0.5 * (-B - sqrt(D));
			}
			return true;
		}
		return false;
	}

	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::Ray& ray, IntersectInfo& result){
		return Intersect(ray, s, result);
	}

	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::Polygon& p, IntersectInfo& result){
		const PTUtility::Vec3& dd = ray.m_Dir;
		const PTUtility::Vec3& e1 = (p.m_V[1].m_Pos - p.m_V[0].m_Pos);
		const PTUtility::Vec3& e2 = (p.m_V[2].m_Pos - p.m_V[0].m_Pos);
		const PTUtility::Vec3& ps = dd.cross(e2);
		float u, v, d = 0.0f;

		float a = e1.dot(ps);
		if (a > -FLT_MIN && a < FLT_MIN) { return false; }
		float f = 1.0f / a;
		PTUtility::Vec3 s = ray.m_Org - p.m_V[0].m_Pos;
		u = f * s.dot(ps);
		if (u < 0.0|| u > 1.0) { return false; }

		PTUtility::Vec3 q = s.cross(e1);
		v = f * dd.dot(q);
		if (v < 0.0 || u + v > 1.0) { return false; }
		d = f * e2.dot(q);

		result.u = u; result.v = v; result.t = d;
		return true;
	}

	template<>
	bool Intersect(const Primitive::Polygon& p, const Primitive::Ray& ray, IntersectInfo& result){
		return Intersect(ray, p, result);
	}

	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::Triangle_ID& p, IntersectInfo& result) {
		const PTUtility::Vec3& dd = ray.m_Dir;
		const PTUtility::Vec3& e1 = (p.m_V[1].m_Pos - p.m_V[0].m_Pos);
		const PTUtility::Vec3 & e2 = (p.m_V[2].m_Pos - p.m_V[0].m_Pos);
		const PTUtility::Vec3 & ps = dd.cross(e2);
		float u, v, d = 0.0f;

		float a = e1.dot(ps);
		if (a > -FLT_MIN && a < FLT_MIN) { return false; }
		float f = 1.0f / a;
		PTUtility::Vec3 s = ray.m_Org - p.m_V[0].m_Pos;
		u = f * s.dot(ps);
		if (u < 0.0 || u > 1.0) { return false; }

		PTUtility::Vec3 q = s.cross(e1);
		v = f * dd.dot(q);
		if (v < 0.0 || u + v > 1.0) { return false; }
		d = f * e2.dot(q);

		result.u = u; result.v = v; result.t = d;
		return true;
	}

	template<>
	bool Intersect(const Primitive::Triangle_ID& p, const Primitive::Ray& ray, IntersectInfo& result) {
		return Intersect(ray, p, result);
	}

	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Ray& ray, IntersectInfo& result){
		float mm[3];
		float MM[3];
		float t = 0.0f;
		PTUtility::Vec3 Normal(0, 1, 0);

		float tmax, tmin;
		const PTUtility::Vec3& max = aabb.m_MaxPos;
		const PTUtility::Vec3& min = aabb.m_MinPos;

		if (abs(ray.m_Dir.x()) < 0.0001){
			tmax = (2.0f * (ray.m_Dir.x() > 0) - 1) * 10000.0f * (max.x() - ray.m_Org.x());
			tmin = (2.0f * (ray.m_Dir.x() > 0) - 1) * 10000.0f * (min.x() - ray.m_Org.x());
		}
		else{
			tmax = (max.x() - ray.m_Org.x()) / ray.m_Dir.x();
			tmin = (min.x() - ray.m_Org.x()) / ray.m_Dir.x();
		}
		if (tmax < tmin){
			std::swap(tmax, tmin);
		}
		MM[0] = tmax;
		mm[0] = tmin;

		float tymin, tymax;
		if (abs(ray.m_Dir.y()) < 0.0001){
			tymax = (2.0f * (ray.m_Dir.y() > 0) - 1) * 10000.0f * (max.y() - ray.m_Org.y());
			tymin = (2.0f * (ray.m_Dir.y() > 0) - 1) * 10000.0f * (min.y() - ray.m_Org.y());
		}
		else{
			tymax = (max.y() - ray.m_Org.y()) / ray.m_Dir.y();
			tymin = (min.y() - ray.m_Org.y()) / ray.m_Dir.y();
		}
		if (tymax < tymin){
			std::swap(tymax, tymin);
		}
		MM[1] = tmax;
		mm[1] = tmin;

		if (tmin > tymax || tymin > tmax){
			return false;
		}
		tmin = tmin > tymin ? tmin : tymin;
		tmax = tmax < tymax ? tmax : tymax;

		if (abs(ray.m_Dir.z()) < 0.0001){
			tymax = (2.0f * (ray.m_Dir.z() > 0) - 1) * 10000.0f * (max.z() - ray.m_Org.z());
			tymin = (2.0f * (ray.m_Dir.z() > 0) - 1) * 10000.0f * (min.z() - ray.m_Org.z());
		}
		else{
			tymax = (max.z() - ray.m_Org.z()) / ray.m_Dir.z();
			tymin = (min.z() - ray.m_Org.z()) / ray.m_Dir.z();
		}
		if (tymax < tymin){
			std::swap(tymax, tymin);
		}
		MM[2] = tmax;
		mm[2] = tmin;

		if (tmin > tymax || tymin > tmax){
			return false;
		}
		tmin = tmin > tymin ? tmin : tymin;
		tmax = tmax < tymax ? tmax : tymax;


		if (tmin < 0){
			t = tmax;
		}
		else{
			t = tmin;
		}

		if (tmax < ray.m_MinT) {
			return false;
		}

		PTUtility::Vec3 ppp = ray.m_Org + t*ray.m_Dir;
		int ii = 0;
		int jj = 0;
		float dd = 10000;
		for (int i = 0; i < 3; i++){
			if (abs(ppp[i] - max[i]) < dd){
				ii = i;
				jj = 1;
				dd = abs(ppp[ii] - max[ii]);
			}
			if (abs(ppp[i] - min[i]) < dd){
				ii = i;
				jj = -1;
				dd = abs(ppp[ii] - min[ii]);
			}
			Normal = PTUtility::Vec3(0, 0, 0);
			Normal[ii] = jj;
		}

		result.t = t;
		result.u = tmax;
		result.v = tmin;
		result.normal = Normal;
		return true;
	}

	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::AABB& aabb, IntersectInfo& result){
		return Intersect(aabb, ray, result);
	}

	template<>
	bool Intersect(const Primitive::AABB& A, const Primitive::AABB& B, IntersectInfo& result){
		for (int i = 0; i < 3; i++){
			if (A.m_MinPos[i] > B.m_MaxPos[i] || A.m_MaxPos[i] < B.m_MinPos[i]){
				return false;
			}
		}
		return true;
	}

	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::AABB& aabb, IntersectInfo& result){
		Primitive::AABB aa(s.m_Pos + s.m_Radius * PTUtility::Vec3::Ones(), s.m_Pos - s.m_Radius * PTUtility::Vec3::Ones());
		return Intersect(aa, aabb, result);
	}
	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Sphere& s, IntersectInfo& result){
		return Intersect(s, aabb, result);
	}

	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::Vertex& v, IntersectInfo& result){
		if ((s.m_Pos - v.m_Pos).norm2() < s.m_Radius*s.m_Radius){
			return true;
		}
		else{
			return false;
		}
	}
	template<>
	bool Intersect(const Primitive::Vertex& v, const Primitive::Sphere& s, IntersectInfo& result){
		return Intersect(s, v, result);
	}

	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Vertex& v, IntersectInfo& result){
		for (int i = 0; i < 3; i++){
			if (aabb.m_MaxPos[i] < v.m_Pos[i] || aabb.m_MinPos[i] > v.m_Pos[i]){
				return false;
			}
		}
		return true;
	}
	template<>
	bool Intersect(const Primitive::Vertex& v, const Primitive::AABB& aabb, IntersectInfo& result){
		return Intersect(aabb, v, result);
	}

	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::Triangle_ID& t, IntersectInfo& result) {
		const PTUtility::Vec3 p = t.m_V[1].m_Pos - t.m_V[0].m_Pos;
		const PTUtility::Vec3 q = t.m_V[2].m_Pos - t.m_V[0].m_Pos;
		const PTUtility::Vec3 pq = s.m_Pos - t.m_V[0].m_Pos;
		const PTUtility::Vec3 N = (p.normalized()).cross(q.normalized());

		float dd = 0;

		float mat[3][3] = {
			{ -N.x(), p.x(), q.x() },
			{ -N.y(), p.y(), q.y() },
			{ -N.z(), p.z(), q.z() }
		};

		float det =
			mat[0][0] * mat[1][1] * mat[2][2] + mat[1][0] * mat[2][1] * mat[0][2] + mat[2][0] * mat[0][1] * mat[1][2] -
			mat[0][0] * mat[2][1] * mat[1][2] - mat[2][0] * mat[1][1] * mat[0][2] - mat[1][0] * mat[0][1] * mat[2][2];

		if (abs(det) < 0.000001) {
			return false;
		}

		float invmat[3][3] = {
			{ mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1], mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2], mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1] },
			{ mat[1][2] * mat[2][0] - mat[0][0] * mat[2][2], mat[0][0] * mat[2][2] - mat[0][2] * mat[2][1], mat[0][2] * mat[1][0] - mat[0][0] * mat[1][2] },
			{ mat[1][0] * mat[2][1] - mat[1][1] * mat[2][1], mat[0][1] * mat[2][0] - mat[0][0] * mat[2][1], mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0] }
		};

		PTUtility::Vec3 x(0, 0, 0);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				x[i] += invmat[i][j] * pq[j] / det;
			}
		}
		if (x[0] <= s.m_Radius && x[1] >= 0.0f && x[2] >= 0.0f && x[1] + x[2] <= 1.0f) {
			return true;
		}
		else if ((s.m_Pos - t.m_V[2].m_Pos).norm() <= s.m_Radius) {
			return true;
		}
		else if ((s.m_Pos - t.m_V[1].m_Pos).norm() <= s.m_Radius) {
			return true;
		}
		else if ((s.m_Pos - t.m_V[0].m_Pos).norm() <= s.m_Radius) {
			return true;
		}
		else if (x[1] < 0.0f) {
			if (Intersect(Primitive::Ray(t.m_V[0].m_Pos, p), Primitive::Sphere(s.m_Pos, s.m_Radius), result)) {
				return true;
			}
		}
		else if (x[2] < 0.0f) {
			if (Intersect(Primitive::Ray(t.m_V[0].m_Pos, q), Primitive::Sphere(s.m_Pos, s.m_Radius), result)) {
				return true;
			}
		}
		else if (x[1] + x[2] < 1.0f) {
			if (Intersect(Primitive::Ray(t.m_V[1].m_Pos, t.m_V[2].m_Pos - t.m_V[1].m_Pos), Primitive::Sphere(s.m_Pos, s.m_Radius), result)) {
				return true;
			}
		}

		else if (x[1] <= 0.0f && x[2] >= 1.0f && abs(x[1]) + abs(x[2]) >= 1.0f && (s.m_Pos - t.m_V[2].m_Pos).norm() <= s.m_Radius) {
			return true;
		}
		else if (x[1] >= 1.0f && x[2] <= 0.0f && abs(x[1]) + abs(x[2]) >= 1.0f && (s.m_Pos - t.m_V[1].m_Pos).norm() <= s.m_Radius) {
			return true;
		}
		else if (x[1] <= 0.0f && x[2] <= 0.0f && abs(x[1]) + abs(x[2]) <= 1.0f && (s.m_Pos - t.m_V[0].m_Pos).norm() <= s.m_Radius) {
			return true;
		}
		return false;
	}

	template<>
	bool Intersect(const Primitive::Triangle_ID& t, const Primitive::Sphere& s, IntersectInfo& result) {
		return Intersect(s, t, result);
	}

	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::Polygon& PL, IntersectInfo& result){
		PTUtility::Vec3 N = (0.333333 * (PL.m_V[0].m_Normal + PL.m_V[1].m_Normal + PL.m_V[2].m_Normal)).normalized();
		PTUtility::Vec3 p = PL.m_V[1].m_Pos - PL.m_V[0].m_Pos;
		PTUtility::Vec3 q = PL.m_V[2].m_Pos - PL.m_V[0].m_Pos;
		PTUtility::Vec3 pq = s.m_Pos - PL.m_V[0].m_Pos;
		float dd = 0;

		float mat[3][3] = {
			{ -N.x(), p.x(), q.x() },
			{ -N.y(), p.y(), q.y() },
			{ -N.z(), p.z(), q.z() }
		};

		float det =
			mat[0][0] * mat[1][1] * mat[2][2] + mat[1][0] * mat[2][1] * mat[0][2] + mat[2][0] * mat[0][1] * mat[1][2] -
			mat[0][0] * mat[2][1] * mat[1][2] - mat[2][0] * mat[1][1] * mat[0][2] - mat[1][0] * mat[0][1] * mat[2][2];

		if (abs(det) < 0.000001){
			return false;
		}

		float invmat[3][3] = {
			{ mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1], mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2], mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1] },
			{ mat[1][2] * mat[2][0] - mat[0][0] * mat[2][2], mat[0][0] * mat[2][2] - mat[0][2] * mat[2][1], mat[0][2] * mat[1][0] - mat[0][0] * mat[1][2] },
			{ mat[1][0] * mat[2][1] - mat[1][1] * mat[2][1], mat[0][1] * mat[2][0] - mat[0][0] * mat[2][1], mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0] }
		};

		PTUtility::Vec3 x(0, 0, 0);
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 3; j++){
				x[i] += invmat[i][j] * pq[j] / det;
			}
		}
		if (x[0] <= s.m_Radius && x[1] >= 0.0f && x[2] >= 0.0f && x[1] + x[2] <= 1.0f){
			return true;
		}
		else if ((s.m_Pos - PL.m_V[2].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		else if ((s.m_Pos - PL.m_V[1].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		else if ((s.m_Pos - PL.m_V[0].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		else if (x[1] < 0.0f){
			if (Intersect(Primitive::Ray(PL.m_V[0].m_Pos, p), Primitive::Sphere(s.m_Pos, s.m_Radius), result)){
				return true;
			}
		}
		else if (x[2] < 0.0f){
			if (Intersect(Primitive::Ray(PL.m_V[0].m_Pos, q), Primitive::Sphere(s.m_Pos, s.m_Radius), result)){
				return true;
			}
		}
		else if (x[1] + x[2] < 1.0f){
			if (Intersect(Primitive::Ray(PL.m_V[1].m_Pos, PL.m_V[2].m_Pos - PL.m_V[1].m_Pos), Primitive::Sphere(s.m_Pos, s.m_Radius), result)){
				return true;
			}
		}

		else if (x[1] <= 0.0f && x[2] >= 1.0f && abs(x[1]) + abs(x[2]) >= 1.0f && (s.m_Pos - PL.m_V[2].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		else if (x[1] >= 1.0f && x[2] <= 0.0f && abs(x[1]) + abs(x[2]) >= 1.0f && (s.m_Pos - PL.m_V[1].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		else if (x[1] <= 0.0f && x[2] <= 0.0f && abs(x[1]) + abs(x[2]) <= 1.0f && (s.m_Pos - PL.m_V[0].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		return false;
	}
	template<>
	bool Intersect(const Primitive::Polygon& PL, const Primitive::Sphere& s, IntersectInfo& result){
		return Intersect(s, PL, result);
	}


	bool AABB_RAY_SIMD_MASK(
		const __m256 value[2][3],
		const __m256* org,
		const __m256* idir,
		const __m256 farMask,
		const __m256 nearMask,
		const int* sign,
		int& mask
	){
		__m256 tmin = _mm256_set1_ps(-100000.0f);
		__m256 tmax = _mm256_set1_ps(100000.0f);
		int idx0, idx1;

		idx0 = sign[1];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][1], org[1]), idir[1]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][1], org[1]), idir[1]));
		idx0 = sign[2];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][2], org[2]), idir[2]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][2], org[2]), idir[2]));
		idx0 = sign[0];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][0], org[0]), idir[0]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][0], org[0]), idir[0]));
		mask = _mm256_movemask_ps(_mm256_cmp_ps(tmax, tmin, _CMP_GT_OS));
		mask = mask & (_mm256_movemask_ps(_mm256_cmp_ps(tmax, nearMask, _CMP_GT_OS)));
		mask = mask & (_mm256_movemask_ps(_mm256_cmp_ps(farMask, tmin, _CMP_GT_OS)));
		return (mask > 0);
	}

	bool AABB_RAY_SIMD(
		const __m256 value[2][3],
		const __m256* org,
		const __m256* idir,
		const int* sign, 
		__m256& dist, 
		int& mask
		)
	{
		dist = _mm256_set1_ps(-100000.0f);

		__m256 tmax = _mm256_set1_ps(100000.0f);
		__m256 near = _mm256_set1_ps(0.00000001f);
		int idx0, idx1;

		idx0 = sign[1];
		idx1 = 1 - idx0;
		dist = _mm256_max_ps(dist, _mm256_mul_ps(_mm256_sub_ps(value[idx0][1], org[1]), idir[1]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][1], org[1]), idir[1]));
		idx0 = sign[2];
		idx1 = 1 - idx0;
		dist = _mm256_max_ps(dist, _mm256_mul_ps(_mm256_sub_ps(value[idx0][2], org[2]), idir[2]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][2], org[2]), idir[2]));
		idx0 = sign[0];
		idx1 = 1 - idx0;
		dist = _mm256_max_ps(dist, _mm256_mul_ps(_mm256_sub_ps(value[idx0][0], org[0]), idir[0]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][0], org[0]), idir[0]));
		mask = _mm256_movemask_ps(_mm256_cmp_ps(tmax, dist, _CMP_GT_OS));
		mask = mask & _mm256_movemask_ps(_mm256_cmp_ps(tmax, near, _CMP_GE_OS));
		return (mask > 0);
	}

	bool Intersect_AABB4_RAY(
		const Primitive::AABB4& aabb4,
		const Primitive::Ray& ray,
		std::array<float, 4>& intersectDistance
	) {
		const int raySign[3] = { ray.m_Dir[0] > 0, ray.m_Dir[1] > 0 , ray.m_Dir[2] > 0 };
		mSIMD::Double4 tNear_max(-1e16);
		mSIMD::Double4 tFar_min(1e16);
		mSIMD::Double4 Rayorg[3] = { ray.m_Org[0], ray.m_Org[1], ray.m_Org[2] };
		mSIMD::Double4 RayinvDir[3] = {
			std::abs(ray.m_Dir[0]) < 1e-16 ? 1e16 * (2.0f * raySign[0] - 1.0f) : 1.0 / ray.m_Dir[0],
			std::abs(ray.m_Dir[1]) < 1e-16 ? 1e16 * (2.0f * raySign[1] - 1.0f) : 1.0 / ray.m_Dir[1],
			std::abs(ray.m_Dir[2]) < 1e-16 ? 1e16 * (2.0f * raySign[2] - 1.0f) : 1.0 / ray.m_Dir[2] };

		for (int i = 0; i < 3; ++i) {
			const mSIMD::Double4& Near = aabb4.m_Data[1 - raySign[i]][i];
			const mSIMD::Double4& Far = aabb4.m_Data[raySign[i]][i];

			tNear_max = mSIMD::Double4::max(tNear_max, ((Near - Rayorg[i]) * RayinvDir[i]));
			tFar_min = mSIMD::Double4::min(tFar_min, ((Far - Rayorg[i]) * RayinvDir[i]));
		}
		uint32_t hitMask = tFar_min > tNear_max;

		mSIMD::Double4 rayFar(ray.m_MaxT);
		mSIMD::Double4 rayNear(ray.m_MinT);
		hitMask = hitMask & (tNear_max < rayFar);
		hitMask = (hitMask & (tFar_min > rayNear));

		for (int i = 0; i < 4; ++i) {
			if (mSIMD::Double4::GetResult_i(hitMask, i)) {
				intersectDistance[i] = tNear_max[i];
			}
			else {
				intersectDistance[i] = FLT_MAX;
			}
		}

		return hitMask > 0;
	}

	bool Intersect_AABB8_RAY(
		const Primitive::AABB8& aabb4,
		const Primitive::Ray& ray,
		std::array<float, 8>& intersectDistance
	) {
		const int raySign[3] = { ray.m_Dir[0] > 0, ray.m_Dir[1] > 0 , ray.m_Dir[2] > 0 };
		mSIMD::Float8 tNear_max(-1e16);
		mSIMD::Float8 tFar_min(1e16);
		mSIMD::Float8 Rayorg[3] = { ray.m_Org[0], ray.m_Org[1], ray.m_Org[2] };
		mSIMD::Float8 RayinvDir[3] = {
			std::abs(ray.m_Dir[0]) < 1e-16 ? 1e16 * (2.0f * raySign[0] - 1.0f) : 1.0 / ray.m_Dir[0],
			std::abs(ray.m_Dir[1]) < 1e-16 ? 1e16 * (2.0f * raySign[1] - 1.0f) : 1.0 / ray.m_Dir[1],
			std::abs(ray.m_Dir[2]) < 1e-16 ? 1e16 * (2.0f * raySign[2] - 1.0f) : 1.0 / ray.m_Dir[2] };

		for (int i = 0; i < 3; ++i) {
			const mSIMD::Float8& Near = aabb4.m_Data[1 - raySign[i]][i];
			const mSIMD::Float8& Far = aabb4.m_Data[raySign[i]][i];

			tNear_max = mSIMD::Float8::max(tNear_max, ((Near - Rayorg[i]) * RayinvDir[i]));
			tFar_min = mSIMD::Float8::min(tFar_min, ((Far - Rayorg[i]) * RayinvDir[i]));
		}
		uint32_t hitMask = tFar_min > tNear_max;

		mSIMD::Float8 rayNear(ray.m_MinT);
		mSIMD::Float8 rayFar(ray.m_MaxT);
		hitMask = hitMask & (tNear_max < rayFar);
		hitMask = (hitMask & (tFar_min > rayNear));

		for (int i = 0; i < 8; ++i) {
			if (mSIMD::Float8::GetResult_i(hitMask, i)) {
				intersectDistance[i] = tNear_max[i];
			}
			else {
				intersectDistance[i] = FLT_MAX;
			}
		}

		return hitMask > 0;
	}
}

template<>
void Primitive::AABB::Expand<Primitive::Triangle_ID>(const Triangle_ID& obj) {

	float max[3], min[3];

	PTUtility::Vec3 p1(obj.m_V[0].m_Pos);
	PTUtility::Vec3 p2(obj.m_V[1].m_Pos);
	PTUtility::Vec3 p3(obj.m_V[2].m_Pos);

	for (int i = 0; i < 3; i++) {
		max[i] = (p1(i) > p2(i) ? p1(i) : p2(i)) > p3(i) ? (p1(i) > p2(i) ? p1(i) : p2(i)) : p3(i);
	}
	for (int i = 0; i < 3; i++) {
		min[i] = (p1(i) < p2(i) ? p1(i) : p2(i)) < p3(i) ? (p1(i) < p2(i) ? p1(i) : p2(i)) : p3(i);
	}
	for (int i = 0; i < 3; i++) {
		m_MaxPos(i) = (max[i] + 0.001f > m_MaxPos[i] ? max[i] + 0.001f : m_MaxPos[i]);
		m_MinPos(i) = (min[i] - 0.001f < m_MinPos[i] ? min[i] - 0.001f : m_MinPos[i]);
	}
}


template<>
void Primitive::AABB::Expand<Primitive::Polygon>(const Polygon& obj) {

	float max[3], min[3];

	PTUtility::Vec3 p1(obj.m_V[0].m_Pos);
	PTUtility::Vec3 p2(obj.m_V[1].m_Pos);
	PTUtility::Vec3 p3(obj.m_V[2].m_Pos);

	//ÇÊÇ≠ÇÌÇ©ÇÁÇÒÇ™ÇΩÇ‘ÇÒÅHÇ†Ç¡ÇƒÇÈ
	for (int i = 0; i < 3; i++) {
		max[i] = (p1(i) > p2(i) ? p1(i) : p2(i)) > p3(i) ? (p1(i) > p2(i) ? p1(i) : p2(i)) : p3(i);
	}
	for (int i = 0; i < 3; i++) {
		min[i] = (p1(i) < p2(i) ? p1(i) : p2(i)) < p3(i) ? (p1(i) < p2(i) ? p1(i) : p2(i)) : p3(i);
	}
	for (int i = 0; i < 3; i++) {
		m_MaxPos(i) = (max[i] + 0.001f > m_MaxPos[i] ? max[i] + 0.001f : m_MaxPos[i]);
		m_MinPos(i) = (min[i] - 0.001f < m_MinPos[i] ? min[i] - 0.001f : m_MinPos[i]);
	}
}

template<>
void Primitive::AABB::Expand<PTUtility::Vec3>(const PTUtility::Vec3& p) {
	PTUtility::Vec3 max = m_MaxPos;
	PTUtility::Vec3 min = m_MinPos;

	for (int i = 0; i < 3; i++) {
		m_MaxPos(i) = max(i) > p(i) ? max(i) : p(i);
		m_MinPos(i) = min(i) < p(i) ? min(i) : p(i);
	}
}

template<>
void Primitive::AABB::Expand<Primitive::Vertex>(const Primitive::Vertex& p) {
	Expand<PTUtility::Vec3>(p.m_Pos);
}

template<>
void Primitive::AABB::Expand<Primitive::AABB>(const AABB& ref) {
	PTUtility::Vec3 max = m_MaxPos;
	PTUtility::Vec3 min = m_MinPos;

	for (int i = 0; i < 3; i++) {
		m_MaxPos(i) = max(i) > ref.m_MaxPos(i) ? max(i) : ref.m_MaxPos(i);
		m_MinPos(i) = min(i) < ref.m_MinPos(i) ? min(i) : ref.m_MinPos(i);
	}
}
template<>
void Primitive::AABB::Expand<Primitive::Sphere>(const Sphere& ref) {
	PTUtility::Vec3 max = ref.m_Pos + PTUtility::Vec3::Ones() * ref.m_Radius;
	PTUtility::Vec3 min = ref.m_Pos - PTUtility::Vec3::Ones() * ref.m_Radius;

	for (int i = 0; i < 3; i++) {
		m_MaxPos(i) = m_MaxPos(i) > max(i) ? m_MaxPos(i) : max(i);
		m_MinPos(i) = m_MinPos(i) < min(i) ? m_MinPos(i) : min(i);
	}
}
