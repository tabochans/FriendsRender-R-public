#include"Normal_Facet.h"

template<>
void Primitive::AABB::Expand<Primitive::inoutFacet>(const Primitive::inoutFacet& f){
	float max[3], min[3];

	PTUtility::Vec3 p1(f._Pos[0]);
	PTUtility::Vec3 p2(f._Pos[1]);
	PTUtility::Vec3 p3(f._Pos[2]);

	//ÇÊÇ≠ÇÌÇ©ÇÁÇÒÇ™ÇΩÇ‘ÇÒÅHÇ†Ç¡ÇƒÇÈ
	for (int i = 0; i < 3; i++){
		max[i] = (p1(i) > p2(i) ? p1(i) : p2(i)) > p3(i) ? (p1(i) > p2(i) ? p1(i) : p2(i)) : p3(i);
	}
	for (int i = 0; i < 3; i++){
		min[i] = (p1(i) < p2(i) ? p1(i) : p2(i)) < p3(i) ? (p1(i) < p2(i) ? p1(i) : p2(i)) : p3(i);
	}
	for (int i = 0; i < 3; i++){
		m_MaxPos(i) = (max[i] + 0.001f > m_MaxPos[i] ? max[i] + 0.001f : m_MaxPos[i]);
		m_MinPos(i) = (min[i] - 0.001f < m_MinPos[i] ? min[i] - 0.001f : m_MinPos[i]);
	}
}

namespace Isc{

	template<>
	bool Intersect(const Primitive::inoutFacet& p, const Primitive::Ray& ray, IntersectInfo& result){
		PTUtility::Vec3 T = ray.m_Org - p._Pos[0];
		float inv = ray.m_Dir.cross(p._Vec_2_0).dot(p._Vec_1_0);
		if (std::abs(inv) < 0.000001){
			return false;
		}
		result.t = T.cross(p._Vec_1_0).dot(p._Vec_2_0) / inv;
		result.u = ray.m_Dir.cross(p._Vec_2_0).dot(T) / inv;
		result.v = T.cross(p._Vec_1_0).dot(ray.m_Dir) / inv;
		if (result.t > FLT_MIN && result.u > 0.000 && result.u < 1.0 && result.v > 0.000 && result.v < 1.0 && (result.u + result.v) < 1.0){
			return true;
		}
		return false;
	}

	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::inoutFacet& p, IntersectInfo& result){
		return Intersect(p, ray, result);
	}
}