#pragma once

#include "Vec.h"
#include "SIMD.h"
#include <array>
#include <float.h>

namespace Primitive {
	struct Sphere {
		PTUtility::Vec3 m_Pos;
		float m_Radius;

		static bool compPX(const Sphere* left, const Sphere* right) {
			return left->m_Pos.x() - right->m_Pos.x() < 0;
		}
		static bool compPY(const Sphere* left, const Sphere* right) {
			return left->m_Pos.y() - right->m_Pos.y() < 0;
		}
		static bool compPZ(const Sphere* left, const Sphere* right) {
			return left->m_Pos.z() - right->m_Pos.z() < 0;
		}

		Sphere(const PTUtility::Vec3& Center, float R) : m_Pos(Center), m_Radius(R) {
		}
		Sphere() : m_Pos(PTUtility::Vec3::Zero()), m_Radius(0.5f) {
		}
		~Sphere() {}
	};

	struct Ray {
		static constexpr float RAY_OFFSET = 1e-4;

		PTUtility::Vec3 m_Org;
		PTUtility::Vec3 m_Dir;
		float m_Index;
		float m_MinT;
		float m_MaxT;

		Ray operator-()const {
			return Ray(this->m_Org, -this->m_Dir, this->m_Index, this->m_MinT, this->m_MaxT);
		}
		Ray(const PTUtility::Vec3& Origin, const PTUtility::Vec3& Direction) : 
			m_Org(Origin), m_Dir(Direction.normalized()), m_Index(1.000), m_MinT(RAY_OFFSET), m_MaxT(FLT_MAX) {}

		Ray(const PTUtility::Vec3& Origin, const PTUtility::Vec3& Direction, float Index) : 
			m_Org(Origin), m_Dir(Direction.normalized()), m_Index(Index), m_MinT(RAY_OFFSET), m_MaxT(FLT_MAX) {}
		
		Ray(const PTUtility::Vec3& Origin, const PTUtility::Vec3& Direction, float Index, float MinT, float MaxT) : 
			m_Org(Origin), m_Dir(Direction), m_Index(Index), m_MinT(MinT), m_MaxT(MaxT) {}

		~Ray() {}
	};

	struct Vertex {
		PTUtility::Vec3 m_Pos;
		PTUtility::Vec3 m_Normal;
		PTUtility::Vec2 m_UV;

		static bool compPX(const Vertex* left, const Vertex* right) {
			return (left->m_Pos.x() - right->m_Pos.x()) < 0;
		}
		static bool compPY(const Vertex* left, const Vertex* right) {
			return (left->m_Pos.y() - right->m_Pos.y()) < 0;
		}
		static bool compPZ(const Vertex* left, const Vertex* right) {
			return (left->m_Pos.z() - right->m_Pos.z()) < 0;
		}

		Vertex() {}
		explicit Vertex(const PTUtility::Vec3& Pos) : m_Pos(Pos), m_Normal(0, 1, 0), m_UV(0, 0) {}
		Vertex(const PTUtility::Vec3& Pos, const PTUtility::Vec3& Normal) : m_Pos(Pos), m_Normal(Normal.normalized()), m_UV(0, 0) {}
		Vertex(const PTUtility::Vec3& Pos, const PTUtility::Vec3& Normal, const PTUtility::Vec2& UV) : m_Pos(Pos), m_Normal(Normal.normalized()), m_UV(UV) {}
	};

	struct Point {
		PTUtility::Vec3 m_Pos;

		static bool compPX(const Point* left, const Point* right) {
			return (left->m_Pos.x() - right->m_Pos.x()) < 0;
		}
		static bool compPY(const Point* left, const Point* right) {
			return (left->m_Pos.y() - right->m_Pos.y()) < 0;
		}
		static bool compPZ(const Point* left, const Point* right) {
			return (left->m_Pos.z() - right->m_Pos.z()) < 0;
		}

		Point() {}
		Point(const PTUtility::Vec3& pos) : m_Pos(pos) {}
		Point(const std::array<float, 3>& pos) : m_Pos(pos[0], pos[1], pos[2]) {}
	};

	struct Triangle_ID {
		std::array<Point, 3> m_V;
		int m_Index;

		PTUtility::Vec3 GetCenter()const {
			return 0.33333333* (m_V[0].m_Pos + m_V[1].m_Pos + m_V[2].m_Pos);
		}
		static bool compPX(const Triangle_ID* left, const Triangle_ID* right) {
			return (left->GetCenter().x() - right->GetCenter().x()) < 0;
		}
		static bool compPY(const Triangle_ID* left, const Triangle_ID* right) {
			return (left->GetCenter().y() - right->GetCenter().y()) < 0;
		}
		static bool compPZ(const Triangle_ID* left, const Triangle_ID* right) {
			return (left->GetCenter().z() - right->GetCenter().z()) < 0;
		}

		Triangle_ID() {}
		Triangle_ID(const std::array<Point, 3>& points, int ID) : m_V(points), m_Index(ID) {}
		Triangle_ID(const std::array<PTUtility::Vec3, 3>& points, int ID) : m_V({ points[0], points[1],points[2] }), m_Index(ID) {}

		PTUtility::Vec3 GetNormal() const{
			PTUtility::Vec3 N(0, 1, 0);
			PTUtility::Vec3 u = (m_V[2].m_Pos - m_V[0].m_Pos).normalized();
			PTUtility::Vec3 s = (m_V[1].m_Pos - m_V[0].m_Pos).normalized();

			return u.cross(s).normalized();
		}
	};

	struct Polygon {
		std::array<Vertex, 3> m_V;

		PTUtility::Vec3 GetCenter()const {
			return 0.33333333 * (m_V[0].m_Pos + m_V[1].m_Pos + m_V[2].m_Pos);
		}
		static bool compPX(const Polygon* left, const Polygon* right) {
			return (left->GetCenter().x() - right->GetCenter().x()) < 0;
		}
		static bool compPY(const Polygon* left, const Polygon* right) {
			return (left->GetCenter().y() - right->GetCenter().y()) < 0;
		}
		static bool compPZ(const Polygon* left, const Polygon* right) {
			return (left->GetCenter().z() - right->GetCenter().z()) < 0;
		}

		Polygon(const Vertex& v1, const Vertex& v2, const Vertex& v3) : m_V{ v1, v2, v3 } {}
		Polygon(const std::array<Vertex, 3>& vs) : m_V(vs) {}

		void CalNormal() {
			PTUtility::Vec3 N(0, 1, 0);
			PTUtility::Vec3 u = (m_V[2].m_Pos - m_V[0].m_Pos).normalized();
			PTUtility::Vec3 s = (m_V[1].m_Pos - m_V[0].m_Pos).normalized();

			N = u.cross(s).normalized();
			for (auto v : m_V) {
				v.m_Normal = N;
			}
		}
		~Polygon() {}
	};

	struct AABB {
		PTUtility::Vec3 m_MinPos;
		PTUtility::Vec3 m_MaxPos;

		PTUtility::Vec3 GetWidth() const {
			return m_MaxPos - m_MinPos;
		}
		PTUtility::Vec3 GetCenter() const {
			return 0.5 * (m_MaxPos + m_MinPos);
		}

		// Create invalid AABB
		AABB() : m_MinPos(1e16, 1e16, 1e16), m_MaxPos(-1e16, -1e16, -1e16) {}

		AABB(const PTUtility::Vec3& max, const PTUtility::Vec3& min) : m_MinPos(min), m_MaxPos(max) {}
		~AABB() {}

		template<typename T>
		void Expand(const T& obj);

		// Invalidate AABB
		void Reset() {
			m_MaxPos = PTUtility::Vec3(-1000.001, -1000.001, -1000.001);
			m_MinPos = PTUtility::Vec3(1000.001, 1000.001, 1000.001);
		}

		int MaxDimension()const {
			PTUtility::Vec3 x = m_MaxPos - m_MinPos;
			if (x(0) > x(1)) {
				if (x(0) > x(2)) { return 0; }
				else { return 2; }
			}
			else {
				if (x(1) > x(2)) { return 1; }
				else { return 2; }
			}
		}

		// Calculate surface area
		float GetArea()const {
			PTUtility::Vec3 x = m_MaxPos - m_MinPos;
			return abs(2.0f * (x(0)*x(1) + x(1)*x(2) + x(2)*x(0)));
		}
	};

	struct AABB4 {

		// [2]-- 0:min, 1:max
		// [3]-- 0:x, 1:y, 2:z
		mSIMD::Double4 m_Data[2][3];

		AABB4() : 
			m_Data{ {1e16, 1e16, 1e16}, {-1e16, -1e16, -1e16} } {};
		AABB4(const std::array<Primitive::AABB, 4>& AABBs) :
			m_Data{ {
		mSIMD::Double4({AABBs[0].m_MinPos[0], AABBs[1].m_MinPos[0], AABBs[2].m_MinPos[0], AABBs[3].m_MinPos[0]}), 
		mSIMD::Double4({AABBs[0].m_MinPos[1], AABBs[1].m_MinPos[1], AABBs[2].m_MinPos[1], AABBs[3].m_MinPos[1]}), 
		mSIMD::Double4({AABBs[0].m_MinPos[2], AABBs[1].m_MinPos[2], AABBs[2].m_MinPos[2], AABBs[3].m_MinPos[2]})},{
		mSIMD::Double4({AABBs[0].m_MaxPos[0], AABBs[1].m_MaxPos[0], AABBs[2].m_MaxPos[0], AABBs[3].m_MaxPos[0]}), 
		mSIMD::Double4({AABBs[0].m_MaxPos[1], AABBs[1].m_MaxPos[1], AABBs[2].m_MaxPos[1], AABBs[3].m_MaxPos[1]}), 
		mSIMD::Double4({AABBs[0].m_MaxPos[2], AABBs[1].m_MaxPos[2], AABBs[2].m_MaxPos[2], AABBs[3].m_MaxPos[2]})} } {};
	
		AABB4(const std::array<PTUtility::Vec3, 4>& AABB_Min, const std::array<PTUtility::Vec3, 4>& AABB_Max) :
			m_Data{ {
		mSIMD::Double4({AABB_Min[0][0], AABB_Min[1][0], AABB_Min[2][0], AABB_Min[3][0]}),
		mSIMD::Double4({AABB_Min[0][1], AABB_Min[1][1], AABB_Min[2][1], AABB_Min[3][1]}),
		mSIMD::Double4({AABB_Min[0][2], AABB_Min[1][2], AABB_Min[2][2], AABB_Min[3][2]})},{
		mSIMD::Double4({AABB_Max[0][0], AABB_Max[1][0], AABB_Max[2][0], AABB_Max[3][0]}),
		mSIMD::Double4({AABB_Max[0][1], AABB_Max[1][1], AABB_Max[2][1], AABB_Max[3][1]}),
		mSIMD::Double4({AABB_Max[0][2], AABB_Max[1][2], AABB_Max[2][2], AABB_Max[3][2]})} } {};
	};


	struct AABB8 {

		// [2]-- 0:min, 1:max
		// [3]-- 0:x, 1:y, 2:z
		mSIMD::Float8 m_Data[2][3];

		AABB8() :
			m_Data{ {1e16, 1e16, 1e16}, {-1e16, -1e16, -1e16} } {};
		AABB8(const std::array<Primitive::AABB, 8>& AABBs) :
			m_Data{ {
		mSIMD::Float8({AABBs[0].m_MinPos[0], AABBs[1].m_MinPos[0], AABBs[2].m_MinPos[0], AABBs[3].m_MinPos[0], AABBs[4].m_MinPos[0], AABBs[5].m_MinPos[0], AABBs[6].m_MinPos[0], AABBs[7].m_MinPos[0]}),
		mSIMD::Float8({AABBs[0].m_MinPos[1], AABBs[1].m_MinPos[1], AABBs[2].m_MinPos[1], AABBs[3].m_MinPos[1], AABBs[4].m_MinPos[1], AABBs[5].m_MinPos[1], AABBs[6].m_MinPos[1], AABBs[7].m_MinPos[1]}),
		mSIMD::Float8({AABBs[0].m_MinPos[2], AABBs[1].m_MinPos[2], AABBs[2].m_MinPos[2], AABBs[3].m_MinPos[2], AABBs[4].m_MinPos[2], AABBs[5].m_MinPos[2], AABBs[6].m_MinPos[2], AABBs[7].m_MinPos[2]})},{
		mSIMD::Float8({AABBs[0].m_MaxPos[0], AABBs[1].m_MaxPos[0], AABBs[2].m_MaxPos[0], AABBs[3].m_MaxPos[0], AABBs[4].m_MaxPos[0], AABBs[5].m_MaxPos[0], AABBs[6].m_MaxPos[0], AABBs[7].m_MaxPos[0]}),
		mSIMD::Float8({AABBs[0].m_MaxPos[1], AABBs[1].m_MaxPos[1], AABBs[2].m_MaxPos[1], AABBs[3].m_MaxPos[1], AABBs[4].m_MaxPos[1], AABBs[5].m_MaxPos[1], AABBs[6].m_MaxPos[1], AABBs[7].m_MaxPos[1]}),
		mSIMD::Float8({AABBs[0].m_MaxPos[2], AABBs[1].m_MaxPos[2], AABBs[2].m_MaxPos[2], AABBs[3].m_MaxPos[2], AABBs[4].m_MaxPos[2], AABBs[5].m_MaxPos[2], AABBs[6].m_MaxPos[2], AABBs[7].m_MaxPos[2]})} } {};

		AABB8(const std::array<PTUtility::Vec3, 8>& AABB_Min, const std::array<PTUtility::Vec3, 4>& AABB_Max) :
			m_Data{ {
		mSIMD::Float8({AABB_Min[0][0], AABB_Min[1][0], AABB_Min[2][0], AABB_Min[3][0], AABB_Min[4][0], AABB_Min[5][0], AABB_Min[6][0], AABB_Min[7][0]}),
		mSIMD::Float8({AABB_Min[0][1], AABB_Min[1][1], AABB_Min[2][1], AABB_Min[3][1], AABB_Min[4][1], AABB_Min[5][1], AABB_Min[6][1], AABB_Min[7][1]}),
		mSIMD::Float8({AABB_Min[0][2], AABB_Min[1][2], AABB_Min[2][2], AABB_Min[3][2], AABB_Min[4][2], AABB_Min[5][2], AABB_Min[6][2], AABB_Min[7][2]})},{
		mSIMD::Float8({AABB_Max[0][0], AABB_Max[1][0], AABB_Max[2][0], AABB_Max[3][0], AABB_Max[4][0], AABB_Max[5][0], AABB_Max[6][0], AABB_Max[7][0]}),
		mSIMD::Float8({AABB_Max[0][1], AABB_Max[1][1], AABB_Max[2][1], AABB_Max[3][1], AABB_Max[4][1], AABB_Max[5][1], AABB_Max[6][1], AABB_Max[7][1]}),
		mSIMD::Float8({AABB_Max[0][2], AABB_Max[1][2], AABB_Max[2][2], AABB_Max[3][2], AABB_Max[4][2], AABB_Max[5][2], AABB_Max[6][2], AABB_Max[7][2]})} } {};
	};
}