#pragma once
#include<cmath>

namespace aObject {

	// describe intersection info. BVH::Intersect() returns object which inherits this class. 
	struct iIntersectResult {
		static constexpr unsigned int InvalidID = 0xffffffff;

		unsigned int _ElementID;
		unsigned int _LeafNodeID;
		iIntersectResult() : _ElementID(InvalidID), _LeafNodeID(InvalidID) {
		}
	};

	// describe intersection result (Ray and Triangle)
	struct IntersectResult_RayTriangle : iIntersectResult {
		float _t;
		float _u;
		float _v;
		explicit IntersectResult_RayTriangle() : _t(1e16), _u(0.0f), _v(0.0f), iIntersectResult() {
		}
	};

};