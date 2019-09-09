#pragma once

#include "Primitive.h"
#include <fstream>
#include <vector>
#include <utility>
#include "Intersect.h"
#include <deque>
#include "bvh_intersectResult.h"
#include <mutex>

namespace aObject {

	/*
	template BVH class

	N is the number of children
	E is element of data structure,
	K is key from which elements are gotten using Intersect() function

	DEMAND
	E:
	 - Isc::Intersect(E, K),
	 - AABB::Expand(E),
	must be defined correctly.

	K:
	 - Isc::Intersect(K, AABB)
	must be defined correctly.

	*/
	template<typename E, typename K, int N>
	class N_BVH {
	public:
		typedef std::pair<unsigned int, E> ID_Element;

	private:
		struct Node;

		unsigned int m_NumNodes;

		int createChildNodes(Node& node, std::vector<ID_Element>& elements);
		void postProcess();
		bool divide(
			std::vector<ID_Element>& elements,
			std::vector<ID_Element>& left,
			std::vector<ID_Element>& right) const;

		std::vector<Node> m_NodeArray;
		std::vector<ID_Element> m_ElementArray;

		Primitive::AABB m_SceneAABB;

		float getDivideScore_SAH(int numTri_left, int numTri_right, float area_left, float area_right, float area_whole) const;
		int getDivideAxis(const std::vector<ID_Element>& elements) const;
		void writeCube(std::ostream& ost, const Primitive::AABB& aabb, unsigned int indexoffset) const;

		void exportBVH_rec(std::ostream& ost, unsigned int node_id, unsigned int& numVertex, int depth) const;
		bool isExistChild(const Node& node, unsigned int childID) const;

		bool findClosestElement(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int rootNode) const;
		bool findAnyElement(const Primitive::Ray& ray, unsigned int rootNode) const;

	public:

		Primitive::AABB GetSceneAABB() const {
			return m_SceneAABB;
		}
		PTUtility::Vec3 GetSceneWidth() const {
			return (m_SceneAABB.m_MaxPos - m_SceneAABB.m_MinPos);
		}
		PTUtility::Vec3 GetSceneCenter() const {
			return 0.5f * (m_SceneAABB.m_MinPos + m_SceneAABB.m_MaxPos);
		}

		void Build(const std::vector<E>& elements);
		void Delete();
		void ExportBVH(const char* objFileName) const;

		E GetElement(unsigned int elementID) const;

		bool ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray) const;
		bool ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int leafNodeID) const;
		bool AnyHit(const Primitive::Ray& ray) const;
		bool AnyHit(const Primitive::Ray& ray, unsigned int leafNodeID) const;

		bool Intersect(const K& key) const;

		int GetNumElement() const {
			return m_ElementArray.size();
		}

		N_BVH();
		~N_BVH();
	};


	// 2-BVH
	template<typename E, typename K>
	class N_BVH<E, K, 2> {
	public:
		typedef std::pair<unsigned int, E> ID_Element;

	private:
		struct Node {
			static constexpr unsigned int InvalidID = 0xffffffff;

			unsigned int node_parent;
			std::array<unsigned int, 2> node_children;
			unsigned int node_current;

			unsigned int elementID;
			Primitive::AABB aabb;

			Node(
				unsigned int _parent, const std::array<unsigned int, 2>& _children, unsigned int _current,
				unsigned int _elementID, const Primitive::AABB& _aabb) :
				node_parent(_parent), node_children(_children), node_current(_current), elementID(_elementID), aabb(_aabb) {
			}
			Node() {
				Node(InvalidID, { InvalidID, InvalidID }, InvalidID, InvalidID, Primitive::AABB());
			}
		};
		unsigned int m_NumNodes;

		int createChildNodes(Node& node, std::vector<ID_Element>& elements);
		int createChildNodes(Node& node, std::vector<ID_Element>& elements, int startID, int endID);

		void postProcess();
		bool divide(
			std::vector<ID_Element>& elements,
			std::vector<ID_Element>& left,
			std::vector<ID_Element>& right,
			int& axis) const;
		int divide(std::vector<ID_Element>& elements, int startID, int endID, int& axis) const;

		std::vector<Node> m_NodeArray;
		std::vector<ID_Element> m_ElementArray;

		Primitive::AABB m_SceneAABB;

		float getDivideScore_SAH(int numTri_left, int numTri_right, float area_left, float area_right, float area_whole) const;
		int getDivideAxis(const std::vector<ID_Element>& elements) const;
		int getDivideAxis(const std::vector<ID_Element>& elements, int startID, int endID) const;

		void writeCube(std::ostream& ost, const Primitive::AABB& aabb, unsigned int indexoffset) const;

		void exportBVH_rec(std::ostream& ost, unsigned int node_id, unsigned int& numVertex, int depth) const;
		bool isExistChild(const Node& node, unsigned int childID) const;

		bool findClosestElement(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int rootNode) const;
		bool findAnyElement(const Primitive::Ray& ray, unsigned int rootNode) const;

	public:

		Primitive::AABB GetSceneAABB() const {
			return m_SceneAABB;
		}
		PTUtility::Vec3 GetSceneWidth() const {
			return (m_SceneAABB.m_MaxPos - m_SceneAABB.m_MinPos);
		}
		PTUtility::Vec3 GetSceneCenter() const {
			return 0.5f* (m_SceneAABB.m_MinPos + m_SceneAABB.m_MaxPos);
		}

		void Build(const std::vector<E>& elements);
		void Build2(const std::vector<E>& elements);

		void Delete();
		void ExportBVH(const char* objFileName) const;

		E GetElement(unsigned int elementID) const;

		bool ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray) const;
		bool ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int leafNodeID) const;
		bool AnyHit(const Primitive::Ray& ray) const;
		bool AnyHit(const Primitive::Ray& ray, unsigned int leafNodeID) const;

		bool Intersect(const K& key) const;

		int GetNumElement() const {
			return m_ElementArray.size();
		}

		N_BVH();
		~N_BVH();
	};


	// 4-BVH
	template<typename E, typename K>
	class N_BVH<E, K, 4> {
	public:
		typedef std::pair<unsigned int, E> ID_Element;

	private:
		struct Node {
			static constexpr unsigned int InvalidID = 0xffffffff;

			Primitive::AABB4 m_childAABBs;
			unsigned int m_children[4];
			uint32_t m_axis;   // [0:1]=x, [2:3]=y, [4:5]=z, 
			unsigned int m_parentID;


			inline bool isLeaf() const {
				return m_children[3] == InvalidID;
			}
			void setLeafFlag() {
				m_children[3] = InvalidID;
			}
			int getAxis(int i) const {
				return (m_axis >> (i * 2)) & 0x3;
			}
			void setAxis(int i, uint32_t value) {
				m_axis = m_axis & (~(0x3 << (i * 2))) | (value << (i * 2));
			}

			Node(const std::array<Primitive::AABB, 4> & _AABBs, unsigned int _child[4], int _axis[3]) :
				m_childAABBs(_AABBs), m_children(_child), m_axis(_axis), m_parentID(0) {
			}
			Node() : m_childAABBs(), m_children{ InvalidID , InvalidID , InvalidID , InvalidID }, m_axis(0), m_parentID(InvalidID) {}
		};
		unsigned int m_NumNodes;

		int createChildNodes(Node & node, std::vector<ID_Element> & elements);
		int createChildNodes(Node& node, std::vector<ID_Element>& elements, int startID, int endID);

		void postProcess();
		bool divide(
			std::vector<ID_Element> & elements,
			std::vector<ID_Element> & left,
			std::vector<ID_Element> & right,
			int& axis) const;
		int divide(std::vector<ID_Element>& elements, int startID, int endID, int& axis) const;

		std::vector<Node> m_NodeArray;
		std::vector<ID_Element> m_ElementArray;

		Primitive::AABB m_SceneAABB;

		float getDivideScore_SAH(int numTri_left, int numTri_right, float area_left, float area_right, float area_whole) const;
		int getDivideAxis(const std::vector<ID_Element> & elements) const;
		int getDivideAxis(const std::vector<ID_Element>& elements, int startID, int endID) const;

		void writeCube(std::ostream & ost, const Primitive::AABB & aabb, unsigned int indexoffset) const;

		void exportBVH_rec(std::ostream & ost, unsigned int node_id, unsigned int& numVertex, int depth) const;
		bool isExistChild(const Node & node, unsigned int childID) const;

		bool findClosestElement(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int rootNode) const;
		bool findAnyElement(const Primitive::Ray& ray, unsigned int rootNode) const;

	public:

		Primitive::AABB GetSceneAABB() const {
			return m_SceneAABB;
		}
		PTUtility::Vec3 GetSceneWidth() const {
			return (m_SceneAABB.m_MaxPos - m_SceneAABB.m_MinPos);
		}
		PTUtility::Vec3 GetSceneCenter() const {
			return 0.5f* (m_SceneAABB.m_MinPos + m_SceneAABB.m_MaxPos);
		}

		void Build(const std::vector<E> & elements);
		void Build2(const std::vector<E>& elements);

		void Delete();
		void ExportBVH(const char* objFileName) const;

		E GetElement(unsigned int elementID) const;

		bool ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray) const;
		bool ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int leafNodeID) const;
		bool AnyHit(const Primitive::Ray& ray) const;
		bool AnyHit(const Primitive::Ray& ray, unsigned int leafNodeID) const;

		int GetNumElement() const {
			return m_ElementArray.size();
		}

		N_BVH();
		~N_BVH();
	};


	// 8-BVH
	template<typename E, typename K>
	class N_BVH<E, K, 8> {

		std::mutex mtx_;

	public:

		typedef std::pair<unsigned int, E> ID_Element;

	private:
		struct Node {
			static constexpr unsigned int InvalidID = 0xffffffff;

			Primitive::AABB8 m_childAABBs;
			unsigned int m_children[8];
			uint32_t m_axis;
			unsigned int m_parentID;

			inline bool isLeaf() const {
				return m_children[7] == InvalidID;
			}
			void setLeafFlag() {
				m_children[7] = InvalidID;
			}
			int getAxis(int i) const {
				return (m_axis >> (i * 2)) & 0x3;
			}
			void setAxis(int i, uint32_t value) {
				m_axis = m_axis & (~(0x3 << (i * 2))) | (value << (i * 2));
			}

			Node(const std::array<Primitive::AABB, 8> & _AABBs, unsigned int _child[8], int _axis[4]) :
				m_childAABBs(_AABBs), m_children(_child), m_axis(_axis), m_parentID(0) {
			}
			Node() : m_childAABBs(), m_children{ InvalidID , InvalidID , InvalidID , InvalidID, InvalidID , InvalidID , InvalidID , InvalidID }, m_axis(0), m_parentID(InvalidID) {}
		};
		unsigned int m_NumNodes;

		int createChildNodes(Node & node, std::vector<ID_Element> & elements);
		int createChildNodes(Node& node, std::vector<ID_Element>& elements, int startID, int endID, int NumDepth = 0);
		std::array<int, 9> create8Children(Node& node, std::vector<ID_Element>& elements, int startID, int endID);

		void postProcess();
		
		bool divide(
			std::vector<ID_Element> & elements,
			std::vector<ID_Element> & left,
			std::vector<ID_Element> & right,
			int& axis) const;

		int divide(
			std::vector<ID_Element>& elements,
			int startID, int endID, int& axis) const;

		std::vector<Node> m_NodeArray;
		std::vector<ID_Element> m_ElementArray;

		Primitive::AABB m_SceneAABB;

		float getDivideScore_SAH(int numTri_left, int numTri_right, float area_left, float area_right, float area_whole) const;

		int getDivideAxis(const std::vector<ID_Element> & elements) const;
		int getDivideAxis(const std::vector<ID_Element>& elements, int startID, int endID) const;

		void writeCube(std::ostream & ost, const Primitive::AABB & aabb, unsigned int indexoffset) const;

		void exportBVH_rec(std::ostream & ost, unsigned int node_id, unsigned int& numVertex, int depth) const;
		bool isExistChild(const Node & node, unsigned int childID) const;

		bool findClosestElement(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int rootNode) const;
		bool findAnyElement(const Primitive::Ray& ray, unsigned int rootNode) const;

		inline int incNumNode() {
			std::lock_guard<std::mutex> lock(mtx_);

			int result = m_NumNodes++;
			return result;
		}

	public:

		Primitive::AABB GetSceneAABB() const {
			return m_SceneAABB;
		}
		PTUtility::Vec3 GetSceneWidth() const {
			return (m_SceneAABB.m_MaxPos - m_SceneAABB.m_MinPos);
		}
		PTUtility::Vec3 GetSceneCenter() const {
			return 0.5f* (m_SceneAABB.m_MinPos + m_SceneAABB.m_MaxPos);
		}

		void Build(const std::vector<E> & elements);

		// In general, Build2() is faster than build()
		void Build2(const std::vector<E>& elements); 

		// cheap multi thread build
		void Build3(const std::vector<E>& elements);

		void Delete();
		void ExportBVH(const char* objFileName) const;

		E GetElement(unsigned int elementID) const;

		bool ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray) const;
		bool ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int leafNodeID) const;
		bool AnyHit(const Primitive::Ray& ray) const;
		bool AnyHit(const Primitive::Ray& ray, unsigned int leafNodeID) const;

		int GetNumElement() const {
			return m_ElementArray.size();
		}

		N_BVH();
		~N_BVH();
	};
}

#include "bvh2.h"
#include "bvh4.h"
#include "bvh8.h"
