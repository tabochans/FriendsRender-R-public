#pragma once
#include <vector>
#include <ostream>
#include <fstream>
#include "MT.h"
#include "Primitive.h"
#include "Vec.h"

namespace tgraph {

	template<typename T>
	struct Node {
		T element;

		enum AXIS {
			AX_X = 0,
			AX_Y = 1,
			AX_Z = 2
		};
		AXIS axis;

		const Node* parent;
		const Node* children[2];

		Node(const T& _element, AXIS _axis, const Node* _parent, const Node* _children1, const Node* _children2) :
			element(_element), axis(_axis), parent(_parent), children{ _children1, _children2 } {
		}
		Node() : element(), axis(AX_X), parent(nullptr), children{ nullptr, nullptr } {}
	};

	template<typename T>
	class kdtree {
		static constexpr int Num_Bin = 8;
		int m_NumNode;

		mutable RandomMT m_MT;

	private:
		mutable std::vector<std::vector<int>> m_indexBin;

		std::vector<Node<T>> m_Nodes;
		const Node<T>* RootNode;

		struct BoundingBox {
			float xyz_minmax[6];
			BoundingBox(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max) :
				xyz_minmax{ x_min, x_max, y_min, y_max, z_min, z_max } {
			}
			BoundingBox() {
				BoundingBox(FLT_MAX, FLT_MIN, FLT_MAX, FLT_MIN, FLT_MAX, FLT_MIN);
			}
		};
		BoundingBox m_BoundingBox;

		bool divide(const std::vector<T>& elements, Node<T>& newNode, std::vector<T>& left, std::vector<T>& right) const;
		int getDivideIndex(const std::vector<T>& elements, typename Node<T>::AXIS axis) const;
		void getBoundingBox(const std::vector<T>& elements, std::array<float, 3>& minX, std::array<float, 3>& maxX) const;
		float getScore(const std::array<int, Num_Bin>& bin, int NumElement, float Length, float SliceArea, int dIndex, typename Node<T>::AXIS axis) const;

		float getAverage(const std::vector<T>& elements, typename Node<T>::AXIS axis) const;
		float getVariance(const std::vector<T>& elements, float average, typename Node<T>::AXIS axis) const;

		const Node<T>* createTree_rec(const std::vector<T>& elements, const Node<T>* parent);

		const Node<T>* getNearestRegion(const PTUtility::Vec3& point) const;
		const Node<T>* getNearestRegion_rec(const PTUtility::Vec3& point, const Node<T>* parent) const;

		void GetNearPointInRegion(const PTUtility::Vec3& point, PTUtility::Vec3& candidate, const Node<T>* node, BoundingBox& bb) const;

		void NaiveSearch(const PTUtility::Vec3& point, PTUtility::Vec3& candidate, const Node<T>* node) const;
		void expandBB(BoundingBox& bb, const PTUtility::Vec3& point) const;
		bool isInterBB(const BoundingBox& bb, const PTUtility::Vec3& point, const PTUtility::Vec3& candidate) const;
		bool isBBIncludesbb(const BoundingBox& BB, const BoundingBox& bb) const;
		void getPointsInsideBox_rec(std::vector<T>& points, const PTUtility::Vec3& AABBmin, const PTUtility::Vec3& AABBmax, const Node<T>* node) const;

		void exportTree_rec(const Node<T>* node, const Primitive::AABB& aabb, int depth, std::ofstream& file, int& offset) const;

		static bool isIncludePoint(const PTUtility::Vec3& point, const PTUtility::Vec3& min, const PTUtility::Vec3& max) {
			for (int i = 0; i < 3; ++i) {
				if (min[i] > point[i]) {
					return false;
				}
				if (max[i] < point[i]) {
					return false;
				}
			}
			return true;
		}

		static bool writeBox(std::ostream& ost, const Primitive::AABB& aabb, unsigned int indexoffset) {
			const PTUtility::Vec3 center = 0.5f * (aabb.m_MinPos + aabb.m_MaxPos);
			const PTUtility::Vec3 width = aabb.m_MaxPos - aabb.m_MinPos;

			// write 8 vertices forming Cube object
			constexpr int vlist[8][3] = {
				{-1,-1, 1},
				{-1, 1, 1},
				{ 1, 1, 1},
				{ 1,-1, 1},
				{-1,-1,-1},
				{-1, 1,-1},
				{ 1, 1,-1},
				{ 1,-1,-1}
			};
			for (int i = 0; i < 8; ++i) {
				const PTUtility::Vec3 pos = center + 0.5 * width * PTUtility::Vec3(vlist[i][0], vlist[i][1], vlist[i][2]);
				ost << "v " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
			}

			// write index
			ost << "f " << 1 + indexoffset << " " << 4 + indexoffset << " " << 3 + indexoffset << " " << indexoffset + 2 << std::endl;
			ost << "f " << 1 + indexoffset << " " << 5 + indexoffset << " " << 8 + indexoffset << " " << indexoffset + 4 << std::endl;
			ost << "f " << 2 + indexoffset << " " << 6 + indexoffset << " " << 5 + indexoffset << " " << indexoffset + 1 << std::endl;
			ost << "f " << 3 + indexoffset << " " << 7 + indexoffset << " " << 6 + indexoffset << " " << indexoffset + 2 << std::endl;
			ost << "f " << 4 + indexoffset << " " << 8 + indexoffset << " " << 7 + indexoffset << " " << indexoffset + 3 << std::endl;
			ost << "f " << 6 + indexoffset << " " << 7 + indexoffset << " " << 8 + indexoffset << " " << indexoffset + 5 << std::endl;

			return 0;
		}

	public:

		int CreateTree(const std::vector<T>& points);
		T GetNearestElement(const PTUtility::Vec3& point);
		const T GetNearestElement(const PTUtility::Vec3& point) const;
		int GetElementsInsideBox(std::vector<T>& points, const PTUtility::Vec3& AABBmin, const PTUtility::Vec3& AABBmax) const;

		void ExportTree(const char* fileName, int MaxDepth) const;

		kdtree();
		virtual ~kdtree();


	};

};

namespace tgraph {

	template<typename T>
	bool kdtree<T>::divide(const std::vector<T>& elements, Node<T> & newNode, std::vector<T>& left, std::vector<T>& right) const {

		if (elements.size() == 0) {
			return 0;
		}
		else if (elements.size() == 1) {
			newNode.axis = Node<T>::AXIS::AX_X;
			newNode.element = elements[0];

			return 0;
		}

		float max_var = 0.0f;
		float average[3] = {0,0,0};
		typename Node<T>::AXIS max_axis = Node<T>::AXIS::AX_X;
		
		for (int dim = 0; dim < 3; ++dim) {
			average[dim] = getAverage(elements, (typename Node<T>::AXIS)dim);
			float var = getVariance(elements, average[dim], (typename Node<T>::AXIS)dim);
			
			if (var > max_var) {
				max_var = var;
				max_axis = (typename Node<T>::AXIS)dim;
			}
		}
		newNode.axis = (typename Node<T>::AXIS)max_axis;
		int axis = newNode.axis;

		
		// elements are divided into right and left 
		if(0){
			// divide point is calculated by sorting elements. It's really costly.

			std::vector<T> elm = elements;
			std::sort(elm.begin(), elm.end(), [&axis](const T & a, const T & b) { return a.Pos()[axis] < b.Pos()[axis]; });

			float dividePos = (elm[0].Pos()[axis] + elm[elm.size() - 1].Pos()[axis]) / 2.0f;

			int DividePoint = elm.size() / 2.0f;
			for (int i = 1; i < elm.size(); ++i) {
				if (elm[i - 1].Pos()[axis] < dividePos && elm[i].Pos()[axis] + 0.0001 >= dividePos) {
					DividePoint = i;
					break;
				}
			}
			
			newNode.element = elm[DividePoint];
			for (int i = 0; i < elm.size(); ++i) {
				if (i == DividePoint) {
					continue;
				}
				else if (i < DividePoint) {
					left.push_back(elm[i]);
				}
				else {
					right.push_back(elm[i]);
				}
			}
		}
		else {
			// divide point is calculated by binning and SAH. 


			int divideIndex = getDivideIndex(elements, newNode.axis);
			newNode.element = elements[divideIndex];

			left.reserve(elements.size() / 2);
			right.reserve(elements.size() / 2);
			for (int i = 0; i < elements.size(); ++i) {
				if (i == divideIndex) {

				}
				else if (elements[i].Pos()[axis] < newNode.element.Pos()[axis]) {
					left.push_back(elements[i]);
				}
				else {
					right.push_back(elements[i]);
				}
			}
		}

		return right.size() + left.size();
	}

	template<typename T>
	void kdtree<T>::getBoundingBox(const std::vector<T>& elements, std::array<float, 3>& minX, std::array<float, 3>& maxX) const {
		
		minX = std::array<float, 3>{ 1e10, 1e10, 1e10 };
		maxX = std::array<float, 3>{ -1e10 ,-1e10 ,-1e10 };
		
		for (const auto& e : elements) {
			for (int i = 0; i < 3; ++i) {
				if (minX[i] > e.Pos()[i]) {
					minX[i] = e.Pos()[i];
				}
				if (maxX[i] < e.Pos()[i]) {
					maxX[i] = e.Pos()[i];
				}
			}
		}
	}

	template<typename T>
	float kdtree<T>::getScore(const std::array<int, Num_Bin>& bin, int NumElement, float Length, float SliceArea, int dIndex, typename Node<T>::AXIS axis) const {
		
		int N_left = 0;
		float D_left = 0.0f;
		{
			int i0 = 0;
			for (i0 = 0; i0 <= dIndex && bin[i0] == 0; ++i0);

			int i1 = 0;
			for (i1 = dIndex; i1 >= 0 && bin[i1] == 0; --i1);

			for (int i = i0; i < i1; ++i) {
				N_left += bin[i];
			}
			D_left = (i1 - i0 + 1) * Length / (float)Num_Bin;
		}

		int N_right = 0;
		float D_right = 0.0f;
		{
			int i0 = 0;
			for (i0 = dIndex + 1; i0 < Num_Bin && bin[i0] == 0; ++i0);

			int i1 = 0;
			for (i1 = Num_Bin - 1; i1 > dIndex && bin[i1] == 0; --i1);

			for (int i = i0; i < i1; ++i) {
				N_right += bin[i];
			}
			D_right = (i1 - i0 + 1) * Length / (float)Num_Bin;
		}
		
		if (N_left <= 1 && dIndex == 0) {
			return 1e10;
		}
		if (N_right <= 1 && dIndex == Num_Bin - 1) {
			return 1e10;
		}

		return (D_left / ((float)N_left + 0.01f) + D_right / ((float)N_right + 0.01f)) * SliceArea / (float)NumElement;
	}


	template<typename T>
	int kdtree<T>::getDivideIndex(const std::vector<T>& elements, typename Node<T>::AXIS axis) const {

		const int N = elements.size();
		if (N <= 2) {
			return 0;
		}

		std::array<float, 3> minX, maxX;
		getBoundingBox(elements, minX, maxX);

		const float rangeMin = minX[axis];
		const float rangeMax = maxX[axis];
		const float D = (rangeMax - rangeMin) / (float)Num_Bin;

		const float S = (maxX[(axis + 1) % 3] - minX[(axis + 1) % 3]) * (maxX[(axis + 2) % 3] - minX[(axis + 2) % 3]);

		std::array<int, Num_Bin> Bin{};
		std::array<int, Num_Bin> Ptr{};
		for (int i = 0; i < elements.size(); ++i) {
			int index = std::max(0, std::min(Num_Bin - 1, int((elements[i].Pos()[axis] - rangeMin) / D)));
			Bin[index]++;
			m_indexBin[index][++Ptr[index]] = i;
		}

		float minScore = FLT_MAX;
		int divideBinIndex = 0;
		for (int i = 0; i < Num_Bin; ++i) {
			float score = getScore(Bin, elements.size(), rangeMax - rangeMin, S, i, axis);
			if (score < minScore) {
				minScore = score;
				divideBinIndex = i;
			}
		}

		return m_indexBin[divideBinIndex][Bin[divideBinIndex] * m_MT.genrand64_real2()];
	}

	template<typename T>
	float kdtree<T>::getAverage(const std::vector<T>& elements, typename Node<T>::AXIS axis) const {
		if (elements.size() == 0) { return 0; }
		float x = 0.0f;
		for (const auto& e : elements) {
			x += e.Pos()[axis];
		}
		return x / float(elements.size());
	}

	template<typename T>
	float kdtree<T>::getVariance(const std::vector<T>& elements, float average, typename Node<T>::AXIS axis) const {
		if (elements.size() == 0) {
			return 0;
		}

		float result = 0.0f;
		for (const auto& e : elements) {
			result += std::pow(e.Pos()[axis] - average, 2.0f);
		}
		return result / float(elements.size());
	}

	template<typename T>
	const Node<T>* kdtree<T>::createTree_rec(const std::vector<T>& elements, const Node<T>* parent) {
		if (elements.size() == 0) {
			return nullptr;
		}
		Node<T>& node = m_Nodes[m_NumNode++];
		node.parent = parent;

		std::vector<T> *right, *left;
		right = new std::vector<T>;
		left = new std::vector<T>;

		bool res = divide(elements, node, *left, *right);

		node.children[0] = createTree_rec(*left, &node);
		delete left;

		node.children[1] = createTree_rec(*right, &node);
		delete right;

		return &node;
	}

	template<typename T>
	const Node<T>* kdtree<T>::getNearestRegion(const PTUtility::Vec3 & point) const {
		return getNearestRegion_rec(point, &m_Nodes[0]);
	}

	template<typename T>
	const Node<T>* kdtree<T>::getNearestRegion_rec(const PTUtility::Vec3 & point, const Node<T>* parent) const {
		const Node<T>* next = (parent->element.Pos()[parent->axis] > point[parent->axis] ? parent->children[0] : parent->children[1]);
		if (next) {
			return getNearestRegion_rec(point, next);
		}
		else {
			return parent;
		}
	}

	template<typename T>
	void kdtree<T>::GetNearPointInRegion(const PTUtility::Vec3 & point, PTUtility::Vec3& candidate, const Node<T>* node, BoundingBox& bb) const {
		if (!node) { return; }

		if (point[node->axis] > node->element.Pos()[node->axis]) {
			if (bb.xyz_minmax[node->axis * 2] > node->element.Pos()[node->axis]) {
				bb.xyz_minmax[node->axis * 2] = node->element.Pos()[node->axis];
			}
		}
		else {
			if (bb.xyz_minmax[node->axis * 2 + 1] < node->element.Pos()[node->axis]) {
				bb.xyz_minmax[node->axis * 2 + 1] = node->element.Pos()[node->axis];
			}
		}

		if ((node->element.Pos() - point).norm2() < (candidate - point).norm2()) {
			candidate = node->element.Pos();
		}

		float dist = (candidate - point).norm();
		if (node->element.Pos()[node->axis] < point[node->axis] + dist) {
			// search left node
			NaiveSearch(point, candidate, node->children[1]);
		}
		if (node->element.Pos()[node->axis] > point[node->axis] - dist) {
			// search right node
			NaiveSearch(point, candidate, node->children[0]);
		}
	}

	template<typename T>
	int kdtree<T>::CreateTree(const std::vector<T>& elements) {
		m_Nodes.resize(elements.size() + 100);

		m_indexBin = std::vector<std::vector<int>>(Num_Bin, std::vector<int>(elements.size(), 0));

		m_NumNode = 0;
		RootNode = createTree_rec(elements, nullptr);

		for (const auto& e : elements) {
			expandBB(m_BoundingBox, e.Pos());
		}

		return 0;
	}

	template<typename T>
	T kdtree<T>::GetNearestElement(const PTUtility::Vec3& point) {

		const Node<T>* node = getNearestRegion(point);
		T candidate = node->element;
		BoundingBox bb(point[0], point[0], point[1], point[1], point[2], point[2]);

		GetNearPointInRegion(point, candidate.Pos(), node->children[0], bb);
		GetNearPointInRegion(point, candidate.Pos(), node->children[1], bb);
		
		while (node->parent && !isInterBB(bb, point, candidate.Pos())) {
			const Node<T>* parent = node->parent;
			candidate = (point - candidate.Pos()).norm2() < (point - parent->element.Pos()).norm2() ? candidate : parent->element;
			GetNearPointInRegion(point, candidate.Pos(), parent->children[point[parent->axis] < parent->element.Pos()[parent->axis]], bb);
			node = node->parent;
		}

		return candidate;
	}
	template<typename T>
	const T kdtree<T>::GetNearestElement(const PTUtility::Vec3& point) const {
		return GetNearestElement(point);
	}

	template<typename T>
	int kdtree<T>::GetElementsInsideBox(std::vector<T> & points, const PTUtility::Vec3& AABBmin, const PTUtility::Vec3& AABBmax) const {
		getPointsInsideBox_rec(points, AABBmin, AABBmax, &m_Nodes[0]);
		return points.size();
	}

	template<typename T>
	void kdtree<T>::getPointsInsideBox_rec(std::vector<T>& points, const PTUtility::Vec3& AABBmin, const PTUtility::Vec3& AABBmax, const Node<T>* node) const {

		const int axis = node->axis;
		const PTUtility::Vec3& pos = node->element.Pos();

		if (isIncludePoint(pos, AABBmin, AABBmax)) {
			points.push_back(node->element);
		}
		if (node->children[1]) {
			if (pos[axis] < AABBmax[axis]) {
				getPointsInsideBox_rec(points, AABBmin, AABBmax, node->children[1]);
			}
		}
		if (node->children[0]) {
			if (pos[axis] > AABBmin[axis]) {
				getPointsInsideBox_rec(points, AABBmin, AABBmax, node->children[0]);
			}
		}
	}


	template<typename T>
	void kdtree<T>::NaiveSearch(const PTUtility::Vec3& point, PTUtility::Vec3& candidate, const Node<T>* node) const {
		if (!node) {
			return;
		}

		candidate = (point - candidate).norm2() < (point - node->element.Pos()).norm2() ? candidate : node->element.Pos();
		float dist = (point - candidate).norm();

		if (node->element.Pos()[node->axis] < point[node->axis] + dist) {
			// search left node
			NaiveSearch(point, candidate, node->children[1]);
		}
		if (node->element.Pos()[node->axis] > point[node->axis] - dist) {
			// search right node
			NaiveSearch(point, candidate, node->children[0]);
		}
	}

	template<typename T>
	void kdtree<T>::expandBB(BoundingBox & bb, const PTUtility::Vec3 & point) const {
		for (int i = 0; i < 3; ++i) {
			if (bb.xyz_minmax[i * 2] > point[i]) {
				bb.xyz_minmax[i * 2] = point[i];
			}
			if (bb.xyz_minmax[i * 2 + 1] < point[i]) {
				bb.xyz_minmax[i * 2 + 1] = point[i];
			}
		}
	}

	template<typename T>
	bool kdtree<T>::isInterBB(const BoundingBox& bb, const PTUtility::Vec3 & point, const PTUtility::Vec3 & candidate) const {
		float dist = (point - candidate).norm();

		for (int i = 0; i < 3; ++i) {
			if (bb.xyz_minmax[i * 2] > point[i] - dist) {
				return false;
			}
			if (bb.xyz_minmax[i * 2 + 1] < point[i] + dist) {
				return false;
			}
		}
		return true;
	}

	template<typename T>
	bool kdtree<T>::isBBIncludesbb(const BoundingBox& BB, const BoundingBox& bb) const {
		for (int i = 0; i < 3; ++i) {
			if (bb.xyz_minmax[i * 2] > BB.xyz_minmax[i * 2]) {
				return false;
			}
			if (bb.xyz_minmax[i * 2 + 1] < BB.xyz_minmax[i * 2 + 1]) {
				return false;
			}
		}
		return true;
	}

	template<typename T>
	void kdtree<T>::exportTree_rec(const Node<T>* node, const Primitive::AABB& aabb, int depth, std::ofstream& file, int& offset) const {
		if (!node) { return; }
		if (depth <= 0) { return;  }
		
		const T& e = node->element;

		Primitive::AABB left, right; left = right = aabb;
		left.m_MaxPos[node->axis] = e.Pos()[node->axis];
		right.m_MinPos[node->axis] = e.Pos()[node->axis];

		writeBox(file, left, offset); offset += 8;
		writeBox(file, right, offset); offset += 8;

		exportTree_rec(node->children[0], left, depth - 1, file, offset);
		exportTree_rec(node->children[1], right, depth - 1, file, offset);	
	}

	template<typename T>
	void kdtree<T>:: ExportTree(const char* fileName, int MaxDepth) const {
		std::ofstream file(fileName);

		Primitive::AABB aabb(
			PTUtility::Vec3(m_BoundingBox.xyz_minmax[0], m_BoundingBox.xyz_minmax[2], m_BoundingBox.xyz_minmax[4]),
			PTUtility::Vec3(m_BoundingBox.xyz_minmax[1], m_BoundingBox.xyz_minmax[3], m_BoundingBox.xyz_minmax[5])
		);

		int count = 0;
		writeBox(file, aabb, count); count += 8;

		exportTree_rec(&m_Nodes[0], aabb, MaxDepth, file, count);

		file.close();
		
		return;
	}


	template<typename T>
	kdtree<T>::kdtree() {

	}

	template<typename T>
	kdtree<T>::~kdtree() {

	}


};
