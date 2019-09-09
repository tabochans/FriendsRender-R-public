
namespace aObject {

	using namespace Primitive;
	using namespace PTUtility;

	template<typename E, typename K>
	int N_BVH<E, K, 4>::createChildNodes(Node& node, std::vector<ID_Element>& elements) {

		std::vector<ID_Element> divide4[6]; int axis[3];
		divide(elements, divide4[4], divide4[5], axis[0]);
		divide(divide4[4], divide4[0], divide4[1], axis[1]);
		divide(divide4[5], divide4[2], divide4[3], axis[2]);

		node.setAxis(0, axis[0]); node.setAxis(1, axis[1]); node.setAxis(2, axis[2]);

		std::array<AABB, 4> aabbs;
		for (int i = 0; i < 4; ++i) {
			if (divide4[i].size() > 1) {
				// more than two triangles exist

				Node& childNode = m_NodeArray[node.m_children[i]];

				for (const auto& t : divide4[i]) {
					aabbs[i].Expand(t.second);
				}
				//// node.m_axis = axis[i > 3];

				for (int c = 0; c < 4; ++c) {
					childNode.m_children[c] = m_NumNodes++;
					m_NodeArray[childNode.m_children[c]].m_parentID = node.m_children[c];
				}
			}
			else if (divide4[i].size() == 1) {
				// only one triangle exists
				// create leaf node which points triangle ID instead of child nodes. 

				aabbs[i].Expand(divide4[i][0].second);

				Node& childNode = m_NodeArray[node.m_children[i]];
				childNode.m_children[0] = divide4[i][0].first;

			}
			else {
				// no triangle
				// insert dummy triangle

				aabbs[i].Reset();
			}
		}

		node.m_childAABBs = Primitive::AABB4(aabbs);
		for (int i = 0; i < 4; ++i) {
			if (divide4[i].size() > 1) {
				createChildNodes(m_NodeArray[node.m_children[i]], divide4[i]);
			}
		}

		return 0;
	}

	template<typename E, typename K>
	int N_BVH<E, K, 4>::createChildNodes(Node& node, std::vector<ID_Element>& elements, int startID, int endID) {
		int dIndex[5]; int axis[3];

		dIndex[0] = startID; 
		dIndex[4] = endID;

		dIndex[2] = divide(elements, startID, endID, axis[0]);
		dIndex[1] = divide(elements, startID, dIndex[2], axis[1]);
		dIndex[3] = divide(elements, dIndex[2], endID, axis[2]);

		node.setAxis(0, axis[0]); node.setAxis(1, axis[1]); node.setAxis(2, axis[2]);

		std::array<AABB, 4> aabbs;
		for (int i = 0; i < 4; ++i) {

			int numElement = dIndex[i + 1] - dIndex[i];

			if (numElement > 1) {
				// more than two triangles exist

				Node& childNode = m_NodeArray[node.m_children[i]];

				for (int j = dIndex[i]; j < dIndex[i + 1]; ++j) {
					aabbs[i].Expand(elements[j].second);
				}

				for (int c = 0; c < 4; ++c) {
					childNode.m_children[c] = m_NumNodes++;
					m_NodeArray[childNode.m_children[c]].m_parentID = node.m_children[c];
				}
			}
			else if (numElement == 1) {
				// only one triangle exists
				// create leaf node which points triangle ID instead of child nodes. 

				aabbs[i].Expand(elements[dIndex[i]].second);

				Node& childNode = m_NodeArray[node.m_children[i]];
				childNode.m_children[0] = dIndex[i];
			}
			else {
				aabbs[i].Reset();
			}
		}

		node.m_childAABBs = Primitive::AABB4(aabbs);
		for (int i = 0; i < 4; ++i) {
			if (dIndex[i + 1] - dIndex[i] > 1) {
				createChildNodes(m_NodeArray[node.m_children[i]], elements, dIndex[i], dIndex[i + 1]);
			}
		}

		return 0;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 4>::postProcess() {
		m_NodeArray.resize(m_NumNodes + 10);
		return;
	}

	template<typename E, typename K>
	bool aObject::N_BVH<E, K, 4>::divide(std::vector<ID_Element>& elements,
		std::vector<ID_Element>& left, std::vector<ID_Element>& right, int& axis) const {

		axis = 0;

		if (elements.size() <= 1) {
			for (int i = 0; i < elements.size(); ++i) {
				left.push_back(elements[i]);
			}
			return false;
		}

		// decide axis for divide
		axis = getDivideAxis(elements);
		std::sort(elements.begin(), elements.end(), [axis](const ID_Element & left, const ID_Element & right) {return left.second.GetCenter()[axis] < right.second.GetCenter()[axis]; });

		std::vector<float> area_right;
		area_right.resize(elements.size() + 1);

		// calculate surface area of right aabb
		AABB aabb_right;
		for (int i = 1; i < elements.size() + 1; ++i) {
			aabb_right.Expand(elements[elements.size() - i].second);
			area_right[i] = aabb_right.GetArea();
		}
		area_right[0] = 0.0f;

		// surface area of AABB which includes all elements
		const float area_whole = area_right[elements.size()];

		// calculate divide point
		AABB aabb_left;
		float min_score = FLT_MAX; int num_leftElement = 0;
		for (int i = 1; i < elements.size(); ++i) {
			float area_left;
			aabb_left.Expand(elements[i - 1].second);
			area_left = aabb_left.GetArea();

			float score = getDivideScore_SAH(i, elements.size() - i, area_left, area_right[elements.size() - i], area_whole);
			if (score < min_score) { min_score = score; num_leftElement = i; }
		}

		// create left and right triangle array
		left.resize(num_leftElement, elements[0]);
		right.resize(elements.size() - num_leftElement, elements[0]);
		for (int i = 0; i < num_leftElement; ++i) {
			left[i] = elements[i];
		}
		for (int i = 0; i < elements.size() - num_leftElement; ++i) {
			right[i] = elements[i + num_leftElement];
		}

		return true;
	}

	template<typename E, typename K>
	int aObject::N_BVH<E, K, 4>::divide(std::vector<ID_Element>& elements, int startID, int endID, int& axis) const {

		axis = 0;
		if (endID - startID <= 1) {
			return endID;
		}

		// decide axis for divide
		axis = getDivideAxis(elements, startID, endID);
		std::sort(elements.begin() + startID, elements.begin() + endID, [axis](const ID_Element & left, const ID_Element & right) {return left.second.GetCenter()[axis] < right.second.GetCenter()[axis]; });

		std::vector<float> area_right;
		area_right.resize(endID - startID + 1);

		// calculate surface area of right aabb
		AABB aabb_right;
		for (int i = 1; i < endID - startID + 1; ++i) {
			aabb_right.Expand(elements[endID - i].second);
			area_right[i] = aabb_right.GetArea();
		}
		area_right[0] = 0.0f;

		// surface area of AABB which includes all elements
		const float area_whole = area_right[endID - startID];

		// calculate divide point
		AABB aabb_left;
		float min_score = FLT_MAX; int num_leftElement = 0;
		for (int i = 1; i < endID - startID + 1; ++i) {
			float area_left;
			aabb_left.Expand(elements[startID + i - 1].second);
			area_left = aabb_left.GetArea();

			float score = getDivideScore_SAH(i, endID - startID - i, area_left, area_right[endID - startID - i], area_whole);
			if (score < min_score) { min_score = score; num_leftElement = i; }
		}

		return num_leftElement + startID;
	}

	template<typename E, typename K>
	float N_BVH<E, K, 4>::getDivideScore_SAH(int numElm_left, int numElm_right, float area_left, float area_right, float area_whole) const {
		const static float T_aabb = 1.0f;
		const static float T_tri = 2.0f;
		return 2.0f* T_aabb + (area_left * numElm_left + area_right * numElm_right) * (T_tri / area_whole) + 0.001f / (numElm_right * numElm_left + 1.0f);
	}

	template<typename E, typename K>
	int N_BVH<E, K, 4>::getDivideAxis(const std::vector<ID_Element> & elements) const {

		float maxVar = 0.0f;
		int axis = 0;

		for (int i = 0; i < 3; ++i) {
			float avg = 0.0f;
			for (const auto& t : elements) {
				avg += t.second.GetCenter()[i];
			}
			avg /= (float)elements.size();

			float var = 0.0f;
			for (const auto& t : elements) {
				var += std::pow(avg - t.second.GetCenter()[i], 2.0f);
			}
			if (maxVar < var) {
				maxVar = var;
				axis = i;
			}
		}
		return axis;
	}

	template<typename E, typename K>
	int N_BVH<E, K, 4>::getDivideAxis(const std::vector<ID_Element>& elements, int startID, int endID) const {

		if (endID - startID <= 0) { return 0; }

		float maxVar = 0.0f;
		int axis = 0;

		for (int i = 0; i < 3; ++i) {
			float avg = 0.0f;
			for (int j = startID; j < endID; ++j) {
				avg += (elements[j].second).GetCenter()[i];
			}
			avg /= (float)(endID - startID);

			float var = 0.0f;
			for (int j = startID; j < endID; ++j) {
				var += std::pow(avg - (elements[j].second).GetCenter()[i], 2.0f);
			}
			if (maxVar < var) {
				maxVar = var;
				axis = i;
			}
		}
		return axis;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 4>::writeCube(std::ostream & ost, const Primitive::AABB & aabb, unsigned int indexoffset) const {

		const Vec3 center = 0.5f * (aabb.m_MinPos + aabb.m_MaxPos);
		const Vec3 width = aabb.m_MaxPos - aabb.m_MinPos;

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
			const Vec3 pos = center + 0.5 * width * Vec3(vlist[i][0], vlist[i][1], vlist[i][2]);
			ost << "v " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
		}

		// write index
		ost << "f " << 1 + indexoffset << " " << 4 + indexoffset << " " << 3 + indexoffset << " " << indexoffset + 2 << std::endl;
		ost << "f " << 1 + indexoffset << " " << 5 + indexoffset << " " << 8 + indexoffset << " " << indexoffset + 4 << std::endl;
		ost << "f " << 2 + indexoffset << " " << 6 + indexoffset << " " << 5 + indexoffset << " " << indexoffset + 1 << std::endl;
		ost << "f " << 3 + indexoffset << " " << 7 + indexoffset << " " << 6 + indexoffset << " " << indexoffset + 2 << std::endl;
		ost << "f " << 4 + indexoffset << " " << 8 + indexoffset << " " << 7 + indexoffset << " " << indexoffset + 3 << std::endl;
		ost << "f " << 6 + indexoffset << " " << 7 + indexoffset << " " << 8 + indexoffset << " " << indexoffset + 5 << std::endl;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 4>::exportBVH_rec(std::ostream & ost, unsigned int node_id, unsigned int& numVertex, int depth) const {

		const Node& node = m_NodeArray[node_id];

		if (node.isLeaf()) {
			// current node is leaf node
		}
		else {
			// current node is not leaf node

			for (int c = 0; c < 4; ++c) {

				AABB aabb = AABB(
					Vec3(node.m_childAABBs.m_Data[1][0][c], node.m_childAABBs.m_Data[1][1][c], node.m_childAABBs.m_Data[1][2][c]),
					Vec3(node.m_childAABBs.m_Data[0][0][c], node.m_childAABBs.m_Data[0][1][c], node.m_childAABBs.m_Data[0][2][c])
				);
				Isc::IntersectInfo res;

				if (Isc::Intersect(aabb, Vertex(aabb.GetCenter()), res)) {
					// if child AABB is valid

					writeCube(ost, aabb, numVertex);
					numVertex += 8;

					exportBVH_rec(ost, node.m_children[c], numVertex, ++depth);
				}
			}
		}
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 4>::isExistChild(const Node & node, unsigned int childID) const {
		return false;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 4>::Build(const std::vector<E> & elements) {
		if (elements.size() == 0) {
			return;
		}

		// Copy triangle array
		m_ElementArray.reserve(elements.size());
		for (int i = 0; i < elements.size(); ++i) {
			m_ElementArray.push_back(ID_Element(i, elements[i]));
		}
		std::vector<ID_Element> temp(m_ElementArray);

		// Create aabb including all triangles
		m_SceneAABB = AABB();
		for (const auto& t : m_ElementArray) {
			m_SceneAABB.Expand(t.second);
		}

		// Create Root node
		m_NodeArray.resize(elements.size() * 2 + 64);
		m_NodeArray[m_NumNodes++] = Node();

		Node cNode; cNode.m_parentID = 0;
		for (int c = 0; c < 4; ++c) {
			m_NodeArray[m_NodeArray[0].m_children[c] = m_NumNodes++] = cNode;
		}

		// Create node tree using recursive function
		createChildNodes(m_NodeArray[0], temp);

		// post process (sort triangle arrays, nodes, and so on)
		postProcess();

		return;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 4>::Build2(const std::vector<E>& elements) {
		if (elements.size() == 0) {
			return;
		}

		// Copy triangle array
		m_ElementArray.reserve(elements.size());
		for (int i = 0; i < elements.size(); ++i) {
			m_ElementArray.push_back(ID_Element(i, elements[i]));
		}

		// Create aabb including all triangles
		m_SceneAABB = AABB();
		for (const auto& t : m_ElementArray) {
			m_SceneAABB.Expand(t.second);
		}

		// Create Root node
		m_NodeArray.resize(elements.size() * 4 + 64);
		m_NodeArray[m_NumNodes++] = Node();

		Node cNode; cNode.m_parentID = 0;
		for (int c = 0; c < 4; ++c) {
			m_NodeArray[m_NodeArray[0].m_children[c] = m_NumNodes++] = cNode;
		}

		// Create node tree using recursive function
		createChildNodes(m_NodeArray[0], m_ElementArray, 0, m_ElementArray.size());

		// post process (sort triangle arrays, nodes, and so on)
		postProcess();

		return;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 4>::Delete() {
	}

	template<typename E, typename K>
	void N_BVH<E, K, 4>::ExportBVH(const char* objFileName) const {
		std::ofstream file(objFileName);

		unsigned int vertex = 0;
		exportBVH_rec(file, 0, vertex, 0);
		file.close();
	}

	template<typename E, typename K>
	E N_BVH<E, K, 4>::GetElement(unsigned int elementID) const {
		return m_ElementArray[elementID].second;
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 4>::findClosestElement(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int rootNode) const {
		std::deque<unsigned int> stack;
		stack.push_back(rootNode);

		result = IntersectResult_RayTriangle();
		result._t = ray.m_MaxT;

		while (stack.size() > 0) {
			const unsigned int nodeID = *(stack.end() - 1);
			const Node & node = m_NodeArray[nodeID];
			stack.pop_back();

			if (node.isLeaf()) {
				// if node is leaf node

				if (node.m_children[0] != Node::InvalidID) {

					// intersection test with element triangle and ray
					Isc::IntersectInfo info;
					if (Isc::Intersect(m_ElementArray[node.m_children[0]].second, ray, info)) {
						if (result._t >= info.t && info.t > ray.m_MinT) {
							result._t = info.t;
							result._u = info.u;
							result._v = info.v;
							result._ElementID = node.m_children[0];
							result._LeafNodeID = nodeID;
						}
					}
				}
			}
			else {
				// if node is not leaf node

				// create object for sorting
				std::array<std::pair<unsigned int, float>, 4> res({ std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX) });
				std::array<float, 4> hitDistance;

				if (!Isc::Intersect_AABB4_RAY(node.m_childAABBs, ray, hitDistance)) {
					continue;
				}

				// sort by node.m_axis
				const bool offset = ray.m_Dir[node.getAxis(0)] > 0;
				const bool offset0 = ray.m_Dir[node.getAxis(1)] > 0;
				const bool offset1 = ray.m_Dir[node.getAxis(2)] > 0;
				res[2 * offset + offset0] = std::pair<unsigned int, float>(0, hitDistance[0]); // min-min 
				res[2 * offset + !offset0] = std::pair<unsigned int, float>(1, hitDistance[1]); // min-max
				res[2 * !offset + offset1] = std::pair<unsigned int, float>(2, hitDistance[2]); // max-min
				res[2 * !offset + !offset1] = std::pair<unsigned int, float>(3, hitDistance[3]); // max-max

				// push child nodes accounting for distance from ray origin
				for (int i = 0; i < res.size(); ++i) {
					if (res[i].second < result._t) {
						stack.push_back(node.m_children[res[i].first]);
					}
				}
			}
		}
		return result._t < ray.m_MaxT - 1e-10;
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 4>::findAnyElement(const Primitive::Ray& ray, unsigned int rootNode) const {
		std::deque<unsigned int> stack;
		stack.push_back(rootNode);

		while (stack.size() > 0) {
			const unsigned int nodeID = *(stack.end() - 1);
			const Node & node = m_NodeArray[nodeID];
			stack.pop_back();

			if (node.isLeaf()) {
				// if node is leaf node

				if (node.m_children[0] != Node::InvalidID) {

					// intersection test with element triangle and ray
					Isc::IntersectInfo info;
					if (Isc::Intersect(m_ElementArray[node.m_children[0]].second, ray, info)) {
						if (ray.m_MaxT >= info.t && info.t > ray.m_MinT) {
							return true;
						}
					}
				}
			}
			else {
				// if node is not leaf node

				// create object for sorting
				std::array<std::pair<unsigned int, float>, 4> res({ std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX) });
				std::array<float, 4> hitDistance;

				if (!Isc::Intersect_AABB4_RAY(node.m_childAABBs, ray, hitDistance)) {
					continue;
				}

				// sort by node.m_axis
				const bool offset = ray.m_Dir[node.getAxis(0)] > 0;
				const bool offset0 = ray.m_Dir[node.getAxis(1)] > 0;
				const bool offset1 = ray.m_Dir[node.getAxis(2)] > 0;
				res[2 * offset + offset0] = std::pair<unsigned int, float>(0, hitDistance[0]); // min-min 
				res[2 * offset + !offset0] = std::pair<unsigned int, float>(1, hitDistance[1]); // min-max
				res[2 * !offset + offset1] = std::pair<unsigned int, float>(2, hitDistance[2]); // max-min
				res[2 * !offset + !offset1] = std::pair<unsigned int, float>(3, hitDistance[3]); // max-max

				// push child nodes accounting for distance from ray origin
				for (int i = 0; i < res.size(); ++i) {
					if (res[i].second < ray.m_MaxT) {
						stack.push_back(node.m_children[res[i].first]);
					}
				}
			}
		}
		return false;
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 4>::ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray) const {
		return findClosestElement(result, ray, 0);
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 4>::ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int leafNodeID) const {
		
		unsigned int nodeID = leafNodeID;
		unsigned int fromID = Node::InvalidID;

		result = IntersectResult_RayTriangle();
		result._t = ray.m_MaxT;

		while ((nodeID = m_NodeArray[nodeID].m_parentID) != Node::InvalidID) {
			const Node& n = m_NodeArray[nodeID];

			// create object for sorting
			std::array<std::pair<unsigned int, float>, 4> res({ std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX) });
			std::array<float, 4> hitDistance;

			if (Isc::Intersect_AABB4_RAY(n.m_childAABBs, ray, hitDistance)) {

				// sort by node.m_axis
				const bool offset = ray.m_Dir[n.getAxis(0)] > 0;
				const bool offset0 = ray.m_Dir[n.getAxis(1)] > 0;
				const bool offset1 = ray.m_Dir[n.getAxis(2)] > 0;
				res[2 * offset + offset0] = std::pair<unsigned int, float>(0, hitDistance[0]); // min-min 
				res[2 * offset + !offset0] = std::pair<unsigned int, float>(1, hitDistance[1]); // min-max
				res[2 * !offset + offset1] = std::pair<unsigned int, float>(2, hitDistance[2]); // max-min
				res[2 * !offset + !offset1] = std::pair<unsigned int, float>(3, hitDistance[3]); // max-max

				for (int i = 0; i < 4; ++i) {
					if (n.m_children[res[i].first] != fromID && res[i].second < result._t) {
						IntersectResult_RayTriangle r;
						if (findClosestElement(r, Ray(ray.m_Org, ray.m_Dir, 1.0f, ray.m_MinT, result._t), n.m_children[res[i].first])) {
							result = r;
						}
					}
				}
			}
			fromID = nodeID;
		}
		return result._t < ray.m_MaxT - 1e-10;
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 4>::AnyHit(const Primitive::Ray& ray) const {
		return findAnyElement(ray, 0);
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 4>::AnyHit(const Primitive::Ray& ray, unsigned int leafNodeID) const {
		unsigned int nodeID = leafNodeID;
		unsigned int fromID = Node::InvalidID;

		while ((nodeID = m_NodeArray[nodeID].m_parentID) != Node::InvalidID) {
			const Node& n = m_NodeArray[nodeID];

			// create object for sorting
			std::array<std::pair<unsigned int, float>, 4> res({ std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX),std::pair<unsigned int, float>(-1, FLT_MAX) });
			std::array<float, 4> hitDistance;

			if (Isc::Intersect_AABB4_RAY(n.m_childAABBs, ray, hitDistance)) {

				// sort by node.m_axis
				const bool offset = ray.m_Dir[n.getAxis(0)] > 0;
				const bool offset0 = ray.m_Dir[n.getAxis(1)] > 0;
				const bool offset1 = ray.m_Dir[n.getAxis(2)] > 0;
				res[2 * offset + offset0] = std::pair<unsigned int, float>(0, hitDistance[0]); // min-min 
				res[2 * offset + !offset0] = std::pair<unsigned int, float>(1, hitDistance[1]); // min-max
				res[2 * !offset + offset1] = std::pair<unsigned int, float>(2, hitDistance[2]); // max-min
				res[2 * !offset + !offset1] = std::pair<unsigned int, float>(3, hitDistance[3]); // max-max

				for (int i = 0; i < 4; ++i) {
					if (n.m_children[res[i].first] != fromID && res[i].second < ray.m_MaxT) {
						if (findAnyElement(ray, n.m_children[res[i].first])) {
							return true;
						}
					}
				}
			}
			fromID = nodeID;
		}
		return false;
	}

	template<typename E, typename K>
	N_BVH<E, K, 4>::N_BVH() {
	}

	template<typename E, typename K>
	N_BVH<E, K, 4>::~N_BVH() {
	}
};