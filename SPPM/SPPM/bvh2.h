
namespace aObject {

	using namespace Primitive;
	using namespace PTUtility;

	template<typename E, typename K>
	int N_BVH<E, K, 2>::createChildNodes(Node& node, std::vector<ID_Element>& elements) {
		std::vector<ID_Element> left, right;

		ID_Element elm0 = elements[0];
		int axis = -1;

		if (divide(elements, left, right, axis)) {
			// triangles are divided into left and right triangle array

			// create child nodes
			{
				AABB aabb_left, aabb_right;
				for (const auto& t : left) {
					aabb_left.Expand(t.second);
				}
				for (const auto& t : right) {
					aabb_right.Expand(t.second);
				}
				Node node_left(node.node_current, { Node::InvalidID, Node::InvalidID }, m_NumNodes++, Node::InvalidID, aabb_left);
				Node node_right(node.node_current, { Node::InvalidID, Node::InvalidID }, m_NumNodes++, Node::InvalidID, aabb_right);
				node.node_children[0] = node_left.node_current; m_NodeArray[node.node_children[0]] = node_left;
				node.node_children[1] = node_right.node_current; m_NodeArray[node.node_children[1]] = node_right;

				createChildNodes(m_NodeArray[node.node_children[0]], left);
				createChildNodes(m_NodeArray[node.node_children[1]], right);
			}
		}
		else {
			// triangles are not divided (ONLY when the size of triangles is 1)

			// create leaf node
			m_NodeArray[node.node_current].elementID = elm0.first;
			return 0;
		}

		return 0;
	}

	template<typename E, typename K>
	int N_BVH<E, K, 2>::createChildNodes(Node& node, std::vector<ID_Element>& elements, int startID, int endID) {

		ID_Element elm0 = elements[0];
		int axis = -1;

		int dIndex = divide(elements, startID, endID, axis);
		if (dIndex != startID && dIndex != endID) {
			// triangles are divided into left and right triangle array

			// create child nodes
			{
				AABB aabb_left, aabb_right;
				for (int i = startID; i < dIndex; ++i) {
					aabb_left.Expand(elements[i].second);
				}
				for (int i = dIndex; i < endID; ++i) {
					aabb_right.Expand(elements[i].second);
				}
				Node node_left(node.node_current, { Node::InvalidID, Node::InvalidID }, m_NumNodes++, Node::InvalidID, aabb_left);
				Node node_right(node.node_current, { Node::InvalidID, Node::InvalidID }, m_NumNodes++, Node::InvalidID, aabb_right);
				node.node_children[0] = node_left.node_current; m_NodeArray[node.node_children[0]] = node_left;
				node.node_children[1] = node_right.node_current; m_NodeArray[node.node_children[1]] = node_right;

				createChildNodes(m_NodeArray[node.node_children[0]], elements, startID, dIndex);
				createChildNodes(m_NodeArray[node.node_children[1]], elements, dIndex, endID);
			}
		}
		else {
			// triangles are not divided (ONLY when the size of triangles is 1)

			// create leaf node
			m_NodeArray[node.node_current].elementID = elm0.first;
			return 0;
		}

		return 0;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 2>::postProcess() {
		return;
	}

	template<typename E, typename K>
	bool aObject::N_BVH<E, K, 2>::divide(std::vector<ID_Element>& elements,
		std::vector<ID_Element>& left, std::vector<ID_Element>& right, int& axis) const {

		if (elements.size() <= 1) {
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
	int aObject::N_BVH<E, K, 2>::divide(std::vector<ID_Element>& elements, int startID, int endID, int& axis) const {

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
	float N_BVH<E, K, 2>::getDivideScore_SAH(int numElm_left, int numElm_right, float area_left, float area_right, float area_whole) const {
		const static float T_aabb = 1.0f;
		const static float T_tri = 2.0f;

		return 2.0f* T_aabb + (area_left * numElm_left + area_right * numElm_right) * (T_tri / area_whole) + 0.001f / (numElm_right * numElm_left + 1.0f);
	}

	template<typename E, typename K>
	int N_BVH<E, K, 2>::getDivideAxis(const std::vector<ID_Element> & elements) const {

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
	int N_BVH<E, K, 2>::getDivideAxis(const std::vector<ID_Element>& elements, int startID, int endID) const {

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
	void N_BVH<E, K, 2>::writeCube(std::ostream & ost, const Primitive::AABB & aabb, unsigned int indexoffset) const {

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
	void N_BVH<E, K, 2>::exportBVH_rec(std::ostream & ost, unsigned int node_id, unsigned int& numVertex, int depth) const {
		const Node& node = m_NodeArray[node_id];

		writeCube(ost, node.aabb, numVertex);
		numVertex += 8;

		for (int i = 0; i < 2; ++i) {
			if (isExistChild(node, i)) {
				exportBVH_rec(ost, node.node_children[i], numVertex, depth + 1);
			}
		}
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 2>::isExistChild(const Node & node, unsigned int childID) const {
		return node.node_children[childID] != Node::InvalidID;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 2>::Build(const std::vector<E> & elements) {
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
		m_NodeArray.resize(elements.size() * 2);
		m_NodeArray[0] = Node(Node::InvalidID, { Node::InvalidID, Node::InvalidID }, 0, iIntersectResult::InvalidID, m_SceneAABB);
		m_NumNodes = 1;

		// Create node tree using recursive function
		createChildNodes(m_NodeArray[0], temp);

		// post process (sort triangle arrays, nodes, and so on)
		postProcess();

		return;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 2>::Build2(const std::vector<E>& elements) {
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
		m_NodeArray.resize(elements.size() * 2);
		m_NodeArray[0] = Node(Node::InvalidID, { Node::InvalidID, Node::InvalidID }, 0, iIntersectResult::InvalidID, m_SceneAABB);
		m_NumNodes = 1;

		// Create node tree using recursive function
		createChildNodes(m_NodeArray[0], 0, elements.size());

		// post process (sort triangle arrays, nodes, and so on)
		postProcess();

		return;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 2>::Delete() {
	}

	template<typename E, typename K>
	void N_BVH<E, K, 2>::ExportBVH(const char* objFileName) const {
		std::ofstream file(objFileName);

		unsigned int vertex = 0;
		exportBVH_rec(file, 0, vertex, 0);
		file.close();
	}

	template<typename E, typename K>
	E N_BVH<E, K, 2>::GetElement(unsigned int elementID) const {
		return m_ElementArray[elementID].second;
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 2>::findClosestElement(IntersectResult_RayTriangle & result, const Primitive::Ray & ray, unsigned int rootNode) const {

		std::deque<unsigned int> stack;
		stack.push_back(rootNode);

		result = IntersectResult_RayTriangle();
		result._t = ray.m_MaxT;

		while (stack.size() > 0) {
			const Node& node = m_NodeArray[stack[stack.size() - 1]];
			stack.pop_back();

			if (node.elementID != iIntersectResult::InvalidID) {
				// leaf node

				// test intersection using "Isc" methods
				Isc::IntersectInfo info;
				if (Isc::Intersect(m_ElementArray[node.elementID].second, ray, info)) {
					if (result._t >= info.t && info.t > ray.m_MinT) {
						result._t = info.t;
						result._u = info.u;
						result._v = info.v;
						result._ElementID = node.elementID;
						result._LeafNodeID = node.node_current;
					}
				}
			}
			else {
				// not leaf node

				// find child nodes which has AABB intersecting ray
				std::vector<std::pair<unsigned int, float>> res(2, std::pair<unsigned int, float>(-1, FLT_MAX));
				for (int i = 0; i < 2; ++i) {
					if (isExistChild(node, i)) {
						Isc::IntersectInfo info;
						if (Isc::Intersect(m_NodeArray[node.node_children[i]].aabb, ray, info)) {
							res[i] = std::pair<unsigned int, float>(i, info.v);
						}
					}
				}

				// sort child nodes by distance from ray origin
				std::sort(res.begin(), res.end(), [](const std::pair<unsigned int, float> & left, const std::pair<unsigned int, float> & right) { return left.second > right.second; });

				// push child nodes accounting for distance from ray origin
				for (int i = 0; i < res.size(); ++i) {
					if (res[i].second < result._t) {
						stack.push_back(node.node_children[res[i].first]);
					}
				}
			}
		}
		return result._t < ray.m_MaxT - 1e-10;
	}


	template<typename E, typename K>
	bool N_BVH<E, K, 2>::ClosestHit(IntersectResult_RayTriangle & result, const Primitive::Ray & key) const {
		return findClosestElement(result, key, 0);
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 2>::findAnyElement(const Primitive::Ray & ray, unsigned int rootNode) const {

		// initialize stack
		std::deque<unsigned int> stack;
		stack.push_back(rootNode);

		int count = 0;

		while (stack.size() > 0) {
			const Node& node = m_NodeArray[*(stack.end() - 1)];
			stack.pop_back();
			count++;

			if (node.elementID != iIntersectResult::InvalidID) {
				// leaf node

				// test intersection with triangle
				Isc::IntersectInfo info;
				if (Isc::Intersect(m_ElementArray[node.elementID].second, ray, info)) {
					if (ray.m_MaxT >= info.t && ray.m_MinT <= info.t) {
						return true;
					}
				}
			}
			else {
				// not leaf node

				std::vector<std::pair<unsigned int, float>> res(2, std::pair<unsigned int, float>(-1, FLT_MAX));
				for (int i = 0; i < 2; ++i) {
					int index = (count + i) & 0x1;

					if (isExistChild(node, index)) {

						Isc::IntersectInfo info;
						if (Isc::Intersect(m_NodeArray[node.node_children[index]].aabb, ray, info)) {
							if (info.v < ray.m_MaxT) {
								stack.push_back(node.node_children[index]);
							}
						}
					}
				}
			}
		}
		return false;
	}

	template <typename E, typename K>
	bool N_BVH<E, K, 2>::ClosestHit(IntersectResult_RayTriangle & result, const Primitive::Ray & ray, unsigned int leafNodeID) const {

		result = IntersectResult_RayTriangle();
		result._t = ray.m_MaxT;

		if (leafNodeID == iIntersectResult::InvalidID) {
			return false;
		}

		unsigned int nodeID = leafNodeID;
		unsigned int fromNodeID = Node::InvalidID;
		while ((nodeID = m_NodeArray[nodeID].node_parent) != Node::InvalidID) {
			const Node& n = m_NodeArray[nodeID];

			for (int i = 0; i < 2; ++i) {
				if (n.node_children[i] != fromNodeID) {
					IntersectResult_RayTriangle res;
					if (findClosestElement(res, Primitive::Ray(ray.m_Org, ray.m_Dir, 1.0f, ray.m_MinT, result._t), n.node_children[i])) {
						result = res;
					}
				}
			}
			fromNodeID = nodeID;
		}
		return result._t < ray.m_MaxT - 1e-10;
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 2>::AnyHit(const Primitive::Ray & ray) const {
		return findAnyElement(ray, 0);
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 2>::AnyHit(const Primitive::Ray& ray, unsigned int leafNodeID) const {
		
		if (leafNodeID == iIntersectResult::InvalidID) {
			return false;
		}

		unsigned int nodeID = leafNodeID;
		unsigned int fromNodeID = leafNodeID;
		while ((nodeID = m_NodeArray[nodeID].node_parent) != Node::InvalidID) {
			const Node& n = m_NodeArray[nodeID];

			for (int i = 0; i < 2; ++i) {
				if (n.node_children[i] != fromNodeID) {
					
					if (findAnyElement(ray, n.node_children[i])) {
						return true;
					}
				}
			}
			fromNodeID = nodeID;
		}
		return false;
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 2>::Intersect(const K & key) const {
		std::deque<unsigned int> stack;
		stack.push_back(0);

		while (stack.size() > 0) {
			const Node& node = m_NodeArray[stack[stack.size() - 1]];
			stack.pop_back();

			if (node.elementID != iIntersectResult::InvalidID) {
				// leaf node

				// test intersection using "Isc" methods
				Isc::IntersectInfo info;
				if (Isc::Intersect(m_ElementArray[node.elementID].second, key, info)) {
					return true;
				}
			}
			else {
				// not leaf node

				// find child nodes which has AABB intersecting ray
				std::vector<std::pair<unsigned int, float>> res(2, std::pair<unsigned int, float>(-1, FLT_MAX));
				for (int i = 0; i < 2; ++i) {
					if (isExistChild(node, i)) {
						Isc::IntersectInfo info;
						if (Isc::Intersect(m_NodeArray[node.node_children[i]].aabb, key, info)) {
							stack.push_back(node.node_children[i]);
						}
					}
				}
			}

		}
		return false;
	}

	template<typename E, typename K>
	N_BVH<E, K, 2>::N_BVH() {
	}

	template<typename E, typename K>
	N_BVH<E, K, 2>::~N_BVH() {
	}




};