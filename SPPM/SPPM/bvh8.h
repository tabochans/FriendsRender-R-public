namespace aObject {

	using namespace Primitive;
	using namespace PTUtility;

	template<typename E, typename K>
	void N_BVH<E, K, 8>::Build(const std::vector<E>& elements) {
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
		m_NodeArray.resize(elements.size() * 4 + 64);
		m_NodeArray[incNumNode()] = Node();

		Node cNode; cNode.m_parentID = 0;
		for (int c = 0; c < 8; ++c) {
			m_NodeArray[m_NodeArray[0].m_children[c] = incNumNode()] = cNode;
		}

		// Create node tree using recursive function
		createChildNodes(m_NodeArray[0], temp);

		// post process (sort triangle arrays, nodes, and so on)
		postProcess();

		return;
	}

	template<typename E, typename K>
	void N_BVH<E, K, 8>::Build2(const std::vector<E>& elements) {
		if (elements.size() == 0) {
			return;
		}
		m_NumNodes = 0;

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

		m_NodeArray.resize(elements.size() * 4 + 64);

		// Create Root node
		m_NodeArray[incNumNode()] = Node();
		Node cNode; cNode.m_parentID = 0;
		for (int c = 0; c < 8; ++c) {
			m_NodeArray[m_NodeArray[0].m_children[c] = incNumNode()] = cNode;
		}

		// Create node tree using recursive function
		createChildNodes(m_NodeArray[0], m_ElementArray, 0, m_ElementArray.size());
		// post process (sort triangle arrays, nodes, and so on)
		postProcess();

		return;
	}

	template<typename E, typename K>
	std::array<int, 9> N_BVH<E, K, 8>::create8Children(Node& node, std::vector<ID_Element>& elements, int startID, int endID) {
		int dIndex[9]; int axis[7];

		dIndex[0] = startID; dIndex[8] = endID;

		dIndex[4] = divide(elements, startID, endID, axis[0]);

		dIndex[2] = divide(elements, startID, dIndex[4], axis[1]);
		dIndex[6] = divide(elements, dIndex[4], endID, axis[2]);

		for (int i = 0; i < 4; ++i) {
			dIndex[i * 2 + 1] = divide(elements, dIndex[i * 2], dIndex[i * 2 + 2], axis[3 + i]);
		}

		node.setAxis(0, axis[0]); node.setAxis(1, axis[1]); node.setAxis(2, axis[2]); node.setAxis(3, axis[3]);
		node.setAxis(4, axis[4]); node.setAxis(5, axis[5]); node.setAxis(6, axis[6]);

		std::array<AABB, 8> aabbs;
		for (int i = 0; i < 8; ++i) {

			int numElement = dIndex[i + 1] - dIndex[i];

			if (numElement > 1) {
				// more than two triangles exist

				Node& childNode = m_NodeArray[node.m_children[i]];

				for (int j = dIndex[i]; j < dIndex[i + 1]; ++j) {
					aabbs[i].Expand(elements[j].second);
				}

				for (int c = 0; c < 8; ++c) {
					childNode.m_children[c] = incNumNode();
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

		node.m_childAABBs = Primitive::AABB8(aabbs);

		return std::array<int, 9>({dIndex[0],dIndex[1], dIndex[2], dIndex[3], dIndex[4], dIndex[5], dIndex[6], dIndex[7], dIndex[8]});
	}

	template<typename E, typename K>
	void N_BVH<E, K, 8>::Build3(const std::vector<E>& elements) {
		if (elements.size() == 0) {
			return;
		}
		m_NumNodes = 0;

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

		m_NodeArray.resize(m_ElementArray.size() * 4 + 64);


		// create 64 nodes
		std::array<std::array<int, 9>, 8> div = {};
		std::array<int, 9> dIndex;
		m_NodeArray[incNumNode()] = Node();

		for (int c = 0; c < 8; ++c) {
			Node cNode; cNode.m_parentID = 0;
			m_NodeArray[m_NodeArray[0].m_children[c] = incNumNode()] = cNode;
		}

		dIndex = create8Children(m_NodeArray[0], m_ElementArray, 0, m_ElementArray.size());

#pragma omp parallel for schedule(dynamic, 1)
		for (int i = 0; i < 8; ++i) {
			if (dIndex[i + 1] - dIndex[i] > 1) {
				div[i] = create8Children(m_NodeArray[m_NodeArray[0].m_children[i]], m_ElementArray, dIndex[i], dIndex[i + 1]);
			}
			else {
				div[i] = std::array<int, 9>{dIndex[i]};
			}
		}

#pragma omp parallel for schedule(dynamic, 1)
		for (int i = 0; i < 64; ++i) {
			const int p = i / 8;
			const int c = i % 8;

			createChildNodes(m_NodeArray[m_NodeArray[m_NodeArray[0].m_children[p]].m_children[c]], m_ElementArray, div[p][c], div[p][c + 1]);
		}
		
		// post process (sort triangle arrays, nodes, and so on)
		postProcess();

		return;
	}


	template<typename E, typename K>
	int N_BVH<E, K, 8>::createChildNodes(Node& node, std::vector<ID_Element>& elements) {

		std::vector<ID_Element> divide8[14]; int axis[7];
		divide(elements, divide8[12], divide8[13], axis[0]);
		divide(divide8[12], divide8[8], divide8[9], axis[1]);
		divide(divide8[13], divide8[10], divide8[11], axis[2]);

		divide(divide8[8], divide8[0], divide8[1], axis[3]);
		divide(divide8[9], divide8[2], divide8[3], axis[4]);
		divide(divide8[10], divide8[4], divide8[5], axis[5]);
		divide(divide8[11], divide8[6], divide8[7], axis[6]);

		node.setAxis(0, axis[0]); node.setAxis(1, axis[1]); node.setAxis(2, axis[2]); node.setAxis(3, axis[3]); 
		node.setAxis(4, axis[4]); node.setAxis(5, axis[5]); node.setAxis(6, axis[6]); 

		std::array<AABB, 8> aabbs;
		for (int i = 0; i < 8; ++i) {
			if (divide8[i].size() > 1) {
				// more than two triangles exist

				Node& childNode = m_NodeArray[node.m_children[i]];

				for (const auto& t : divide8[i]) {
					aabbs[i].Expand(t.second);
				}

				for (int c = 0; c < 8; ++c) {
					childNode.m_children[c] = incNumNode();
					m_NodeArray[childNode.m_children[c]].m_parentID = node.m_children[c];
				}
			}
			else if (divide8[i].size() == 1) {
				// only one triangle exists
				// create leaf node which points triangle ID instead of child nodes. 

				aabbs[i].Expand(divide8[i][0].second);

				Node& childNode = m_NodeArray[node.m_children[i]];
				childNode.m_children[0] = divide8[i][0].first;

			}
			else {
				// no triangle
				// insert dummy triangle

				aabbs[i].Reset();
			}
		}

		node.m_childAABBs = Primitive::AABB8(aabbs);
		for (int i = 0; i < 8; ++i) {
			if (divide8[i].size() > 1) {
				createChildNodes(m_NodeArray[node.m_children[i]], divide8[i]);
			}
		}

		return 0;
	}


	template<typename E, typename K>
	int aObject::N_BVH<E, K, 8>::createChildNodes(Node& node, std::vector<ID_Element>& elements, int startID, int endID, int NumDepth) {

		int dIndex[9]; int axis[7];

		dIndex[0] = startID; dIndex[8] = endID;

		dIndex[4] = divide(elements, startID, endID, axis[0]);
		
		dIndex[2] = divide(elements, startID, dIndex[4], axis[1]);
		dIndex[6] = divide(elements, dIndex[4], endID, axis[2]);
		
		dIndex[1] = divide(elements, dIndex[0], dIndex[2], axis[3]);
		dIndex[3] = divide(elements, dIndex[2], dIndex[4], axis[4]);
		dIndex[5] = divide(elements, dIndex[4], dIndex[6], axis[5]);
		dIndex[7] = divide(elements, dIndex[6], dIndex[8], axis[6]);

		node.setAxis(0, axis[0]); node.setAxis(1, axis[1]); node.setAxis(2, axis[2]); node.setAxis(3, axis[3]);
		node.setAxis(4, axis[4]); node.setAxis(5, axis[5]); node.setAxis(6, axis[6]);

		std::array<AABB, 8> aabbs;
		for (int i = 0; i < 8; ++i) {

			int numElement = dIndex[i + 1] - dIndex[i];

			if (numElement > 1) {
				// more than two triangles exist

				Node& childNode = m_NodeArray[node.m_children[i]];

				for (int j = dIndex[i]; j < dIndex[i + 1]; ++j) {
					aabbs[i].Expand(elements[j].second);
				}

				for (int c = 0; c < 8; ++c) {
					childNode.m_children[c] = incNumNode();
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

		node.m_childAABBs = Primitive::AABB8(aabbs);
		for (int i = 0; i < 8; ++i) {
			if (dIndex[i + 1] - dIndex[i] > 1) {
				createChildNodes(m_NodeArray[node.m_children[i]], elements, dIndex[i], dIndex[i + 1], NumDepth + 1);
			}
		}

		return 0;
	}

	template<typename E, typename K>
	bool aObject::N_BVH<E, K, 8>::divide(std::vector<ID_Element> & elements,
		std::vector<ID_Element> & left, std::vector<ID_Element> & right, int& axis) const {

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
	int N_BVH<E, K, 8>::divide(
		std::vector<ID_Element>& elements,
		int startID, int endID, int& axis) const {

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
	int N_BVH<E, K, 8>::getDivideAxis(const std::vector<ID_Element> & elements) const {

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
	int N_BVH<E, K, 8>::getDivideAxis(const std::vector<ID_Element>& elements, int startID, int endID) const {

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
	void N_BVH<E, K, 8>::postProcess() {
		m_NodeArray.resize(m_NumNodes + 10);
		return;
	}

	template<typename E, typename K>
	float N_BVH<E, K, 8>::getDivideScore_SAH(int numElm_left, int numElm_right, float area_left, float area_right, float area_whole) const {
		const static float T_aabb = 1.0f;
		const static float T_tri = 2.0f;
		return 2.0f* T_aabb + (area_left * numElm_left + area_right * numElm_right) * (T_tri / area_whole) + 0.001f / (numElm_right * numElm_left + 1.0f);
	}

	template<typename E, typename K>
	void N_BVH<E, K, 8>::writeCube(std::ostream & ost, const Primitive::AABB & aabb, unsigned int indexoffset) const {

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
	void N_BVH<E, K, 8>::exportBVH_rec(std::ostream & ost, unsigned int node_id, unsigned int& numVertex, int depth) const {

		const Node& node = m_NodeArray[node_id];

		if (node.isLeaf()) {
			// current node is leaf node
		}
		else {
			// current node is not leaf node

			for (int c = 0; c < 8; ++c) {

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
	bool N_BVH<E, K, 8>::isExistChild(const Node & node, unsigned int childID) const {
		return false;
	}
	template<typename E, typename K>
	void N_BVH<E, K, 8>::Delete() {
	}

	template<typename E, typename K>
	void N_BVH<E, K, 8>::ExportBVH(const char* objFileName) const {
		std::ofstream file(objFileName);

		unsigned int vertex = 0;
		exportBVH_rec(file, 0, vertex, 0);
		file.close();
	}

	template<typename E, typename K>
	E N_BVH<E, K, 8>::GetElement(unsigned int elementID) const {
		return m_ElementArray[elementID].second;
	}



	template<typename E, typename K>
	bool N_BVH<E, K, 8>::findClosestElement(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int rootNode) const {
		std::deque<unsigned int> stack;
		stack.push_back(rootNode);

		result = IntersectResult_RayTriangle();
		result._t = ray.m_MaxT;

		int NumBox = 0;
		while (stack.size() > 0 && NumBox < m_NumNodes) {
			const unsigned int nodeID = *(stack.end() - 1);
			const Node & node = m_NodeArray[nodeID];
			stack.pop_back();
			NumBox++;

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
				constexpr std::pair<unsigned int, float> tp(-1, FLT_MAX);
				std::array<std::pair<unsigned int, float>, 8> res({ tp,tp,tp,tp,tp,tp,tp,tp });
				std::array<float, 8> hitDistance;
				if (!Isc::Intersect_AABB8_RAY(node.m_childAABBs, ray, hitDistance)) {
					continue;
				}
				for (int i = 0; i < 8; ++i) {
					res[i] = std::pair<unsigned int, float>(i, hitDistance[i]);
				}

				// sort by node.m_axis
				const bool of = ray.m_Dir[node.getAxis(0)] > 0;
				const bool of0 = ray.m_Dir[node.getAxis(1)] > 0;
				const bool of1 = ray.m_Dir[node.getAxis(2)] > 0;
				const bool of00 = ray.m_Dir[node.getAxis(3)] > 0;
				const bool of01 = ray.m_Dir[node.getAxis(4)] > 0;
				const bool of10 = ray.m_Dir[node.getAxis(5)] > 0;
				const bool of11 = ray.m_Dir[node.getAxis(6)] > 0;
				res[of * 4 + of0 * 2 + of00] = std::pair<unsigned int, float>(0, hitDistance[0]);
				res[of * 4 + of0 * 2 + !of00] = std::pair<unsigned int, float>(1, hitDistance[1]);
				res[of * 4 + !of0 * 2 + of00] = std::pair<unsigned int, float>(2, hitDistance[2]);
				res[of * 4 + !of0 * 2 + !of00] = std::pair<unsigned int, float>(3, hitDistance[3]);
				res[!of * 4 + of0 * 2 + of00] = std::pair<unsigned int, float>(4, hitDistance[4]);
				res[!of * 4 + of0 * 2 + !of00] = std::pair<unsigned int, float>(5, hitDistance[5]);
				res[!of * 4 + !of0 * 2 + of00] = std::pair<unsigned int, float>(6, hitDistance[6]);
				res[!of * 4 + !of0 * 2 + !of00] = std::pair<unsigned int, float>(7, hitDistance[7]);

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
	bool N_BVH<E, K, 8>::findAnyElement(const Primitive::Ray& ray, unsigned int rootNode) const {
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
				constexpr std::pair<unsigned int, float> tp(-1, FLT_MAX);
				std::array<std::pair<unsigned int, float>, 8> res({ tp,tp,tp,tp,tp,tp,tp,tp });
				std::array<float, 8> hitDistance;
				if (!Isc::Intersect_AABB8_RAY(node.m_childAABBs, ray, hitDistance)) {
					continue;
				}
				for (int i = 0; i < 8; ++i) {
					res[i] = std::pair<unsigned int, float>(i, hitDistance[i]);
				}

				// sort by node.m_axis
				const bool of = ray.m_Dir[node.getAxis(0)] > 0;
				const bool of0 = ray.m_Dir[node.getAxis(1)] > 0;
				const bool of1 = ray.m_Dir[node.getAxis(2)] > 0;
				const bool of00 = ray.m_Dir[node.getAxis(3)] > 0;
				const bool of01 = ray.m_Dir[node.getAxis(4)] > 0;
				const bool of10 = ray.m_Dir[node.getAxis(5)] > 0;
				const bool of11 = ray.m_Dir[node.getAxis(6)] > 0;
				res[of * 4 + of0 * 2 + of00] = std::pair<unsigned int, float>(0, hitDistance[0]);
				res[of * 4 + of0 * 2 + !of00] = std::pair<unsigned int, float>(1, hitDistance[1]);
				res[of * 4 + !of0 * 2 + of00] = std::pair<unsigned int, float>(2, hitDistance[2]);
				res[of * 4 + !of0 * 2 + !of00] = std::pair<unsigned int, float>(3, hitDistance[3]);
				res[!of * 4 + of0 * 2 + of00] = std::pair<unsigned int, float>(4, hitDistance[4]);
				res[!of * 4 + of0 * 2 + !of00] = std::pair<unsigned int, float>(5, hitDistance[5]);
				res[!of * 4 + !of0 * 2 + of00] = std::pair<unsigned int, float>(6, hitDistance[6]);
				res[!of * 4 + !of0 * 2 + !of00] = std::pair<unsigned int, float>(7, hitDistance[7]);

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
	bool N_BVH<E, K, 8>::ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray) const {
		return findClosestElement(result, ray, 0);
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 8>::ClosestHit(IntersectResult_RayTriangle& result, const Primitive::Ray& ray, unsigned int leafNodeID) const {

		unsigned int nodeID = leafNodeID;
		unsigned int fromID = Node::InvalidID;

		result = IntersectResult_RayTriangle();
		result._t = ray.m_MaxT;

		while ((nodeID = m_NodeArray[nodeID].m_parentID) != Node::InvalidID) {
			const Node& n = m_NodeArray[nodeID];

			// create object for sorting
			constexpr std::pair<unsigned int, float> tp(-1, FLT_MAX);
			std::array<std::pair<unsigned int, float>, 8> res({ tp,tp,tp,tp,tp,tp,tp,tp });
			std::array<float, 8> hitDistance;
			if (Isc::Intersect_AABB8_RAY(n.m_childAABBs, ray, hitDistance)) {
				for (int i = 0; i < 8; ++i) {
					res[i] = std::pair<unsigned int, float>(i, hitDistance[i]);
				}

				// sort by node.m_axis
				const bool of = ray.m_Dir[n.getAxis(0)] > 0;
				const bool of0 = ray.m_Dir[n.getAxis(1)] > 0;
				const bool of1 = ray.m_Dir[n.getAxis(2)] > 0;
				const bool of00 = ray.m_Dir[n.getAxis(3)] > 0;
				const bool of01 = ray.m_Dir[n.getAxis(4)] > 0;
				const bool of10 = ray.m_Dir[n.getAxis(5)] > 0;
				const bool of11 = ray.m_Dir[n.getAxis(6)] > 0;
				res[of * 4 + of0 * 2 + of00] = std::pair<unsigned int, float>(0, hitDistance[0]);
				res[of * 4 + of0 * 2 + !of00] = std::pair<unsigned int, float>(1, hitDistance[1]);
				res[of * 4 + !of0 * 2 + of00] = std::pair<unsigned int, float>(2, hitDistance[2]);
				res[of * 4 + !of0 * 2 + !of00] = std::pair<unsigned int, float>(3, hitDistance[3]);
				res[!of * 4 + of0 * 2 + of00] = std::pair<unsigned int, float>(4, hitDistance[4]);
				res[!of * 4 + of0 * 2 + !of00] = std::pair<unsigned int, float>(5, hitDistance[5]);
				res[!of * 4 + !of0 * 2 + of00] = std::pair<unsigned int, float>(6, hitDistance[6]);
				res[!of * 4 + !of0 * 2 + !of00] = std::pair<unsigned int, float>(7, hitDistance[7]);

				for (int i = 7; i >= 0; --i) {
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
	bool N_BVH<E, K, 8>::AnyHit(const Primitive::Ray& ray) const {
		return findAnyElement(ray, 0);
	}

	template<typename E, typename K>
	bool N_BVH<E, K, 8>::AnyHit(const Primitive::Ray& ray, unsigned int leafNodeID) const {
		unsigned int nodeID = leafNodeID;
		unsigned int fromID = Node::InvalidID;

		while ((nodeID = m_NodeArray[nodeID].m_parentID) != Node::InvalidID) {
			const Node& n = m_NodeArray[nodeID];

			std::array<float, 8> hitDistance;
			if (Isc::Intersect_AABB8_RAY(n.m_childAABBs, ray, hitDistance)) {
				for (int i = 0; i < 8; ++i) {
					if (hitDistance[i] < ray.m_MaxT && n.m_children[i] != fromID) {
						if (findAnyElement(ray, n.m_children[i])) {
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
	N_BVH<E, K, 8>::N_BVH() {
	}

	template<typename E, typename K>
	N_BVH<E, K, 8>::~N_BVH() {
	}
};