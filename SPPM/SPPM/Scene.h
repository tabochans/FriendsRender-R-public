#pragma once
#include "Texture.h"
#include "LightSampler.h"
#include "Vec.h"
#include "nbvh.h"
#include "Intersect.h"
#include "Sampling.h"
#include "material.h"
#include "SceneMaterial.h"
#include "Normal_Facet.h"
#include <utility>
#include <vector>
#include <map>

class iScene{
private:

public:

	struct IntersectionResult {
		float _u, _v, _t;
		const Primitive::inoutFacet* _Facet;

		bool operator<(const IntersectionResult& ref) const{
			return this->_t < ref._t;
		}
		bool operator>(const IntersectionResult& ref) const{
			return this->_t > ref._t;
		}
		bool operator<=(const IntersectionResult& ref) const{
			return this->_t <= ref._t;
		}
		bool operator>=(const IntersectionResult& ref) const{
			return this->_t >= ref._t;
		}
		IntersectionResult& operator=(const IntersectionResult& ref) {
			_Facet = ref._Facet;
			_u = ref._u;
			_v = ref._v;
			_t = ref._t;
			return *this;
		}
	};

	virtual bool CreateScene(const char* OBJ_FileName) = 0;
	virtual void ClearScene() = 0;
	virtual int GetOneFacetFromRay(const Primitive::Ray& ray, aObject::IntersectResult_RayTriangle& result)const = 0;
	virtual PTUtility::Vec3 GetSceneWidth()const = 0;
	virtual PTUtility::Vec3 GetSceneCenter()const = 0;

	iScene(){ ; }
	virtual ~iScene(){ ; }
};

class Scene{

	static constexpr int BVH_N = 8;

private:
	PTUtility::AttributeManager m_Attributes;
	std::unique_ptr<aObject::N_BVH<Primitive::Triangle_ID, Primitive::Ray, BVH_N>> m_Aobject;
	std::vector<Primitive::inoutFacet> m_Facets;
	LightSampler m_LightSampler;
	void AddExtraLights(std::vector<LightSampler::OmniLight>& omniLights);

public:

	const LightSampler* GetLightManager()const{ return (&m_LightSampler) ; }

	virtual bool CreateScene(const char* OBJ_FileName, const PTUtility::SceneMaterialManager* sceneMaterial, Texture2D* IBLTexture);
	virtual void ClearScene();
	virtual int GetOneFacetFromRay(const Primitive::Ray& ray, iScene::IntersectionResult& result)const;
	virtual bool IfOccluded(const Primitive::Ray& ray, float MinDist, float MaxDist)const;
	virtual bool IsOccluded(const Primitive::Ray& ray, float MinDist, float MaxDist, float translucency) const;
	virtual PTUtility::Vec3 GetSceneWidth()const{
		return m_Aobject->GetSceneWidth();
	}
	virtual PTUtility::Vec3 GetSceneCenter()const{
		return m_Aobject->GetSceneCenter();
	}
	const PTUtility::Attribute* GetAttribute(short ID)const{
		return m_Attributes.GetAttribute(ID);
	}
	const Texture2D* GetTexture(const std::string& TextureName)const{
		if (TextureName.size() < 2){
			return nullptr;
		}
		return m_Attributes.GetTexture(TextureName);
	}

	Scene(){}
	virtual ~Scene(){}
};
