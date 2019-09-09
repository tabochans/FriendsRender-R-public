#include "Scene.h"
#include "tiny_obj_loader.h"
#include "stb_image.hpp"
#include "stb_image_write.h"
#include "ScopedTimer.h"
#include "Primitive.h"
#include <iostream>
#include <fstream>

using namespace PTUtility;
using namespace Primitive;

bool Scene::CreateScene(const char* OBJ_FileName, const PTUtility::SceneMaterialManager* sceneMaterial, Texture2D* IBLTexture){

	const PTUtility::SceneMaterialManager* smat = sceneMaterial;
	
	SceneMaterialManager defmat;
	if (smat == nullptr) {
		smat = &defmat;
	}

	{
		std::vector<Primitive::inoutFacet> Facets;
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> materials;
		std::string err;
		std::string objpath("meshData//");
		bool ret = tinyobj::LoadObj(shapes, materials, err, OBJ_FileName, objpath.c_str());
		std::vector<LightSampler::AreaLight> AreaLights;

		if (!err.empty()) {
			std::cerr << err << std::endl;
		}
		if (!ret) {
			std::cerr << "Failed to load/parse .obj.\n" << std::endl;
			return false;
		}

		//Load all textures
		for (int m = 0; m < materials.size(); m++) {
			if (materials[m].diffuse_texname.size() > 2) {
				materials[m].diffuse_texname = objpath + materials[m].diffuse_texname;
				m_Attributes.LoadTexture(materials[m].diffuse_texname.c_str());
			}
			if (materials[m].specular_texname.size() > 2) {
				materials[m].specular_texname = objpath + materials[m].specular_texname;
				m_Attributes.LoadTexture(materials[m].specular_texname.c_str());
			}
			if (materials[m].alpha_texname.size() > 2) {
				materials[m].alpha_texname = objpath + materials[m].alpha_texname;
				m_Attributes.LoadTexture(materials[m].alpha_texname.c_str());
			}
			if (materials[m].bump_texname.size() > 2) {
				materials[m].bump_texname = objpath + materials[m].bump_texname;
				m_Attributes.LoadTexture(materials[m].bump_texname.c_str());
			}
		}

		//Create Attributes
		for (int m = 0; m < materials.size(); m++) {
			Attribute* At = new Attribute;
			At->m_SurfaceParam.Index = 1.0f;
			At->m_SurfaceParam.Albedo = Vec3(materials[m].diffuse[0], materials[m].diffuse[1], materials[m].diffuse[2]);
			At->m_SurfaceParam.Radiance = Vec3(materials[m].emission[0], materials[m].emission[1], materials[m].emission[2]);
			smat->SetParamtoAttribute(At, materials[m].name);

			/*
			map_kd : albedo
			map_d : roughness
			map_bump : bump mapping texture
			*/
			if (m_Attributes.FindTexture(materials[m].diffuse_texname)) { At->m_AlbedoTextureFileName = materials[m].diffuse_texname; }
			if (m_Attributes.FindTexture(materials[m].specular_texname)) { materials[m].specular_texname; }
			if (m_Attributes.FindTexture(materials[m].alpha_texname)) { At->m_RoughnessTextureFileName = materials[m].alpha_texname; }
			if (m_Attributes.FindTexture(materials[m].bump_texname)) { At->m_BumpMapTextureFileName = materials[m].bump_texname; }
			m_Attributes.Add_Attribute(At, m);
		}

		std::vector<Primitive::Triangle_ID> m_TriangleData;

		int NumFacets = 0;
		for (int sps = 0; sps < shapes.size(); sps++) {
			NumFacets += shapes[sps].mesh.indices.size() / 3;
		}
		m_Facets.resize(NumFacets);
		m_TriangleData.resize(NumFacets);

		int Fcount = 0;
		for (int sps = 0; sps < shapes.size(); sps++) {
			for (int idx = 0, face = 0; idx + 2 < shapes[sps].mesh.indices.size(); idx += 3) {

				const int mtid = shapes[sps].mesh.material_ids[face++];
				if (materials.size() > 0) {
					std::array<PTUtility::Vec3, 3> pos;
					pos[0] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 0]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 0]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 0]]);
					pos[1] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 1]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 1]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 1]]);
					pos[2] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 2]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 2]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 2]]);

					std::array<PTUtility::Vec3, 3> normal;
					if (shapes[sps].mesh.normals.size() > 0) {
						normal[0] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 0]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 0]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 0]]);
						normal[1] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 1]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 1]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 1]]);
						normal[2] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 2]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 2]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 2]]);
					}
					else {
						Vec3 fNormal = (pos[1] - pos[2]).normalized().cross((pos[2] - pos[0]).normalized());
						normal[0] = fNormal;
						normal[1] = fNormal;
						normal[2] = fNormal;
					}

					std::array<PTUtility::Vec3, 3> UV;
					if (shapes[sps].mesh.texcoords.size() > 0) {
						UV[0] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 0]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 0]], 0);
						UV[1] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 1]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 1]], 0);
						UV[2] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 2]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 2]], 0);
					}
					else {
						UV[0] = Vec3(0, 0, 0);
						UV[1] = Vec3(0, 0, 0);
						UV[2] = Vec3(0, 0, 0);
					}
					m_TriangleData[Fcount] = Primitive::Triangle_ID(pos, Fcount);
					m_Facets[Fcount] = inoutFacet(pos, normal, UV, mtid, Fcount);

					//Add Light Data to LightManager
					if (mtid >= 0) {
						Vec3 radiance = m_Attributes.GetAttribute(mtid)->m_SurfaceParam.Radiance;
						if (radiance[0] + radiance[1] + radiance[2] > 0.00001f) {
							const float area = m_Facets[Fcount].CalculateArea();

							LightSampler::AreaLight alight;
							LightSampler::CreateAreaLight(alight, m_Facets[Fcount], radiance, GetTexture(m_Attributes.GetAttribute(mtid)->m_AlbedoTextureFileName));
							AreaLights.push_back(alight);
						}
					}
					Fcount++;
				}
				else {
					std::array<PTUtility::Vec3, 3> pos;
					pos[0] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 0]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 0]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 0]]);
					pos[1] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 1]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 1]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 1]]);
					pos[2] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 2]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 2]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 2]]);

					std::array<PTUtility::Vec3, 3> normal;
					if (shapes[sps].mesh.normals.size() > 0) {
						normal[0] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 0]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 0]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 0]]);
						normal[1] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 1]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 1]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 1]]);
						normal[2] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 2]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 2]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 2]]);
					}
					else {
						Vec3 fNormal = (pos[1] - pos[2]).normalized().cross((pos[2] - pos[0]).normalized());
						normal[0] = fNormal;
						normal[1] = fNormal;
						normal[2] = fNormal;
					}

					std::array<PTUtility::Vec3, 3> UV;
					if (shapes[sps].mesh.texcoords.size() > 0) {
						UV[0] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 0]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 0]], 0);
						UV[1] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 1]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 1]], 0);
						UV[2] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 2]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 2]], 0);
					}
					else {

					}
					m_TriangleData[Fcount] = Primitive::Triangle_ID(pos, Fcount);
					m_Facets[Fcount] = inoutFacet(pos, normal, UV, mtid, Fcount);
					Fcount++;
				}
			}
		}

		m_Aobject.reset(new aObject::N_BVH<Primitive::Triangle_ID, Ray, BVH_N>);
		m_Aobject->Build3(m_TriangleData);

		std::vector<LightSampler::OmniLight> omniLights;
		m_LightSampler.CreateLightSampler(AreaLights, omniLights, IBLTexture);
	}

	return true;
}

void Scene::ClearScene(){
	m_Aobject->Delete();
}

int Scene::GetOneFacetFromRay(const Ray& ray, iScene::IntersectionResult& result) const{
	aObject::IntersectResult_RayTriangle iscRayTriangle;
	bool res = m_Aobject->ClosestHit(iscRayTriangle, ray);
	if (res) {
		result._t = iscRayTriangle._t;
		result._u = iscRayTriangle._u;
		result._v = iscRayTriangle._v;
		result._Facet = &m_Facets[m_Aobject->GetElement(iscRayTriangle._ElementID).m_Index];
	}
	else {
		result._t = FLT_MAX;
	}
	return res;
}

bool Scene::IfOccluded(const Ray& ray, float MinDist, float MaxDist) const{
	aObject::IntersectResult_RayTriangle result;
	Ray r(ray);
	r.m_MinT = MinDist; r.m_MaxT = MaxDist;
	return m_Aobject->ClosestHit(result, r);
}

bool Scene::IsOccluded(const Ray& ray, float MinDist, float MaxDist, float translucency) const {
	aObject::IntersectResult_RayTriangle result;
	Ray r(ray);
	r.m_MinT = MinDist; r.m_MaxT = MaxDist;
	if (m_Aobject->ClosestHit(result, r)) {
		const inoutFacet* Facet = &m_Facets[m_Aobject->GetElement(result._ElementID).m_Index];
		Vec3 uv = result._u * (Facet->_UV[1] - Facet->_UV[0]) + result._v * (Facet->_UV[2] - Facet->_UV[0]) + Facet->_UV[0];

		short attrID = Facet->_AttributeID;
		if (attrID >= 0) {
			const Attribute* attr = GetAttribute(attrID);
			if (attr->m_AlbedoTextureFileName.size() > 0) {
				TColor albedo = TextureUtility::GetColor_UV(*GetTexture(attr->m_AlbedoTextureFileName), uv.x(), uv.y());
				float t = 1.0 - albedo[3];

				if (t > translucency) {
					Vec3 pos = ray.m_Org + (result._t + Ray::RAY_OFFSET) * ray.m_Dir;
					float dist = result._t + Ray::RAY_OFFSET;
					Ray newRay(pos, ray.m_Dir, ray.m_Index, std::max(MinDist - dist, (float)Ray::RAY_OFFSET), MaxDist - dist);

					return IsOccluded(newRay, newRay.m_MinT, newRay.m_MaxT, translucency);
				}
				else {
					return true;
				}
			}
			else {
				return true;
			}
		}
		else {
			return true;
		}
	}
	return false;
}