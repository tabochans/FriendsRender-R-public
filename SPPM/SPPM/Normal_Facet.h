#pragma once
#include"Vec.h"
#include"Intersect.h"
#include"Texture.h"
#include"MT.h"
#include"material.h"
#include<array>
#include<vector>
#include<map>
#include<string>
#include<memory>
#include<functional>

namespace Primitive{

	struct inoutFacet{

		std::array<PTUtility::Vec3, 3> _Pos;
		std::array<PTUtility::Vec3, 3> _Normal;
		std::array<PTUtility::Vec3, 3> _UV;

		PTUtility::Vec3 _Vec_1_0;
		PTUtility::Vec3 _Vec_2_0;

		PTUtility::Vec3 _Center;
		short _AttributeID;

		int _FacetID;

		inoutFacet(const std::array<PTUtility::Vec3, 3>& Pos, const std::array<PTUtility::Vec3, 3>& Normal, const std::array<PTUtility::Vec3, 3>& UV, short ID, int FacetID)
			: _Pos(Pos), _Normal(Normal), _UV(UV), _AttributeID(ID), _Center(0.33333333 * (Pos[0] + Pos[1] + Pos[2])), _FacetID(FacetID){
			_Vec_1_0 = _Pos[1] - _Pos[0];
			_Vec_2_0 = _Pos[2] - _Pos[0];
		}
		inoutFacet()
			: _Pos(std::array<PTUtility::Vec3, 3>()), _Normal(std::array<PTUtility::Vec3, 3>()), _UV(std::array<PTUtility::Vec3, 3>()), _AttributeID(-1234), _Center(), _FacetID(-1){
		}

		static bool compPX(const inoutFacet* left, const inoutFacet* right){
			return left->_Center.x() < right->_Center.x();
		}
		static bool compPY(const inoutFacet* left, const inoutFacet* right){
			return left->_Center.y() < right->_Center.y();
		}
		static bool compPZ(const inoutFacet* left, const inoutFacet* right){
			return left->_Center.z() < right->_Center.z();
		}

		static inoutFacet CreateDummyFacet(){
			std::array<PTUtility::Vec3, 3> Pos = { PTUtility::Vec3(10000.00001, 10000, 10000), PTUtility::Vec3(10000.0, 10000.00001, 10000), PTUtility::Vec3(10000.0, 10000, 10000.00001) };
			std::array<PTUtility::Vec3, 3> Normal = { PTUtility::Vec3(1, 0, 0), PTUtility::Vec3(1, 0, 0), PTUtility::Vec3(1, 0, 0) };
			std::array<PTUtility::Vec3, 3> UV = { PTUtility::Vec3::Zero(), PTUtility::Vec3::Zero(), PTUtility::Vec3::Zero() };
			return inoutFacet(Pos, Normal, UV, -1234, -1);
		}

		float CalculateArea()const{
			return 0.5f * ((_Vec_1_0).cross(_Vec_2_0).norm());
		}
	};

}

namespace PTUtility{

	class Vec3;

	class Attribute{

	private:
		Attribute(const Attribute& ref);
		Attribute& operator=(const Attribute& ref);
	public:

		bool operator<(const Attribute& ref){
			return m_SurfaceParam.Albedo.norm2() * m_SurfaceParam.Index < ref.m_SurfaceParam.Albedo.norm2() * ref.m_SurfaceParam.Index;
		}
		bool operator>(const Attribute& ref){
			return m_SurfaceParam.Albedo.norm2() * m_SurfaceParam.Index > ref.m_SurfaceParam.Albedo.norm2() * ref.m_SurfaceParam.Index;
		}
		bool operator<=(const Attribute& ref){
			return m_SurfaceParam.Albedo.norm2() * m_SurfaceParam.Index <= ref.m_SurfaceParam.Albedo.norm2() * ref.m_SurfaceParam.Index;
		}
		bool operator>=(const Attribute& ref){
			return m_SurfaceParam.Albedo.norm2() * m_SurfaceParam.Index >= ref.m_SurfaceParam.Albedo.norm2() * ref.m_SurfaceParam.Index;
		}
		bool operator==(const Attribute& ref){
			return m_SurfaceParam.Albedo.norm2() * m_SurfaceParam.Index == ref.m_SurfaceParam.Albedo.norm2() * ref.m_SurfaceParam.Index;
		}

		PTUtility::Material::Surface* m_Surface;
		PTUtility::Material::SURFACE_PARAM m_SurfaceParam;
		PTUtility::Material::Medium* m_Medium;

		std::string m_AlbedoTextureFileName;
		std::string m_RoughnessTextureFileName;
		std::string m_ParamTextureFileName;
		std::string m_BumpMapTextureFileName;

		std::string _AttributeName;

		Attribute() : m_Surface(nullptr), m_Medium(nullptr), _AttributeName("Default"){
			delete m_Surface;
			delete m_Medium;
		}
	};

	class AttributeManager{
	private:
		std::map<std::string, std::unique_ptr<Texture2D>> m_TextureMap;
		std::map<short, std::unique_ptr<Attribute>>m_AttributeTable;
	public:

		bool LoadTexture(const char* TextureFileName){
			if (std::string(TextureFileName).size() < 2){
				return false;
			}
			if (m_TextureMap.find(TextureFileName) == m_TextureMap.end()){
				Texture2D* tex = new Texture2D();
				bool load = TextureUtility::LoadTextureFromFile(*tex, TextureFileName);
				if (load){
					m_TextureMap[std::string(TextureFileName)] = std::unique_ptr<Texture2D>(tex);
					return true;
				}
				else{
					return false;
				}
			}
			else{
				return true;
			}
		}

		void Add_Attribute(Attribute* attribute, short AttributeID){
			m_AttributeTable[AttributeID] = std::unique_ptr<Attribute>(attribute);
		}
		const Attribute* GetAttribute(short AttributeID)const{
			return m_AttributeTable.at(AttributeID).get();
		}
		const Texture2D* GetTexture(const std::string& TextureName)const{
			if (m_TextureMap.count(TextureName) > 0) {
				return m_TextureMap.at(TextureName).get();
			}
			else {
				return nullptr;
			}
		}
		bool FindTexture(const std::string& TextureName)const{
			return (m_TextureMap.find(TextureName) != m_TextureMap.end());
		}
	};

}