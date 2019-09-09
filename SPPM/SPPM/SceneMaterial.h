#pragma once
#include"material.h"
#include"Normal_Facet.h"
#include<string>
#include<vector>
#include<fstream>

namespace PTUtility{
	class SceneMaterialManager{
	private:
		mutable std::vector<std::unique_ptr<PTUtility::Material::Surface>> m_Materials;

		virtual PTUtility::Material::Surface* SetParamtoAttribute_p(Attribute* at, const std::string& MaterialName) const{
			at->m_Surface = new PTUtility::Material::DiffuseSurface;
			return at->m_Surface;
		}

		void AddMaterialPtr(PTUtility::Material::Surface* ptr) const{
			if (ptr) {
				m_Materials.push_back(std::unique_ptr<PTUtility::Material::Surface>(ptr));
			}
		}

	public:
		void SetParamtoAttribute(Attribute* at, const std::string& MaterialName) const{
			PTUtility::Material::Surface* pp = SetParamtoAttribute_p(at, MaterialName);
			AddMaterialPtr(pp);
		}
		virtual void LoadMaterialTable(const char* FileName){
		}
		virtual ~SceneMaterialManager() {
		}
	};
}