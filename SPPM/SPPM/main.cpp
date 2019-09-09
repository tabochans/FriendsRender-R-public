#include <iostream>
#include "SPPM.h"
#include "Scene.h"
#include "Camera.h"
#include "Texture.h"
#include "TextureUtility.h"
#include "ScopedTimer.h"
std::vector<ScopedTimer::ID_TIME> ScopedTimer::m_Table;

#include "RenderTimer.h"

using namespace PTUtility;

class MainSceneMaterial : public SceneMaterialManager {
	virtual PTUtility::Material::Surface* SetParamtoAttribute_p(Attribute* at, const std::string& MaterialName) const override {

		if (MaterialName == "up_light") {
			at->m_Surface = new PTUtility::Material::MetalSurface();
			at->m_SurfaceParam.IfDeltaSurface = true;
		}
		else if (MaterialName == "lamp_glass") {
			at->m_Surface = new PTUtility::Material::GlossTransmitSurface();
			at->m_SurfaceParam.Albedo = Vec3(0.97f, 0.98f, 0.98f);
			at->m_SurfaceParam.Roughness = 0.1f;
			at->m_SurfaceParam.Reflectance = 0.1f;
			at->m_SurfaceParam.IfOpeque = false;
			at->m_SurfaceParam.isSPPMSurface = false;
		}
		else if (MaterialName == "lamp_body") {
			at->m_Surface = new PTUtility::Material::Roughconductor();
			at->m_SurfaceParam.Roughness = 0.2f;
			at->m_SurfaceParam.Index = 1.1978;
			at->m_SurfaceParam.Extinction = 7.0488;
			at->m_SurfaceParam.IfOpeque = true;
			at->m_SurfaceParam.isSPPMSurface = true;
		}
		else if (MaterialName == "table_paper") {
			at->m_Surface = new PTUtility::Material::MetalSurface();
			at->m_SurfaceParam.isSPPMSurface = false;
			at->m_SurfaceParam.IfDeltaSurface = true;
			at->m_SurfaceParam.Albedo = Vec3::Ones() * 0.999;
		}
		else if (MaterialName == "table2") {
			at->m_Surface = new PTUtility::Material::BiMetalSurface();
			at->m_SurfaceParam.IfOpeque = true;
			at->m_SurfaceParam.isSPPMSurface = false;
			at->m_SurfaceParam.IfDeltaSurface = true;
		}
		else if (MaterialName == "star_outside") {
			at->m_Surface = new PTUtility::Material::GlossTransmitSurface();
			at->m_SurfaceParam.Albedo = Vec3(0.97f, 0.98f, 0.98f);
			at->m_SurfaceParam.Roughness = 0.25f;
			at->m_SurfaceParam.Reflectance = 0.1f;
			at->m_SurfaceParam.IfOpeque = false;
			at->m_SurfaceParam.isSPPMSurface = true;
		}
		else if (MaterialName == "ornament_bell_top") {
			at->m_Surface = new PTUtility::Material::DiffuseTransmitSurface();
			at->m_SurfaceParam.Reflectance = 0.7;
			at->m_SurfaceParam.isSPPMSurface = true;
			at->m_SurfaceParam.IfOpeque = false;
		}
		else if (MaterialName == "tree_leaf") {
			at->m_Surface = new PTUtility::Material::DiffuseTransmitSurface();
			at->m_SurfaceParam.IfOpeque = false;
			at->m_SurfaceParam.isSPPMSurface = true;
			at->m_SurfaceParam.Reflectance = 0.88f;
		}
		else if (MaterialName == "ornament_sphere_body") {
			at->m_Surface = new PTUtility::Material::Roughconductor();
			at->m_SurfaceParam.Roughness = 0.25f;
			at->m_SurfaceParam.Index = 1.1978;
			at->m_SurfaceParam.Extinction = 7.0488;
			at->m_SurfaceParam.IfOpeque = true;
			at->m_SurfaceParam.isSPPMSurface = true;
		}
		else if (MaterialName == "ornament_sphere_body2") {
			at->m_Surface = new PTUtility::Material::GlassSurface();
			at->m_SurfaceParam.Index = 1.4;
			at->m_SurfaceParam.Albedo = Vec3(0.85, 0.92, 0.99);
			at->m_SurfaceParam.IfDeltaSurface = true;
			at->m_SurfaceParam.isSPPMSurface = false;
			at->m_SurfaceParam.IfOpeque = false;
		}
		else if (MaterialName == "spotlight_cone") {
			at->m_Surface = new PTUtility::Material::BiMetalSurface();
			at->m_SurfaceParam.IfOpeque = true;
			at->m_SurfaceParam.IfDeltaSurface = true;
			at->m_SurfaceParam.isSPPMSurface = false;
			at->m_SurfaceParam.Albedo = Vec3::Ones() * 0.9;
		}
		
		else if (MaterialName == "ornament_bell") {
			at->m_Surface = new PTUtility::Material::MetalSurface();
			at->m_SurfaceParam.IfOpeque = true;
			at->m_SurfaceParam.IfDeltaSurface = true;
			at->m_SurfaceParam.isSPPMSurface = false;
		}
		else if (MaterialName == "star_inside") {
			at->m_Surface = new PTUtility::Material::Roughconductor();
			at->m_SurfaceParam.Roughness = 0.12f;
			at->m_SurfaceParam.Index = 1.1978;
			at->m_SurfaceParam.Extinction = 7.0488;
			at->m_SurfaceParam.IfOpeque = true;
			at->m_SurfaceParam.isSPPMSurface = true;
		}
		else {
			at->m_Surface = new PTUtility::Material::DiffuseSurface;
			at->m_SurfaceParam.isSPPMSurface = true;
			at->m_SurfaceParam.IfOpeque = true;
		}
		return at->m_Surface;
	}
};




void RenderScene() {
	constexpr int Resolution = 2;
	constexpr int ScreenWidth = 2860 / Resolution;
	constexpr int ScreenHeight = 4400 / Resolution;

	constexpr unsigned int RenderTimeMilliSecond = 60 * 1000;
	
	unsigned int startTime = RenderTimerMillisecond::GetGlobalTime();

	Texture2D MainRenderTarget;
	MainRenderTarget.Init(ScreenWidth, ScreenHeight);
	TextureUtility::SetColor(MainRenderTarget, TColor(0, 0, 0, 1));

	Texture2D IBL;
	TextureUtility::LoadTextureFromFile(IBL, "image//texture//IBL.jpg");

	MainSceneMaterial Material;

	// Construct Main Scene
	Scene MainScene;
	{
		std::cout << "Load and build main scene		START" << std::endl;
		ScopedTimer _prof("Build Scene");

		MainScene.CreateScene("meshData//MainScene.obj", &Material, &IBL);

		std::cout << "Load and build main scene		END" << std::endl;
	}

	unsigned int endTime = RenderTimerMillisecond::GetGlobalTime();

	// Render
	{
		std::cout << "Render						START" << std::endl;

		ScopedTimer _prof("Render Scene");

		ThinLensCamera camera(ScreenWidth, ScreenHeight, Vec3(2.5f, 1.7, 10.2), Vec3(0.42, 1.5, -0.5f), 12.4, 7.2, 20.05f, ScreenWidth / (float)ScreenHeight, new UniformSample(0) , "image//texture//hole.png");
		
		Render::StochasticProgressivePhotonMapping render;
		render.RenderImage_FixRenderTime(MainRenderTarget, MainScene, camera, 0x180000, (RenderTimeMilliSecond - endTime + startTime) / 1000.0f);

		TextureUtility::ExportPNG(MainRenderTarget, "ResultImage.png");

		std::cout << "Render						END" << std::endl;
	}

	ScopedTimer::PrintTimeTable(std::cout);
}


int main() {
	int startTime = RenderTimerMillisecond::GetGlobalTime();


	std::cout << "Friends Render Returns !" << std::endl;

	RenderScene();

	int endTime = RenderTimerMillisecond::GetGlobalTime();

	std::cout << "Total Time:" << (endTime - startTime) / 1000 << std::endl;

	return 0;
}
