#pragma once
#include "Normal_Facet.h"
#include "Scene.h"
#include "Texture.h"
#include "Camera.h"
#include "MT.h"
#include "kdtree.h"
#include "material.h"
#include <memory>
#include <vector>
#include <deque>

namespace Render {

	class StochasticProgressivePhotonMapping {

		long int m_RenderStartTime;
		long int m_RenderEndTime;
		mutable long int m_CurrentTime;

		long int getPassedTime() const ;
		long int getRemainedTime() const;
		long int mesureTime() const;

	private:

		static bool near(float s, float t) {
			return std::abs(s - t) < 0.00001;
		}
		static float gI(float Ii, float Io) {
			return near(Ii, 1.0f) ? Io : 1.0f;
		}
		static bool russianRulette_PhotonMapping(RandomMT & MT, int depth, const PTUtility::Vec3 & albedo, float& pdf) {

			static constexpr int MIN_DEPTH = 0;
			static constexpr int MAX_DEPTH = 16;

			pdf = 1.0f;

			// russian roulette
			if (depth > MAX_DEPTH) {
				return false;
			}
			else if (depth >= MIN_DEPTH) {
				pdf = 0.98f * (albedo[0] > albedo[1] ? (albedo[0] > albedo[2] ? albedo[0] : albedo[2]) : (albedo[1] > albedo[2] ? albedo[1] : albedo[2]));

				if (MT.genrand64_real1() > pdf) {
					return false;
				}
			}
			return true;
		}

		static constexpr float Alpha = 0.1f;

		struct Geometry {
			PTUtility::Vec3 Position;
			PTUtility::Vec3 Normal;
			PTUtility::Vec3 ShadingNormal;
			PTUtility::Vec3 UV;
			PTUtility::Material::SURFACE_PARAM param;
			const PTUtility::Attribute* attribute;

			float Translucency;

			Geometry() : Position(), Normal(), ShadingNormal(), UV(), param(), attribute(nullptr), Translucency(0.0f) {
			}
		};

		struct TraceParam {
			Primitive::Ray CurrentRay;
			Primitive::Ray NextRay;
			float PDF;
			PTUtility::Vec3 Througput;
			int Depth;
			bool TraceNext;
			bool isStoredPhoton;
			float pdf_prevDir;

			Geometry PrevGmt;
		};

		struct HitPoint {

			enum HitPointType {
				NONE = 0,
				HIT = 1, 
				ONLY_DIRECTLIGHT = 2
			};

			HitPointType Type;
			Primitive::inoutFacet Facet;
			Geometry Gmt;

			PTUtility::Vec3 Position;
			PTUtility::Vec3 OutDir;
			int Pixel_X;
			int Pixel_Y;
			PTUtility::Vec3 PixelWeight;
			float Radius;
			PTUtility::Vec3 ReflectedFlux;
			float NumPhoton;
			PTUtility::Vec3 DirectLight;

			HitPoint() :
				Type(NONE),
				Facet(), Gmt(), Position(), OutDir(0, 1, 0),
				Pixel_X(0), Pixel_Y(0), PixelWeight(1, 1, 1),
				Radius(1), ReflectedFlux(), NumPhoton(0), DirectLight() {};
		};

		struct SharedHitPoint {
			bool Valid;
			int Pixel_X;
			int Pixel_Y;
			PTUtility::Vec3 Radiance;
			PTUtility::Vec3 DirectLight;

			int NumPhoton;
			float Radius;
			int NumGather;
			int NumDirectLight;

			SharedHitPoint() : Valid(false), Pixel_X(0), Pixel_Y(0), Radiance(), NumPhoton(0), Radius(1), NumGather(0), NumDirectLight(0) {
			}
		};

		struct Photon {
			bool Valid;

			PTUtility::Vec3 Position;
			PTUtility::Vec3 Flux;
			PTUtility::Vec3 InDir;

			float Index;

			float PDF;

			PTUtility::Vec3& Pos() { return Position; }
			const PTUtility::Vec3& Pos() const { return Position; }

			Photon() : Valid(false), PDF(1.0f), Index(1.0f) {};
		};

		void exportPhotons(const char* fileName, const std::vector<Photon>& photon) {
			std::ofstream file(fileName);
			for (const auto p : photon) {
				const PTUtility::Vec3 v = p.Pos();
				file << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
			}
			file.close();
		}

		bool createPhotonTree(tgraph::kdtree<Photon>& tree, const std::vector<std::deque<Photon>>& Photons) const;

		HitPoint makeHitPoint(
			const Primitive::inoutFacet * facet, const Geometry & gmt, const TraceParam & param, int PixelX, int PixelY, const PTUtility::Vec3 & DirectLight) const;

		void calculateGeometry(const iScene::IntersectionResult & RF, const Scene & scene, Primitive::Ray ray, Geometry & result) const;

		void createHitpoints(
			std::vector<HitPoint>& hitPoints,
			const Scene& scene, int step, const iCamera& camera,
			int ScreenWidth, int ScreenHeight) const;
		void initializeSharedPoints(std::vector<SharedHitPoint> & spoints, int ScreenWidth, int ScreenHeight) const;

		int createPhotonMap(
			std::vector<std::deque<Photon>> & photonArray,
			const Scene & scene, const iCamera & camera, int NumPhoton, int Step) const;

		void applyHitPointInfo(
			const std::vector<HitPoint> & hp, std::vector<SharedHitPoint> & sp, const tgraph::kdtree<Photon>& photonTree, unsigned int ImageWidth, unsigned int ImageHeight) const;

		void reconstructImageFromSharedPoints(const std::vector<SharedHitPoint> & sp, Texture2D & renderTarget, int NumPhoton, int MaxSample) const;

		Primitive::Ray generateLightRay(RandomMT & MT, const Scene & scene, PTUtility::Vec3 & Radiance, float& pdf_Area, float& pdf_Dir) const;

		HitPoint getHitpointInfo(
			const Primitive::Ray & ray, const Scene & scene, RandomMT & MT, const iCamera & camera,
			int PixelX, int PixelY) const;

	public:

		void RenderImage_FixStep(Texture2D & renderTarget, const Scene & scene, const iCamera & camera, int NumPhotons, int Step);
		void RenderImage_FixRenderTime(Texture2D& renderTarget, const Scene& scene, const iCamera& camera, int NumPhotonsPerStep, int RenderTimeSceond);

		explicit StochasticProgressivePhotonMapping() {}
		~StochasticProgressivePhotonMapping() {}
	};


};
