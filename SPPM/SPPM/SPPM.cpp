#include "SPPM.h"
#include "TextureUtility.h"
#include "MT.h"
#include "kdtree.h"
#include "Sampling.h"
#include "Intersect.h"
#include "RayUtility.h"
#include "Vec.h"
#include "Matrix.h"
#include "RenderTimer.h"
#include <omp.h>
#include <algorithm>
#include <thread>
#include <fstream>

using namespace PTUtility;
using namespace Primitive;

constexpr float AirIndex = 1.0f;
constexpr float ppm_Alpha = 0.9;
constexpr float FirstRadius = 0.015f;

namespace Render {

	long int StochasticProgressivePhotonMapping::getPassedTime() const {
		return m_CurrentTime - m_RenderStartTime;
	}
	long int StochasticProgressivePhotonMapping::getRemainedTime() const {
		return m_RenderEndTime - m_CurrentTime;
	}
	long int StochasticProgressivePhotonMapping::mesureTime() const {
		m_CurrentTime = RenderTimerMillisecond::GetGlobalTime();
		return m_CurrentTime;
	}

	void StochasticProgressivePhotonMapping::calculateGeometry(const iScene::IntersectionResult& RF, const Scene& scene, Primitive::Ray ray, Geometry& result) const {
		const inoutFacet* Facet = RF._Facet;

		result.Position = RF._u * (Facet->_Pos[1] - Facet->_Pos[0]) + RF._v * (Facet->_Pos[2] - Facet->_Pos[0]) + Facet->_Pos[0];
		result.Normal = (RF._u * (Facet->_Normal[1] - Facet->_Normal[0]) + RF._v * (Facet->_Normal[2] - Facet->_Normal[0]) + Facet->_Normal[0]).normalized();
		result.UV = RF._u * (Facet->_UV[1] - Facet->_UV[0]) + RF._v * (Facet->_UV[2] - Facet->_UV[0]) + Facet->_UV[0];
		result.ShadingNormal = result.Normal;

		short AttributeID = Facet->_AttributeID;
		result.attribute = nullptr;
		if (AttributeID >= 0) {
			result.attribute = scene.GetAttribute(AttributeID);
			result.param = result.attribute->m_SurfaceParam;
			result.Translucency = 0.0f;

			if (result.param.Radiance.norm2() > 1e-6) { result.param.Albedo = Vec3::Ones(); }

			// calculate roughness and albedo from texture
			if (result.attribute->m_RoughnessTextureFileName.size() > 0) {
				result.param.Roughness = std::min(0.8f, std::max(0.05f, 0.5f * TextureUtility::GetColor_UV(*scene.GetTexture(result.attribute->m_RoughnessTextureFileName), result.UV.x(), result.UV.y())[0]));
			}
			if (result.attribute->m_AlbedoTextureFileName.size() > 0) {
				TColor albedo = TextureUtility::GetColor_UV(*scene.GetTexture(result.attribute->m_AlbedoTextureFileName), result.UV.x(), result.UV.y());
				result.param.Albedo = Vec3(albedo[0], albedo[1], albedo[2]);
				result.param.Radiance *= result.param.Albedo;
				result.Translucency = 1.0 - albedo[3];
			}
			if (result.attribute->m_BumpMapTextureFileName.size() > 0) {
				Vec3 normal = TextureUtility::GetNormalFromNormalmap(*scene.GetTexture(result.attribute->m_BumpMapTextureFileName), result.UV.x(), result.UV.y());
				float z = normal[2]; float y = normal[1];
				normal = Vec3(normal[0], z, y).normalized();

				Vec3 e2 = result.Normal;
				Vec3 e1 = std::abs(e2.y()) > 0.9 ? e2.cross(Vec3(1, 0, 0)) : e2.cross(Vec3(0, 1, 0));
				Vec3 e3 = e1.cross(e2).normalized();

				Matrix3 mat = CreateMatrixFromColumns({ e1, e2, e3 }).inv();

				result.Normal = (mat * normal).normalized();
			}
		}
	}

	StochasticProgressivePhotonMapping::HitPoint StochasticProgressivePhotonMapping::makeHitPoint(
		const Primitive::inoutFacet* facet, const Geometry& gmt, const TraceParam& param, int PixelX, int PixelY, const PTUtility::Vec3& DirectLight) const {

		HitPoint hp;
		hp.Facet = *facet;
		hp.Position = gmt.Position;
		hp.OutDir = -param.CurrentRay.m_Dir;
		hp.Pixel_X = PixelX; hp.Pixel_Y = PixelY;
		hp.PixelWeight = param.Througput / param.PDF;
		hp.Type = HitPoint::HIT;
		hp.Radius = FirstRadius;
		hp.ReflectedFlux = Vec3::Ones();
		hp.NumPhoton = 0;
		hp.Gmt = gmt;
		hp.DirectLight = DirectLight;

		return hp;
	}

	StochasticProgressivePhotonMapping::HitPoint StochasticProgressivePhotonMapping::getHitpointInfo(const Primitive::Ray& ray, const Scene& scene, RandomMT& MT, const iCamera& camera, int PixelX, int PixelY) const {
		constexpr int MIN_DEPTH = 4;
		constexpr int MAX_DEPTH = 6;

		// initialize render parameter
		TraceParam pm = { ray, ray, 1.0f, Vec3::Ones(), 0, true, false };

		// Hitpoint
		HitPoint hp; hp.Pixel_X = PixelX; hp.Pixel_Y = PixelY;

		// Result Color
		Vec3 Color(0, 0, 0);

		while (pm.TraceNext) {
			// russian roulette
			if (pm.Depth > MAX_DEPTH) {
				break;
			}
			else if (pm.Depth > MIN_DEPTH) {
				float rus_pdf = std::exp(MIN_DEPTH - pm.Depth);
				if (MT.genrand64_real1() > rus_pdf) {
					break;
				}
				pm.PDF *= rus_pdf;
			}

			iScene::IntersectionResult iscResult;
			if (scene.GetOneFacetFromRay(pm.CurrentRay, iscResult)) {

				// geometry infomation
				Geometry gm;
				calculateGeometry(iscResult, scene, pm.CurrentRay, gm);
				Vec3 originalNormal = gm.Normal;


				// always gm.nomal�ECurrentRay < 0
				gm.Normal = gm.Normal.dot(pm.CurrentRay.m_Dir) > 0 ? -gm.Normal : gm.Normal;
				pm.NextRay.m_Org = gm.Position;

				// if translucency is larger than 0.5, that surface is delt as transparent surface
				if (gm.Translucency > 0.5f) {
					pm.Througput *= gm.Translucency;
					pm.CurrentRay.m_Org = gm.Position + pm.CurrentRay.m_Dir * Ray::RAY_OFFSET;
					continue;
				}

#if 0
				// store direct light
				if (gm.param.Radiance.norm2() > 1e-6) {
					if (!pm.isStoredPhoton) {
						float MIS_weight = 1.0f;

						if (pm.Depth > 0 && !pm.PrevGmt.param.IfDeltaSurface) {
							LightSampler::AreaLight light;
							scene.GetLightManager()->GetAreaLightFromFacetID(light, iscResult._Facet->_FacetID);
							float sampleFromTrianglePDF = scene.GetLightManager()->SampleAreaLight_pdf(*iscResult._Facet, gm.param.Radiance);

							float cosL = std::max(0.0001f, std::min(1.0f, gm.Normal.dot(-pm.CurrentRay.m_Dir)));

							const float pdf_fromNEE =
								(scene.GetLightManager()->GetAreaLightPDF(light) * sampleFromTrianglePDF) *
								(pm.PrevGmt.Position - gm.Position).norm2() / cosL;
							const float pdf_fromPathTrace = pm.pdf_prevDir;

							MIS_weight = std::pow(pdf_fromPathTrace, 2.0f) / (std::pow(pdf_fromNEE, 2.0f) + std::pow(pdf_fromPathTrace, 2.0f));
						}
						hp.DirectLight += MIS_weight * gm.param.Albedo * gm.param.Radiance * pm.Througput / pm.PDF;
					}
					break;
				}
#endif

				// if surface is not delta surface, hitpoint is created. 
				if (gm.param.isSPPMSurface && !pm.isStoredPhoton) {
					hp = makeHitPoint(iscResult._Facet, gm, pm, PixelX, PixelY, Vec3::Zero());
					pm.isStoredPhoton = true;
				}


				float Ii, Io = 1.0f;
				if (near(pm.CurrentRay.m_Index, 1.0f)) {
					Ii = 1.0f; Io = gm.param.Index;
				}
				else {
					Ii = pm.CurrentRay.m_Index; Io = 1.0f;
				}

				Material::FUNCTION_PARAM mparam(pm.CurrentRay.m_Dir, pm.NextRay.m_Dir, gm.Normal, Ii, Io, gm.param.Roughness, gm.param.Extinction, gm.param.Reflectance);

#if 0
				// Estimate direct light
				Vec3 DirectLightRadiance(0, 0, 0);
				float MIS_weight = 1.0f;
				if (!gm.param.IfDeltaSurface) {
					if (!pm.isStoredPhoton) {
						const LightSampler::AreaLight& light = scene.GetLightManager()->SampleAreaLight(MT); float pdf_area = 0.0f;
						float pdf_selectLight = scene.GetLightManager()->GetAreaLightPDF(light);

						Vec3 lpos, uv, Radiance;
						scene.GetLightManager()->SamplePointOnAreaLight(light, MT, lpos, uv, Radiance, pdf_area);

						Vec3 gPos = gm.Position;
						Vec3 lightNormal = (light._Facet._Normal[0] + light._Facet._Normal[1] + light._Facet._Normal[2]).normalized();

						Ray toLightRay(gPos, (lpos - gPos).normalized());
						float toLightDistance = (lpos - gPos).norm();

						if (!scene.IsOccluded(toLightRay, Ray::RAY_OFFSET, toLightDistance - Ray::RAY_OFFSET * 2.0, 0.5f)) {
							// calculate radiance
							// 
							float cosL = std::min(1.0f, std::max(0.00001f, lightNormal.dot(-toLightRay.m_Dir)));
							float cosE = std::abs(gm.Normal.dot(toLightRay.m_Dir));
							mparam.OutDir = toLightRay.m_Dir;

							float BRDF = std::max(0.0f, gm.attribute->m_Surface->BRDF(mparam));

							DirectLightRadiance
								= Radiance
								* cosL * std::pow(toLightDistance, -2.0f) / (pdf_area * pdf_selectLight)
								* cosE * gm.param.Albedo * BRDF;

							// calculate multiple importance sampling weight
							//
							const float pdf_fromNEE = (pdf_area * pdf_selectLight) * std::pow(toLightDistance, 2.0f) / cosL;
							const float pdf_fromPathTrace = gm.attribute->m_Surface->PDF(mparam);

							MIS_weight = std::pow(pdf_fromNEE, 2.0f) / (std::pow(pdf_fromNEE, 2.0f) + std::pow(pdf_fromPathTrace, 2.0f));
							hp.DirectLight += MIS_weight * DirectLightRadiance * pm.Througput / pm.PDF;
						}
					}
				}
#endif 

				// Sample Next direction using BRDF
				float pdf_sampleNextDir = 1.0f;
				pm.NextRay.m_Dir = gm.attribute->m_Surface->SampleNextDir(MT, mparam, &pdf_sampleNextDir);
				if (!RUtil::isValidDirection(pm.NextRay.m_Dir)) {
					break;
				}

				bool isReflect = true;
				if (gm.param.IfOpeque) {
					isReflect = true;

					if (pm.NextRay.m_Dir.dot(gm.Normal) < 1e-6) {
						break;
					}
				}
				else {
					if (pm.NextRay.m_Dir.dot(gm.Normal) < 0.0f) {
						// transmit

						isReflect = false;
					}
					else {
						// reflect

						isReflect = true;
					}

					if (isReflect) {
						pm.NextRay.m_Index = pm.CurrentRay.m_Index;
					}
					else {
						pm.NextRay.m_Index = Io;
					}
				}
				mparam.OutDir = pm.NextRay.m_Dir;

				pdf_sampleNextDir = gm.attribute->m_Surface->PDF(mparam);

				// always OutNormal�ENextRay > 0
				Vec3 OutNormal = gm.Normal.dot(pm.NextRay.m_Dir) > 0 ? gm.Normal : -gm.Normal;
				pm.NextRay.m_Org += OutNormal * Ray::RAY_OFFSET;

				// prepare next iteration //////
				{
					float BRDF = gm.attribute->m_Surface->BRDF(mparam);
					float cos = std::max(0.0f, std::min(1.0f, pm.NextRay.m_Dir.dot(OutNormal)));

					if (BRDF < 1e-8) { break; }
					if (pdf_sampleNextDir < 0) {
						break;
					}
					pm.Througput *= (gm.param.Albedo * cos * BRDF);
				}
				pm.pdf_prevDir = pdf_sampleNextDir;
				pm.PDF *= pdf_sampleNextDir;
				pm.PrevGmt = gm;

				/////////////////////////////////
			}
			else {
				// Sample IBL

				Vec3 IBLColor;
				if (scene.GetLightManager()->GetIBL()) {
					IBLColor = scene.GetLightManager()->GetIBL().GetIBLColorFromDirection(pm.CurrentRay.m_Dir).xyz();
				}
				hp.DirectLight += IBLColor * pm.Througput / pm.PDF;
				if (hp.Type == HitPoint::NONE) { hp.Type = HitPoint::ONLY_DIRECTLIGHT; }

				break;
			}
			pm.CurrentRay = pm.NextRay;
			pm.Depth++;
		}
		return hp;
	}

	void StochasticProgressivePhotonMapping::createHitpoints(
		std::vector<HitPoint> & hitPoints, const Scene & scene, int step, const iCamera & camera, int ScreenWidth, int ScreenHeight) const {
		std::cout << "3. Create HitPoints" << std::endl;

#ifdef _OPENMP
		int maxThreads = omp_get_max_threads();
#else
		int maxThreads = 1;
#endif

		constexpr long int MarginTime = 4 * 1000;

		const int SX = ScreenWidth;
		const int SY = ScreenHeight;

		// buffer for hitpoints
		hitPoints.resize(SX * SY);

		// render image
#pragma omp parallel for schedule(dynamic, 1) num_threads(maxThreads)
		for (int i = 0; i < SX * SY; ++i) {
			if (omp_get_thread_num() == 0) { mesureTime(); }
			if (getRemainedTime() < MarginTime) {
				continue;
			}

			RandomMT MT(i + ScreenWidth * ScreenHeight * step);

			const int w = (i % SX);
			const int h = (i / SX);
			const float u = w / (float)ScreenWidth;
			const float v = h / (float)ScreenHeight;

			const int threadID = omp_get_thread_num();

			float pdf = 1.0f;
			Ray eyeRay = camera.SampleCameraRay(u, v, step, pdf);
			HitPoint hp = getHitpointInfo(eyeRay, scene, MT, camera, w, h);
			hp.PixelWeight = hp.PixelWeight / pdf;
			hp.DirectLight = hp.DirectLight / pdf;

			hitPoints[i] = hp;
		}

		std::cout << "3. Create HitPoints" << std::endl;
	}


	void StochasticProgressivePhotonMapping::initializeSharedPoints(std::vector<SharedHitPoint> & spoints, int ScreenWidth, int ScreenHeight) const {
		spoints.resize(ScreenWidth * ScreenHeight);

		for (int i = 0; i < ScreenWidth * ScreenHeight; ++i) {

			const int w = i % ScreenWidth;
			const int h = i / ScreenWidth;

			SharedHitPoint& sp = spoints[i];
			sp.Pixel_X = w; sp.Pixel_Y = h;
			sp.Radiance = Vec3::Zero();
			sp.DirectLight = Vec3::Zero();
			sp.Radius = FirstRadius;
			sp.Valid = true;
			sp.NumPhoton = 0;
		}
		return;
	}

	bool StochasticProgressivePhotonMapping::createPhotonTree(tgraph::kdtree<Photon> & tree, const std::vector<std::deque<Photon>> & Photons) const {
		// create photon tree
		std::vector<Photon> photon; photon.reserve(Photons.size() * 32);
		for (const auto& pdq : Photons) {
			for (const auto& p : pdq) {
				if (p.Valid) {
					photon.push_back(p);
				}
			}
		}
		if (photon.size() == 0) {
			return false;
		}
		else {
			tree.CreateTree(photon);
		}

		return true;
	}

	void StochasticProgressivePhotonMapping::applyHitPointInfo(
		const std::vector<HitPoint> & hp, std::vector<SharedHitPoint> & sp, const tgraph::kdtree<Photon> & photonTree,
		unsigned int ImageWidth, unsigned int ImageHeight) const {

#ifdef _OPENMP
		int maxThreads = omp_get_max_threads();
#else
		int maxThreads = 1;
#endif

#pragma omp parallel for schedule(dynamic, 1) num_threads(maxThreads)
		for (int v = 0; v < hp.size(); ++v) {

			const HitPoint& i = hp[v];
			if (i.Type == HitPoint::NONE) {
				continue;
			}

			SharedHitPoint& s = sp[v];
			
			if (i.DirectLight.norm2() > 1e-8) {
				s.DirectLight += i.DirectLight;
				s.NumDirectLight++;
			}

			if (i.Type == HitPoint::HitPointType::ONLY_DIRECTLIGHT) {
				continue;
			}

			Vec3 bbMin(i.Position - s.Radius * Vec3::Ones());
			Vec3 bbMax(i.Position + s.Radius * Vec3::Ones());

			std::vector<Photon> ps;
			photonTree.GetElementsInsideBox(ps, bbMin, bbMax);

			Vec3 additionalRadiance(0, 0, 0); int newPhotons = 0;
			for (const auto& p : ps) {
				if ((p.Pos() - i.Position).norm2() < s.Radius * s.Radius) {

					const float Ii = p.Index;
					const float Io = near(p.Index, 1.0f) ? i.Gmt.param.Index : 1.0f;
					const Vec3 normal = p.InDir.dot(i.Gmt.Normal) < 0 ? i.Gmt.Normal : -i.Gmt.Normal;

					Material::FUNCTION_PARAM param(p.InDir, i.OutDir, normal, Ii, Io, i.Gmt.param.Roughness, i.Gmt.param.Extinction, i.Gmt.param.Reflectance);

					additionalRadiance += i.PixelWeight * i.Gmt.attribute->m_Surface->BRDF(param) * p.Flux / p.PDF;
					newPhotons++;
				}
			}
			if (newPhotons > 0) {
				s.Radiance = (s.Radiance + additionalRadiance) * (s.NumPhoton + ppm_Alpha * newPhotons) / (float)(s.NumPhoton + newPhotons);
				s.Radius *= std::sqrt((s.NumPhoton + ppm_Alpha * newPhotons) / (float)(s.NumPhoton + newPhotons));
				s.NumPhoton += newPhotons * ppm_Alpha;
				s.NumGather++;
			}
		}
	}

	Primitive::Ray StochasticProgressivePhotonMapping::generateLightRay(RandomMT & MT, const Scene & scene, Vec3 & Radiance, float& pdf_Area, float& pdf_Dir) const {
		const LightSampler* lmanage = scene.GetLightManager();
		const float AreaLightlFlux = lmanage->GetTotalAreaLightFlux();
		const float IBLFlux = lmanage->GetTotalFluxIBL();
		const float TotalFlux = lmanage->GetTotalFlux();


		const LightSampler::AreaLight& light = lmanage->SampleAreaLight(MT);
		Vec3 lightPos, uv;
		scene.GetLightManager()->SamplePointOnAreaLight(light, MT, lightPos, uv, Radiance, pdf_Area);

		pdf_Area *= (scene.GetLightManager()->GetAreaLightPDF(light));
		const Vec3 lightNormal = (light._Facet._Normal[0] + light._Facet._Normal[1] + light._Facet._Normal[2]).normalized();

		Vec3 outDir = Sampling::Cos_Sampling(MT, lightNormal, &pdf_Dir);
		Radiance *= std::min(1.0f, std::max(0.0f, lightNormal.dot(outDir)));

		return Ray(lightPos + lightNormal * Ray::RAY_OFFSET, outDir);
	}

	void StochasticProgressivePhotonMapping::reconstructImageFromSharedPoints(const std::vector<SharedHitPoint> & sp, Texture2D & renderTarget, int NumPhoton, int MaxSample) const {

#ifdef _OPENMP
		int maxThreads = omp_get_max_threads();
#else
		int maxThreads = 1;
#endif

#pragma omp parallel for schedule(dynamic, 1) num_threads(maxThreads)
		for (int i = 0; i < sp.size(); ++i) {
			const SharedHitPoint& s = sp[i];

			Vec3 L = s.DirectLight / (float)(s.NumDirectLight + 0.0001f) + s.Radiance / ((NumPhoton * (s.NumGather + 0.0001f) / (0.0001f + (float)MaxSample)) * PI * s.Radius * s.Radius);

			constexpr float b = 1.2f;
			TColor c = TColor(std::pow(L[0], 1.0f / b), std::pow(L[1], 1.0f / b), std::pow(L[2], 1.0f / b), 1.0f);

			renderTarget.GetRawData()[s.Pixel_X + s.Pixel_Y * renderTarget.GetWidth()] = c;
		}
		return;
	}

	int StochasticProgressivePhotonMapping::createPhotonMap(
		std::vector<std::deque<Photon>> & photonArray, const Scene & scene, const iCamera & camera, int NumPhoton, int Step) const {
		constexpr long int MarginTime = 3 * 1000;

#ifdef _OPENMP
		int maxThreads = omp_get_max_threads();
#else
		int maxThreads = 1;
#endif

		std::vector<int> pbuf(maxThreads, 0);

#pragma omp parallel for schedule(dynamic, 1) num_threads(maxThreads)
		for (int i = 0; i < NumPhoton; ++i) {

			if (omp_get_thread_num() == 0) {
				mesureTime();
				printf("photon %d \r", i);
			}
			if (getRemainedTime() < MarginTime) {
				continue;
			}

			pbuf[omp_get_thread_num()]++;

			RandomMT MT(i + 135 + Step * NumPhoton);

			float pdf_Area, pdf_Dir = 1.0f;
			Vec3 Flux;
			Ray lightOut = generateLightRay(MT, scene, Flux, pdf_Area, pdf_Dir);

			// initialize render parameter
			TraceParam pm = { lightOut, lightOut, pdf_Area * pdf_Dir, Vec3::Ones(), 0, true, false };

			// generate photon on light surface (light surface must be DiffuseSurface which albedo is 1)
			{
				Photon photon;
				photon.Flux = Flux * pm.Througput * PI;
				photon.PDF = pm.PDF;
				photon.InDir = lightOut.m_Dir;
				photon.Position = lightOut.m_Org;
				photon.Valid = true;
				photon.Index = pm.CurrentRay.m_Index;

				photonArray[i][0] = photon;
			}


			// Result Color
			Vec3 Color(0, 0, 0);

			int photonIndex = 0;
			int loopcount = 0;
			while (pm.TraceNext) {
				if (loopcount++ > 32 || photonIndex >= 32 - 1) {
					break;
				}

				iScene::IntersectionResult iscResult;
				if (pm.Througput[0] < 1e-8 || pm.Througput[1] < 1e-8 || pm.Througput[2] < 1e-8) { break; }

				if (scene.GetOneFacetFromRay(pm.CurrentRay, iscResult)) {

					// geometry infomation
					Geometry gm;
					calculateGeometry(iscResult, scene, pm.CurrentRay, gm);
					Vec3 originalNormal = gm.Normal;

					// always gm.nomal�ECurrentRay < 0
					gm.Normal = gm.Normal.dot(pm.CurrentRay.m_Dir) > 0 ? -gm.Normal : gm.Normal;
					pm.NextRay.m_Org = gm.Position;

					if (gm.param.Radiance.norm2() > 0.00001) {
						break;
					}

					// if translucency is larger than 0.5, that surface is delt as transparent surface
					if (gm.Translucency > 0.5f) {
						pm.Througput *= gm.Translucency;
						pm.CurrentRay.m_Org = gm.Position + pm.CurrentRay.m_Dir * Ray::RAY_OFFSET;
						continue;
					}

					// russian rulette 
					float pdf_rus = 1.0f;
					if (!russianRulette_PhotonMapping(MT, pm.Depth, gm.param.Albedo, pdf_rus)) {
						break;
					}
					pm.PDF *= pdf_rus;

					float Ii, Io = 1.0f;
					if (near(pm.CurrentRay.m_Index, 1.0f)) {
						Ii = 1.0f; Io = gm.param.Index;
					}
					else {
						Ii = pm.CurrentRay.m_Index; Io = 1.0f;
					}


					Material::FUNCTION_PARAM mparam(pm.CurrentRay.m_Dir, pm.NextRay.m_Dir, gm.Normal, Ii, Io, gm.param.Roughness, gm.param.Extinction, gm.param.Reflectance);

					// store photon 
					if (gm.param.isSPPMSurface) {
						Photon photon;
						photon.Flux = Flux * pm.Througput * gm.param.Albedo;
						photon.PDF = pm.PDF;
						photon.InDir = pm.CurrentRay.m_Dir;
						photon.Position = gm.Position;
						photon.Valid = true;
						photon.Index = pm.CurrentRay.m_Index;

						photonArray[i][photonIndex++] = photon;
					}

					// Sample Next direction using BRDF
					float pdf_sampleNextDir = 1.0f;
					pm.NextRay.m_Dir = gm.attribute->m_Surface->SampleNextDir(MT, mparam, &pdf_sampleNextDir);
					if (!RUtil::isValidDirection(pm.NextRay.m_Dir)) {
						break;
					}

					bool isReflect = true;
					if (gm.param.IfOpeque) {

						isReflect = true;

						if (pm.NextRay.m_Dir.dot(gm.Normal) < 1e-6) {
							break;
						}
					}
					else {
						isReflect = RUtil::IsReflect(pm.CurrentRay.m_Dir, pm.NextRay.m_Dir, gm.Normal);

						if (isReflect) {
							pm.NextRay.m_Index = pm.CurrentRay.m_Index;
						}
						else {
							pm.NextRay.m_Index = Io;
						}
					}
					mparam.OutDir = pm.NextRay.m_Dir;

					pdf_sampleNextDir = gm.attribute->m_Surface->PDF(mparam);

					// always OutNormal�ENextRay > 0
					Vec3 OutNormal = gm.Normal.dot(pm.NextRay.m_Dir) > 0 ? gm.Normal : -gm.Normal;
					pm.NextRay.m_Org += OutNormal * Ray::RAY_OFFSET;

					// prepare next iteration //////
					{
						float BRDF = gm.attribute->m_Surface->BRDF(mparam);
						float cos = std::max(0.0f, std::min(1.0f, pm.NextRay.m_Dir.dot(OutNormal)));

						if (BRDF < 1e-8) { break; }
						if (pdf_sampleNextDir < 0) {
							break;
						}
						pm.Througput *= (gm.param.Albedo * cos * BRDF);
					}
					pm.PDF *= pdf_sampleNextDir;

					/////////////////////////////////
				}
				else {
					// Sample IBL

					Vec3 IBLColor;
					if (scene.GetLightManager()->GetIBL()) {
						IBLColor = scene.GetLightManager()->GetIBL().GetIBLColorFromDirection(pm.CurrentRay.m_Dir).xyz();
					}
					Color += IBLColor * pm.Througput / pm.PDF;

					pm.TraceNext = false;
				}
				pm.CurrentRay = pm.NextRay;
				pm.Depth++;
			}
			photonArray[i][++photonIndex] = Photon();
		}

		int nump = 0;
		for (const auto& p : pbuf) {
			nump += p;
		}
		return nump;
	}

	void StochasticProgressivePhotonMapping::RenderImage_FixStep(Texture2D & renderTarget, const Scene & scene, const iCamera & camera, int NumPhotons, int Step) {

		// Get vector defining camera
		const Vec3 CameraE1 = camera._e1;
		const Vec3 CameraE2 = camera._e2;

		// initialize render target
		TextureUtility::SetColor(renderTarget, TColor(0, 0, 0, 1));

		// Get rendertarget width and height
		const int ScreenWidth = renderTarget.GetWidth();
		const int ScreenHeight = renderTarget.GetHeight();

		// Get max thread
		const int Numthreads = omp_get_max_threads();

		// Shared point
		std::vector<SharedHitPoint> sharedPoints;
		initializeSharedPoints(sharedPoints, ScreenWidth, ScreenHeight);

		// Render
		for (int i = 0; i < Step; ++i) {

			std::vector<HitPoint> hitPoints;
			createHitpoints(hitPoints, scene, i, camera, renderTarget.GetWidth(), renderTarget.GetHeight());

			std::vector<std::deque<Photon>> photons;
			createPhotonMap(photons, scene, camera, NumPhotons / Step, i);

			tgraph::kdtree<Photon> tree;
			createPhotonTree(tree, photons);

			applyHitPointInfo(hitPoints, sharedPoints, tree, ScreenWidth, ScreenHeight);
		}
		reconstructImageFromSharedPoints(sharedPoints, renderTarget, NumPhotons, Step);
	}

	void StochasticProgressivePhotonMapping::RenderImage_FixRenderTime(Texture2D & renderTarget, const Scene & scene, const iCamera & camera, int NumPhotonsPerStep, int RenderTimeSceond) {

		m_RenderStartTime = RenderTimerMillisecond::GetGlobalTime();
		m_RenderEndTime = m_RenderStartTime + RenderTimeSceond * 1000;
		m_CurrentTime = m_RenderStartTime;

#ifdef _OPENMP
		int maxThreads = omp_get_max_threads();
#else
		int maxThreads = 1;
#endif
		std::cout << "__Render With SPPM _________________________" << std::endl;
		std::cout << "Num Threads:" << maxThreads << std::endl;
		std::cout << "''''''''''''''''''''''''''''''''''''''''''''" << std::endl;

		// Get vector defining camera
		const Vec3 CameraE1 = camera._e1;
		const Vec3 CameraE2 = camera._e2;

		// initialize render target
		TextureUtility::SetColor(renderTarget, TColor(0, 0, 0, 1));

		// Get rendertarget width and height
		const int ScreenWidth = renderTarget.GetWidth();
		const int ScreenHeight = renderTarget.GetHeight();

		// Get max thread
		const int Numthreads = omp_get_max_threads();

		// Shared point
		std::vector<SharedHitPoint> sharedPoints;
		initializeSharedPoints(sharedPoints, ScreenWidth, ScreenHeight);

		// Create Photon buffer
		std::vector<std::deque<Photon>> photons(std::vector<std::deque<Photon>>(NumPhotonsPerStep, std::deque<Photon>(32)));

		// Create Hit point buffer
		std::vector<HitPoint> hitPoints(ScreenWidth * ScreenHeight);

		int numstep = 0;
		float averageTime_CreatePhotonMap = 0.0f;
		float averageTime_Gather = 0.0f;

		int TotalPhoton = 0;
		float averageTime = 0.0f;
		int loopCount = 0;
		int reducePhoton = 1;

		// Render
		while (true) {
			std::cout << "Step    : " << numstep << std::endl;

#pragma omp parallel for schedule(static, 1) num_threads(maxThreads)
			for (int i = 0; i < photons.size(); ++i) {
				photons[i][0] = Photon();
			}

#pragma omp parallel for schedule(static, 1) num_threads(maxThreads)
			for (int i = 0; i < hitPoints.size(); ++i) {
				hitPoints[i] = HitPoint();
			}

			int startTime = RenderTimerMillisecond::GetGlobalTime();

			if (loopCount > 0) {
				mesureTime();

				if (getRemainedTime() < averageTime / 4) {
					break;
				}
				else if (getRemainedTime() < averageTime / 2) {
					reducePhoton = 16;
				}
				else if (getRemainedTime() < averageTime) {
					reducePhoton = 8;
				}
			}

			std::cout << "1. Create Photon Map" << std::endl;
			int Photons = createPhotonMap(photons, scene, camera, NumPhotonsPerStep / reducePhoton, numstep + 1);
			std::cout << "1. Photon: " << Photons << std::endl;

			if (Photons < 1000) {
				break;
			}
			else {
				TotalPhoton += Photons;
				numstep++;

				tgraph::kdtree<Photon> tree;
				std::thread createtree([&tree, &photons]() {
					std::cout << "2. Create Photon Tree" << std::endl;

					std::vector<Photon> photon; photon.reserve(photons.size() * 16);
					for (const auto& pdq : photons) {
						for (int i = 0; i < pdq.size(); ++i) {
							if (pdq[i].Valid) {
								photon.push_back(pdq[i]);
							}
							else {
								break;
							}
						}
					}

					if (photon.size() == 0) {
						return false;
					}
					else {
						tree.CreateTree(photon);
					}

					std::cout << "2. Create Photon Tree" << std::endl;
					return true;
				});

				
				createHitpoints(hitPoints, scene, numstep, camera, renderTarget.GetWidth(), renderTarget.GetHeight());

				createtree.join();

				std::cout << "4. Gather photon" << std::endl;
				applyHitPointInfo(hitPoints, sharedPoints, tree, ScreenWidth, ScreenHeight);
				std::cout << "4. Gather photon" << std::endl;

				mesureTime();
				if (getRemainedTime() < 1000) {
					break;
				}
			}

			if (reducePhoton > 1) { break; }

			int endTime = RenderTimerMillisecond::GetGlobalTime();
			averageTime = ((endTime - startTime) + loopCount * averageTime) / float(loopCount + 1);
			loopCount++;
		}

		reconstructImageFromSharedPoints(sharedPoints, renderTarget, TotalPhoton, numstep);
	}

};