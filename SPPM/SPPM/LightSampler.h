#pragma once

#include "Vec.h"
#include "Intersect.h"
#include "IBL.h"
#include "Normal_Facet.h"
#include "Texture.h"
#include "TableSampler.h"
#include "MT.h"
#include "Sampling.h"
#include <vector>
#include <functional>

class Scene;

class LightSampler {
	static constexpr int NumSample = 1000;
public:
	struct AreaLight {
	private:

	public:

		struct UV_L {
			PTUtility::Vec3 UV;
			PTUtility::Vec3 L;
		};
		TableSampler<UV_L> m_Table;

		Primitive::inoutFacet _Facet;
		PTUtility::Vec3 _Flux;
		bool _Textured;

		static float Evaluate(const AreaLight& t);

		AreaLight(
			const Primitive::inoutFacet& facet, const PTUtility::Vec3& Flux, bool Textured = false) : _Facet(facet), _Flux(Flux), _Textured(Textured) {
		}
		AreaLight() : _Facet(Primitive::inoutFacet::CreateDummyFacet()), _Flux(0, 0, 0), _Textured(false) {}
		~AreaLight() {}
	};

	struct OmniLight {
	private:

	public:
		PTUtility::Vec3 _Position;
		PTUtility::Vec3 _Flux;

		OmniLight(const PTUtility::Vec3& Position, const PTUtility::Vec3& Flux) : _Position(Position), _Flux(Flux) {
		}
		OmniLight() : _Position(0, 0, 0), _Flux(0, 0, 0) {}

		~OmniLight() {}
		static float Evaluate(const OmniLight& t);
	};

private:

	float m_TotalFlux_AreaLight;
	float m_TotalFlux_OmniLight;
	float m_TotalFlux_IBL;

	// Light Data
	TableSampler<AreaLight> m_AreaLights; float m_TotalArea;
	TableSampler<OmniLight> m_OmniLights;
	IBL m_IBL;

public:

	bool CreateLightSampler(
		const std::vector<AreaLight>& AreaLights, 
		const std::vector<OmniLight>& OmniLights, 
		const Texture2D* IBL);
	float GetTotalFlux() const { return m_TotalFlux_AreaLight + m_TotalFlux_OmniLight + m_TotalFlux_IBL; }

	void SamplePointOnAreaLight(const AreaLight& arealight, RandomMT& MT, PTUtility::Vec3& Pos, PTUtility::Vec3& UV, PTUtility::Vec3& Radiance, float& pdf) const;
	float SampleAreaLight_pdf(const Primitive::inoutFacet& facet, const PTUtility::Vec3& radiance) const;
	bool GetAreaLightFromFacetID(AreaLight& light, int FacetID) const;
	
	static void CreateAreaLight(
		AreaLight& alight,
		const Primitive::inoutFacet& facet, const PTUtility::Vec3& radiance, const Texture2D* texture);
	const AreaLight* GetAreaLightsData() const { return m_AreaLights.GetRawData(); }
	int GetNumAreaLights() const { return m_AreaLights.GetNumElements(); }
	const AreaLight& SampleAreaLight(RandomMT& MT) const { return m_AreaLights.Sample(MT); }
	const float GetAreaLightPDF(const AreaLight& light) const { return m_AreaLights.GetPDF(light); }
	float GetTotalAreaLightFlux() const { return m_TotalFlux_AreaLight; }

	const OmniLight* GetOmniLightsData() const { return m_OmniLights.GetRawData(); }
	int GetNumOmniLights() const { return m_OmniLights.GetNumElements(); }
	const OmniLight& SampleOmniLight(RandomMT& MT) const { return m_OmniLights.Sample(MT); }
	const float GetOmniLightPDF(const OmniLight& light) const { return m_OmniLights.GetPDF(light); }
	float GetTotalOmniLightFlux() const { return m_TotalFlux_OmniLight; }

	const IBL& GetIBL() const { return m_IBL; }
	float GetTotalFluxIBL() const { return m_TotalFlux_IBL; }

};