#include "LightSampler.h"
#include "Normal_Facet.h"

using namespace PTUtility;

bool LightSampler::CreateLightSampler(const std::vector<AreaLight>& AreaLights, const std::vector<OmniLight>& OmniLights, const Texture2D* IBL){

	if (AreaLights.size() > 0) {
		m_TotalFlux_AreaLight = 0;
		for (const auto& t : AreaLights) {
			m_TotalFlux_AreaLight += LightSampler::AreaLight::Evaluate(t);
		}
		m_AreaLights.Init(&AreaLights[0], AreaLights.size(), LightSampler::AreaLight::Evaluate);
	}
	if (OmniLights.size() > 0) {
		m_TotalFlux_OmniLight = 0;
		for (const auto& t : OmniLights) {
			m_TotalFlux_OmniLight += LightSampler::OmniLight::Evaluate(t);
		}
		m_OmniLights.Init(&OmniLights[0], OmniLights.size(), LightSampler::OmniLight::Evaluate);
	}
	if (IBL) {
		m_TotalFlux_IBL = 0;
		m_IBL.Init(*IBL, IBL::Mapping::SPHERE, [](const TColor & c) {return 0.4 * TColor(c[0] * c[0], c[1] * c[1], c[2]* c[2], 0.0f); });
		m_TotalFlux_IBL = m_IBL.GetTotalFlux() * 4.0f * PI;
	}
	return true;
}

void LightSampler::CreateAreaLight(AreaLight& alight, const Primitive::inoutFacet& facet, const PTUtility::Vec3& radiance, const Texture2D* texture) {
	if (texture) {
		QRandSequence::Hammersley* s = new QRandSequence::Hammersley(NumSample);

		LightSampler::AreaLight::UV_L uvlArray[NumSample];
		Vec3 TotalFlux;
		for (int i = 0; i < NumSample; ++i) {
			Vec3 uvw = s->Sample(i);

			if (uvw[1] > uvw[0]) { uvw[0] *= 0.5f; uvw[1] -= uvw[0]; }
			else { uvw[1] *= 0.5f; uvw[0] -= uvw[1]; }
			Vec3 uv = uvw[0] * facet._UV[0] + uvw[1] * facet._UV[1] + (1.0f - uvw[0] - uvw[1]) * facet._UV[2];

			LightSampler::AreaLight::UV_L uvl;
			uvl.UV = uvw; uvl.L = TextureUtility::GetColor_UV(*texture, uv[0], uv[1]).xyz() * radiance;
			if (uvl.L.norm2() < 0.01f) { uvl.L = Vec3::Zero(); }
			uvlArray[i] = uvl;
			TotalFlux += uvl.L;
		}
		if (TotalFlux.norm2() < 1e-6) {
			uvlArray[0].L = 1e-6 * Vec3::Ones(); uvlArray[0].UV = Vec3::Zero();
		}
		else {
			TotalFlux *= (2.0f * PI * facet.CalculateArea() / (float)NumSample);
		}

		alight._Facet = facet;
		alight._Flux = TotalFlux;
		alight._Textured = true;
		alight.m_Table.Init(uvlArray, NumSample, [](const LightSampler::AreaLight::UV_L & c) { return c.L.norm();});

		delete s;
	}
	else {
		alight = AreaLight(facet, radiance * 2.0f * PI * facet.CalculateArea(), false);
	}
}

void LightSampler::SamplePointOnAreaLight(const AreaLight& arealight, RandomMT& MT, PTUtility::Vec3& Pos, PTUtility::Vec3& UV, PTUtility::Vec3& Radiance, float& pdf) const{
	if (arealight._Textured) {
		AreaLight::UV_L uvl = arealight.m_Table.Sample(MT);
		PTUtility::Vec3 uvw = uvl.UV;
		UV = uvw[0] * arealight._Facet._UV[0] + uvw[1] * arealight._Facet._UV[1] + (1.0f - uvw[0] - uvw[1]) * arealight._Facet._UV[2];
		Pos = uvw[0] * arealight._Facet._Pos[0] + uvw[1] * arealight._Facet._Pos[1] + (1.0f - uvw[0] - uvw[1]) * arealight._Facet._Pos[2];
		Radiance = uvl.L;
		pdf = (arealight.m_Table.GetPDF(uvl) * NumSample) / arealight._Facet.CalculateArea();
	}
	else {
		Pos = Sampling::PointOnTriangle(MT, arealight._Facet._Pos, &pdf);
		UV = PTUtility::Vec3::Zero();
		Radiance = arealight._Flux / (2.0f * PI * arealight._Facet.CalculateArea());
	}
}

float LightSampler::SampleAreaLight_pdf(const Primitive::inoutFacet& facet, const PTUtility::Vec3& radiance) const {
	AreaLight alight;
	GetAreaLightFromFacetID(alight, facet._FacetID);

	float ratio = 1.0f;
	if (alight._Textured) {
		AreaLight::UV_L uvl; uvl.L = radiance;
		ratio = alight.m_Table.GetPDF(uvl) * NumSample;
	}
	else {}

	return ratio / facet.CalculateArea();
}

bool LightSampler::GetAreaLightFromFacetID(AreaLight & light, int FacetID) const{
	for (int i = 0; i < m_AreaLights.GetNumElements(); ++i) {
		if (m_AreaLights.GetRawData()[i]._Facet._FacetID == FacetID) {
			light = m_AreaLights.GetRawData()[i];
			return true;
		}
	}
	return false;
}


float LightSampler::OmniLight::Evaluate(const OmniLight& t){
	return t._Flux.norm();
}

float LightSampler::AreaLight::Evaluate(const AreaLight& t){
	return t._Flux.norm();
}
