#pragma once

#include"Vec.h"
#include"Sampling.h"
#include"Camera.h"

namespace PTUtility{

	namespace Material {

		// if material is opaque, InDir.dot(OutDir) < 0
		struct FUNCTION_PARAM{
			Vec3 InDir;
			Vec3 OutDir;
			Vec3 Normal;
			float FromIndex;
			float ToIndex;
			float Roughness;
			float Absopution;
			float Reflectance;

			FUNCTION_PARAM(
				const PTUtility::Vec3& indir, const PTUtility::Vec3& outdir, const PTUtility::Vec3& normal, 
				float fromindex, float toindex, float roughness, float absopution, float reflectance) : 
				InDir(indir), OutDir(outdir), Normal(normal), FromIndex(fromindex), ToIndex(toindex), Roughness(roughness), Absopution(absopution), Reflectance(reflectance){
			}
			FUNCTION_PARAM(
				const PTUtility::Vec3& indir, const PTUtility::Vec3& normal, float fromindex, float toindex) : 
				InDir(indir), OutDir(Vec3(1, 0, 0)), Normal(normal), FromIndex(fromindex), ToIndex(toindex), Roughness(1.0f), Absopution(0.1), Reflectance(1.0f){
			}
			FUNCTION_PARAM() :
				InDir(Vec3(1, 0, 0)), OutDir(Vec3(1, 0, 0)), Normal(Vec3(1, 0, 0)), FromIndex(1.0f), ToIndex(1.0f), Roughness(1.0f), Absopution(0.1), Reflectance(1.0f) {
			}
		};
		struct SURFACE_PARAM {
			float Index;
			bool IfDeltaSurface;
			bool IfOpeque;
			PTUtility::Vec3 Albedo;
			float Roughness;
			float RoughnessU;
			float RoughnessV;
			PTUtility::Vec3 Radiance;
			float Reflectance;
			float Extinction;
			bool isSPPMSurface;
			bool Bidirectional;

			SURFACE_PARAM(
				float _Index, 
				bool _IfDeltaSurface, 
				bool _IfOpeque, 
				PTUtility::Vec3 _Albedo, 
				float _Roughness, 
				float _RoughnessU, 
				float _RoughnessV, 
				PTUtility::Vec3 _Radiance, 
				float _Reflectance, 
				float _Extinction, 
				bool _isSPPMSurface, 
				bool _Bidirectional)

				: Index(_Index), 
				IfDeltaSurface(_IfDeltaSurface), 
				IfOpeque(_IfOpeque), 
				Albedo(_Albedo), 
				Roughness(_Roughness), 
				RoughnessU(_RoughnessU), 
				RoughnessV(_RoughnessV), 
				Radiance(_Radiance), 
				Reflectance(_Reflectance), 
				Extinction(_Extinction), 
				isSPPMSurface(_isSPPMSurface), 
				Bidirectional(_Bidirectional){
			}
			SURFACE_PARAM() :
				Index(1.0f), 
				IfDeltaSurface(false), 
				IfOpeque(true), 
				Albedo(), 
				Roughness(1.0f), 
				RoughnessU(0.0f), 
				RoughnessV(0.0f), 
				Radiance(Vec3::Zero()), 
				Reflectance(1.0f), 
				Extinction(1.0f), 
				isSPPMSurface(false), 
				Bidirectional(false) {
			}
		};

		class Medium {

			Vec3 m_albedo;
			Vec3 m_sigmaT;

		public:
			Vec3 Tr(float dist) const {
				return Vec3(std::exp(-m_sigmaT.x() * dist), std::exp(-m_sigmaT.y() * dist), std::exp(-m_sigmaT.z() * dist));
			}
			float pdf_mid(float dist) {
				float pp = 0.0f;
				for (int i = 0; i < 3; i++) {
					pp += m_sigmaT[i] * Tr(dist)[i];
				}
				return pp * 0.33333f;
			}
			float pdf_exit(float dist) {
				float pp = 0.0f;
				for (int i = 0; i < 3; i++) {
					pp += Tr(dist)[i];
				}
				return pp * 0.33333f;
			}
			Vec3 Ss() const { return m_sigmaT * m_albedo; }
			Vec3 St() const { return m_sigmaT; }
			Vec3 Sa() const { return St() - Ss(); }
			Vec3 Albedo() const { return m_albedo; }

			virtual float SampleDistance(RandomMT& MT, int& c, float* pdf = nullptr) {
				c = 3 * MT.genrand64_real3();
				return -std::log(MT.genrand64_real3()) / m_sigmaT[c];
			}
			virtual Vec3 SampleScatteringDir(RandomMT& MT, const Vec3& in, float* pdf = nullptr) {
				return Sampling::Sphere_Sampling(MT, pdf);
			}
			virtual Vec3 Phase(const Vec3& in, const Vec3& out) {
				return m_albedo / (4.0f * PI);
			}

			Medium(const Vec3& albedo, const Vec3& sigmaT) : m_albedo(albedo), m_sigmaT(sigmaT){
			}
			virtual ~Medium() {
			}
		};

		class Surface {
		protected:
			inline bool OpeqeReflect(const FUNCTION_PARAM& param) {
				return (param.InDir.dot(-param.Normal) > 1e-4 && param.OutDir.dot(param.Normal) > 1e-4);
			}
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) = 0;
			virtual float BRDF(const FUNCTION_PARAM& param) = 0;
			virtual float PDF(const FUNCTION_PARAM& param) = 0;
		};

		class PlaneSurface : public Surface {
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				return Sampling::HemSphere_Sampling(MT, param.Normal, pdf);
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				if (OpeqeReflect(param)) {
					return 1.0f / (PTUtility::PI);
				}
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				return 1.0f / (2.0f * PTUtility::PI);
			}
		};

		class DiffuseSurface : public Surface {
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				return Sampling::Cos_Sampling(MT, param.Normal, pdf);
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				if (OpeqeReflect(param)) {
					return 1.0f / (PTUtility::PI);
				}
				return 0.0f;
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				return std::max(Sampling::sMin, param.OutDir.dot(param.Normal) / PTUtility::PI);
			}
		};

		class LightSurface_Albedo0 : public Surface {
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				return Sampling::HemSphere_Sampling(MT, param.Normal, pdf);
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				return 1.0f;
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				return 1.0f / (2.0f * PTUtility::PI);
			}
		};

		class VirtualPoint_Light : public Surface {
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				return Sampling::HemSphere_Sampling(MT, param.Normal, pdf);
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				return 1.0f;
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				return std::pow(1.0f, -2.0f);
			}
		};

		class PhongSurface : public Surface {
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				return Sampling::Phong_Sampling(MT, param.Normal, param.InDir, param.Roughness, pdf);
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				if (param.Normal.dot(param.OutDir) < 0) {
					return 0.0f;
				}
				PTUtility::Vec3 ref = Sampling::Reflect_Sampling(param.Normal, param.InDir);
				float cos = std::max(Sampling::sMin, std::min(1.0f, ref.dot(param.OutDir)));
				return std::pow(cos, param.Roughness) * ((param.Roughness + 1.0f) / (2.0f * PTUtility::PI));
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				return BRDF(param);
			}
		};

		class Roughconductor : public Surface {
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				return Sampling::GGX_Sampling(MT, param.Normal, param.InDir, param.Roughness, pdf);
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				//return 1.0f / PI;

				constexpr float smallValue = 1e-10;

				if (param.InDir.dot(param.Normal) > -smallValue || param.OutDir.dot(param.Normal) < smallValue) {
					return 0.0f;
				}

				float i_to = param.ToIndex;
				float i_from = param.FromIndex;
				float k = param.Absopution;
				float eta = i_to / i_from;
				float alpha = param.Roughness;

				Vec3 h = (-param.InDir + param.OutDir).normalized();
				float ccs = h.dot(param.Normal);
				if (ccs < smallValue || h.dot(param.InDir) > -smallValue || h.dot(param.OutDir) < smallValue) {
					return 0.0f;
				}
				float D = std::max(Sampling::sMin, (alpha * alpha) / (PI * std::pow((1.0001f - (1.0f - alpha * alpha) * ccs*ccs), 2.0f)));

				float lc = std::max(0.0f, std::min(0.9999f, param.InDir.dot(-param.Normal)));
				float lv = std::max(0.0f, std::min(0.9999f, param.OutDir.dot(param.Normal)));
				if (lc < Sampling::sMin || lv < Sampling::sMin) {
					return 0.0f;
				}

				float L_l = (-1.0f + std::sqrt(1.0f + alpha * alpha * (1.0f / (lc*lc) - 1.0))) * 0.5;
				float L_v = (-1.0f + std::sqrt(1.0f + alpha * alpha * (1.0f / (lv*lv) - 1.0))) * 0.5;
				float G = 1.0f / ((1.0f + L_l) * (1.0f + L_v));

				float fcos = param.Normal.dot(-param.InDir);
				float temp = eta * eta - k * k - (1.0f - fcos * fcos);
				float apb = std::sqrt(temp * temp + k * k *eta*eta);
				float a = std::sqrt((apb + temp) * 0.5);
				float tm1 = apb + fcos * fcos;
				float tm2 = a * 2.0f * fcos;
				float rs2 = (tm1 - tm2) / (tm1 + tm2);
				float tm3 = apb * fcos *fcos + (1.0f - fcos * fcos)*(1.0 - fcos * fcos);
				float tm4 = tm2 * (1.0f - fcos * fcos);
				float rp2 = rs2 * (tm3 - tm4) / (tm3 + tm4);
				float F = 0.5 * (rp2 + rs2);

				float fx = (4.0f * param.InDir.dot(-param.Normal) * param.OutDir.dot(param.Normal));

				return (F * G * D) / fx;
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				return Sampling::GGX_Sampling_PDF(param.OutDir, param.Normal, param.InDir, param.Roughness);
			}
		};

		class GGXGlass : public Surface {
		public:

			Vec3 Reflect(float fromI, float toI, const Vec3& in, const Vec3& n) {
				return (in - 2.0f * (n.dot(in)) * n).normalized();
			}
			Vec3 Refract(float fromI, float toI, const Vec3& in, const Vec3& n) {
				PTUtility::Vec3 NN = (in.dot(n) > 0) ? -n : n;
				float eta = fromI / toI;
				float LN = -in.dot(NN);

				float sq = std::max(0.00000001f, 1.0f - eta * eta*(1.0f - LN * LN));
				return (-NN * std::sqrt(sq) - eta * (-in - LN * NN)).normalized();
			}
			bool IfReflect(const Vec3& n, const Vec3& in, const Vec3& out) {
				return n.dot(in) * n.dot(out) < 0.0f;
			}
			Vec3 EstimateN(const Vec3& in, const Vec3& out, float I_i, float I_o) {
				Vec3 h;
				float c = std::abs(in.dot(out));
				float eta = I_o / I_i;
				float isq = eta * eta - 2.0f*eta*c + 1.0f;
				if (isq < Sampling::sMin) {
					return Vec3::Zero();
				}
				float ss = 1.0f / std::sqrt(isq);
				h = (-eta * ss * out + ss * in).normalized();
				if (I_o > I_i) { return h; }
				else { return -h; }
			}

			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				return Sampling::GGXGlass_Sampling(MT, param.Normal, param.InDir, param.Roughness, param.FromIndex, param.ToIndex, pdf);
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {

				constexpr float SmallValue = 1e-8;

				const bool isReflect = param.Normal.dot(param.InDir) * param.Normal.dot(param.OutDir) < 0;
				const float a = param.Roughness;
				const float Ii = param.FromIndex; const float Io = param.ToIndex;

				if (isReflect) {
					// ray is reflected

					// half vector
					const Vec3 h = (-param.InDir + param.OutDir).normalized();
					const float n_h = param.Normal.dot(h);
					if (n_h < 0) { return 0.0f; }

					// NDF
					const float D = (a * a) * std::pow(1.0f - (1.0f - a * a) * n_h * n_h, -2.0f) / PI;

					// Masking Function
					const float L_l = (-1.0f + std::sqrt(1.0001f + a * a * (std::pow(param.Normal.dot(-param.InDir), -2.0f) - 1.0f))) * 0.5f;
					const float L_v = (-1.0f + std::sqrt(1.0001f + a * a * (std::pow(param.Normal.dot(param.OutDir), -2.0f) - 1.0f))) * 0.5f;
					const float G = 1.0f / (L_l + L_v + 1.0f);

					// Fresnel reflectance
					const float F = Sampling::Fresnel_Reflectance(h, param.InDir, Ii, Io);

					const float wo_n = std::max(0.0f, std::min(1.0f, param.OutDir.dot(h)));
					const float wi_n = std::max(0.0f, std::min(1.0f, -param.InDir.dot(h)));
					if (wo_n < SmallValue || wi_n < SmallValue) { return 0.0f; }

					return F * G * D / (4.0f * wo_n * wi_n);
				}
				else {
					// ray is transmitted

					constexpr float SmallValue = 1e-8;
					const float eta = Io / Ii;

					// no interaction
					if (std::abs(eta - 1.0f) < SmallValue) { return 1.0; }

					// invalid refract
					if (param.InDir.dot(param.OutDir) < 0.0) { return 0.0f; }


					// calculate microfacet normal from refraction vector
					PTUtility::Vec3 h = EstimateN(param.InDir, param.OutDir, param.FromIndex, param.ToIndex);

					// if invalid normal
					if (h.norm() < SmallValue) { return 0.0f; }

					const float n_h = h.dot(param.Normal);
					if (n_h < SmallValue) { return 0.0f; }

					// NDF
					const float D = (a * a) * std::pow(1.0f - (1.0f - a * a) * n_h * n_h, -2.0f) / PI;

					// Masking Function
					const float L_l = (-1.0f + std::sqrt(1.0001f + a * a * (std::pow(param.Normal.dot(-param.InDir), -2.0f) - 1.0f))) * 0.5f;
					const float L_v = (-1.0f + std::sqrt(1.0001f + a * a * (std::pow(param.Normal.dot(-param.OutDir), -2.0f) - 1.0f))) * 0.5f;
					const float G = 1.0f / (L_l + L_v + 1.0f);

					// Fresnel reflectance
					const float F = Sampling::Fresnel_Reflectance(h, param.InDir, Ii, Io);

					const float wi_wm = -param.InDir.dot(h);
					const float wo_wm = -param.OutDir.dot(h);
					const float wi_n = -param.InDir.dot(param.Normal);
					const float wo_n = -param.OutDir.dot(param.Normal);

					if (wo_wm < SmallValue || wi_wm < SmallValue) { return 0.0f; }
					if (wi_n < SmallValue || wo_n < SmallValue) { return 0.0f; }

					return (wi_wm * wo_wm) / (wi_n * wo_n) * (Io * Io) * (1.0f - F) * G * D * std::pow(Ii * wo_wm + Io * wi_wm, -2.0f);
				}
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				return Sampling::GGXGlass_Sampling_PDF(param.OutDir, param.Normal, param.InDir, param.Roughness, param.FromIndex, param.ToIndex);
			}
		};

		class MetalSurface : public Surface {
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				if (pdf) {
					*pdf = Sampling::DeltaPDF;
				}
				return Sampling::Reflect_Sampling(param.Normal, param.InDir);
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				if (!OpeqeReflect(param)) {
					return 0.0f;
				}
				PTUtility::Vec3 ref = Sampling::Reflect_Sampling(param.Normal, param.InDir);
				if (ref.dot(param.OutDir) > 1.0f - 1.0f / Sampling::DeltaPDF) {
					return Sampling::DeltaPDF / (param.OutDir.dot(param.Normal) + 0.00001);
				}
				else {
					return 0.0f;
				}
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				PTUtility::Vec3 ref = Sampling::Reflect_Sampling(param.Normal, param.InDir);
				if (ref.dot(param.OutDir) > 1.0f - 1.0f / Sampling::DeltaPDF) {
					return Sampling::DeltaPDF;
				}
				else {
					return Sampling::sMin;
				}
			}
		};

		class BiMetalSurface : public Surface {
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				Vec3 N = param.Normal.dot(param.InDir) < 0 ? param.Normal : -param.Normal;

				if (pdf) {
					*pdf = Sampling::DeltaPDF;
				}
				return Sampling::Reflect_Sampling(N, param.InDir);
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				Vec3 N = param.Normal.dot(param.InDir) < 0 ? param.Normal : -param.Normal;

				if (!OpeqeReflect(param)) {
					return 0.0f;
				}

				PTUtility::Vec3 ref = Sampling::Reflect_Sampling(N, param.InDir);
				if (ref.dot(param.OutDir) > 1.0f - 1.0f / Sampling::DeltaPDF) {
					return Sampling::DeltaPDF / (param.OutDir.dot(N) + 0.00001);
				}
				else {
					return 0.0f;
				}
			}
			virtual float PDF(const FUNCTION_PARAM & param) {
				Vec3 N = param.Normal.dot(param.InDir) < 0 ? param.Normal : -param.Normal;

				PTUtility::Vec3 ref = Sampling::Reflect_Sampling(N, param.InDir);
				if (ref.dot(param.OutDir) > 1.0f - 1.0f / Sampling::DeltaPDF) {
					return Sampling::DeltaPDF;
				}
				else {
					return Sampling::sMin;
				}
			}
		};

		class GlassSurface : public Surface {
		private:
			Vec3 Reflect(float fromI, float toI, const Vec3& in, const Vec3& n) {
				return (in - 2.0f * (n.dot(in)) * n).normalized();
			}
			Vec3 Refract(float fromI, float toI, const Vec3& in, const Vec3& n) {
				float eta = fromI / toI;
				PTUtility::Vec3 NN = (in.dot(n) > 0) ? -n : n;
				float LN = -in.dot(NN);

				float sq = std::max(0.00000001f, 1.0f - eta * eta*(1.0f - LN * LN));
				return (-NN * std::sqrt(sq) - eta * (-in - LN * NN)).normalized();
			}
			bool IfReflect(const Vec3& n, const Vec3& in, const Vec3& out) {
				return n.dot(in) * n.dot(out) < 0.0f;
			}

		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				Vec3 NN = param.Normal.dot(param.InDir) < 0 ? param.Normal : -param.Normal;
				float reflectance = Sampling::Fresnel_Reflectance(NN, param.InDir, param.FromIndex, param.ToIndex);
				if (reflectance > MT.genrand64_real1()) {
					*pdf = reflectance;
					return Reflect(param.FromIndex, param.ToIndex, param.InDir, NN);
				}
				else {
					*pdf = 1.0f - reflectance;
					float eta = param.FromIndex / param.ToIndex;
					float LN = -param.InDir.dot(NN);
					float sq = 1.0f - eta * eta*(1.0 - LN * LN);
					if (sq < 0.000001) { return Reflect(param.FromIndex, param.ToIndex, param.InDir, param.Normal); }
					else { return Refract(param.FromIndex, param.ToIndex, param.InDir, param.Normal); }
				}
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				Vec3 NN = param.Normal.dot(param.InDir) < 0 ? param.Normal : -param.Normal;
				float reflectance = Sampling::Fresnel_Reflectance(NN, param.InDir, param.FromIndex, param.ToIndex);
				if (IfReflect(param.Normal, param.InDir, param.OutDir)) {
					return Sampling::DeltaPDF * reflectance / (std::abs(param.OutDir.dot(NN)) + 0.0001);
				}
				else {
					return Sampling::DeltaPDF * (1.0f - reflectance) / (std::abs(param.OutDir.dot(NN)) + 0.0001);
				}
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				PTUtility::Vec3 NN = param.Normal.dot(param.InDir) < 0 ? param.Normal : -param.Normal;
				float reflectance = Sampling::Fresnel_Reflectance(NN, param.InDir, param.FromIndex, param.ToIndex);

				if (IfReflect(param.Normal, param.InDir, param.OutDir)) {
					PTUtility::Vec3 ref = Sampling::Reflect_Sampling(NN, param.InDir);
					if (std::abs(ref.dot(param.OutDir)) > 0.99) {
						return Sampling::DeltaPDF * reflectance;
					}
					else {
						return Sampling::sMin;
					}
				}
				else {
					PTUtility::Vec3 rac = Sampling::Refract_Sampling(NN, param.InDir, param.FromIndex, param.ToIndex);
					if (std::abs(rac.dot(param.OutDir)) > 0.99) {
						return Sampling::DeltaPDF * (1.0f - reflectance);
					}
					else {
						return Sampling::sMin;
					}
				}
			}
		};

		class Plastic : public Surface {
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {

				float FresnelReflectance = Sampling::Fresnel_Reflectance(param.Normal, param.InDir, param.FromIndex, param.ToIndex);

				if (FresnelReflectance > MT.genrand64_real1()) {
					// fresnel reflection

					if (pdf) {
						*pdf = Sampling::DeltaPDF * FresnelReflectance;
					}
					return Sampling::Reflect_Sampling(param.Normal, param.InDir);
				}
				else {
					// transmit

					float pdf2 = 1.0f;
					Vec3 reflectOnInterSurface = Sampling::Cos_Sampling(MT, param.Normal, &pdf2);
					Vec3 outDir = Sampling::Refract_Sampling(param.Normal, reflectOnInterSurface, param.ToIndex, param.FromIndex);
					if (outDir.dot(param.Normal) < Sampling::sMin) {
						*pdf = Sampling::sMin;
						return -param.Normal;
					}

					if (pdf) {
						*pdf = (1.0f - FresnelReflectance) * pdf2;
					}
					return outDir;
				}
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				if (param.Normal.dot(param.OutDir) < Sampling::sMin) {
					return 0.0f;
				}
				Vec3 reflect = Sampling::Reflect_Sampling(param.Normal, param.InDir);
				float FresnelReflectance = Sampling::Fresnel_Reflectance(param.Normal, param.InDir, param.FromIndex, param.ToIndex);

				if (reflect.dot(param.OutDir) > 0.999) {
					return Sampling::DeltaPDF * FresnelReflectance / (std::abs(param.OutDir.dot(param.Normal)) + 0.0001);
				}
				else {
					Vec3 reflectOnInterSurface = -Sampling::Refract_Sampling(param.Normal, -param.OutDir, param.FromIndex, param.ToIndex);
					float FresnelTransmittance = 1.0f - Sampling::Fresnel_Reflectance(param.Normal, reflectOnInterSurface, param.ToIndex, param.FromIndex);
					return (1.0f - FresnelReflectance) * FresnelTransmittance / PI;
				}
			}
			virtual float PDF(const FUNCTION_PARAM & param) {
				if (param.Normal.dot(param.OutDir) < Sampling::sMin) {
					return Sampling::sMin;
				}
				Vec3 reflect = Sampling::Reflect_Sampling(param.Normal, param.InDir);
				float FresnelReflectance = Sampling::Fresnel_Reflectance(param.Normal, param.InDir, param.FromIndex, param.ToIndex);

				if (reflect.dot(param.OutDir) > 0.999) {
					return Sampling::DeltaPDF * FresnelReflectance;
				}
				else {
					Vec3 reflectOnInterSurface = -Sampling::Refract_Sampling(param.Normal, -param.OutDir, param.FromIndex, param.ToIndex);
					float c = reflectOnInterSurface.dot(param.Normal);
					return (1.0f - FresnelReflectance) * c / PI;
				}
			}
		};


		class NonInteraction : public Surface {
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				if (*pdf) { *pdf = 1.0f; }
				return param.InDir;
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				float cos = std::abs(param.Normal.dot(param.OutDir));
				return 1.0f / (cos + 0.000001);
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				return 1.0f;
			}

		};

		class HalfTransmit : public Surface {
			static constexpr float reflectance = 0.1f;
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				if (MT.genrand64_real1() > reflectance) {
					return param.InDir;
				}
				else {
					return -param.InDir;
				}
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				float cos = param.Normal.dot(param.OutDir);
				if (cos > 0) {
					return reflectance / (cos + 0.0001f);
				}
				else {
					return (reflectance - 1.0f) / (cos - 0.0001f);
				}
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				if (param.Normal.dot(param.OutDir) > 0) {
					return reflectance;
				}
				else {
					return 1.0f - reflectance;
				}
			}
		};

		class DiffuseTransmitSurface : public Surface {
		private:
			bool IfReflect(const Vec3& n, const Vec3& in, const Vec3& out) {
				return n.dot(in) * n.dot(out) < 0.0f;
			}
		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				Vec3 n = param.Normal.dot(param.InDir) > 0 ? param.Normal : -param.Normal;
				Vec3 rs;
				if (param.Reflectance > MT.genrand64_real1()) {
					rs = Sampling::Cos_Sampling(MT, -n, pdf);
					*pdf *= param.Reflectance;
				}
				else {
					rs = Sampling::Cos_Sampling(MT, n, pdf);
					*pdf *= (1.0f - param.Reflectance);
				}
				return rs;
			}
			virtual float BRDF(const FUNCTION_PARAM& param) {
				// param1 describes reflectance
				if (IfReflect(param.Normal, param.InDir, param.OutDir)) {
					return (param.Reflectance) / (PTUtility::PI);
				}
				else{
					return (1.0f - param.Reflectance) / (PTUtility::PI);
				}
				return 0.0f;
			}
			virtual float PDF(const FUNCTION_PARAM& param) {
				float cos = std::abs(param.OutDir.dot(param.Normal) + 0.00001);
				if (IfReflect(param.Normal, param.InDir, param.OutDir)) {
					return (param.Reflectance) * cos / (PTUtility::PI);
				}
				else {
					return (1.0f - param.Reflectance) * cos / (PTUtility::PI);
				}
			}
		};

		class GlossTransmitSurface : public Surface {
		private:
			bool IfReflect(const Vec3& n, const Vec3& in, const Vec3& out) const{
				return n.dot(in)* n.dot(out) < 0.0f;
			}
			Vec3 ReflectByN(const Vec3& r, const Vec3& n) const{
				Vec3 M = (r - r.dot(n) * n).normalized();
				return r - 2.0f * n.dot(r) * n;
			}

		public:
			virtual Vec3 SampleNextDir(RandomMT& MT, const FUNCTION_PARAM& param, float* pdf) {
				Vec3 n = param.Normal.dot(param.InDir) < 0 ? param.Normal : -param.Normal;

				Vec3 rs;
				if (param.Reflectance > MT.genrand64_real1()) {
					// reflect

					rs = Sampling::GGX_Sampling(MT, n, param.InDir, param.Roughness, pdf);
					*pdf *= param.Reflectance;
				}
				else {
					// transmit

					rs = ReflectByN(Sampling::GGX_Sampling(MT, n, param.InDir, param.Roughness, pdf), n);
					*pdf *= (1.0f - param.Reflectance);
				}
				return rs;
			}
			virtual float BRDF(const FUNCTION_PARAM & param) {
				// param1 describes reflectance

				float power = IfReflect(param.Normal, param.InDir, param.OutDir) ? param.Reflectance : 1.0f - param.Reflectance;

				Vec3 InDir = param.InDir;
				Vec3 Normal = param.Normal.dot(InDir) < 0 ? param.Normal : -param.Normal;
				
				Vec3 OutDir = param.OutDir;
				if (OutDir.dot(Normal) < 0) { OutDir = ReflectByN(OutDir, Normal); }
				
				constexpr float smallValue = 1e-10;

				if (InDir.dot(Normal) > -smallValue || OutDir.dot(Normal) < smallValue) {
					return 0.0f;
				}

				float i_to = 1.2;
				float i_from = 1.0;
				float alpha = param.Roughness;

				Vec3 h = (-InDir + OutDir).normalized();
				float ccs = h.dot(Normal);
				if (ccs < smallValue || h.dot(InDir) > -smallValue || h.dot(OutDir) < smallValue) {
					return 0.0f;
				}
				float D = std::max(Sampling::sMin, (alpha * alpha) / (PI * std::pow((1.0001f - (1.0f - alpha * alpha) * ccs * ccs), 2.0f)));

				float lc = std::max(0.0f, std::min(0.9999f, InDir.dot(-Normal)));
				float lv = std::max(0.0f, std::min(0.9999f, OutDir.dot(Normal)));
				if (lc < Sampling::sMin || lv < Sampling::sMin) {
					return 0.0f;
				}

				float L_l = (-1.0f + std::sqrt(1.0f + alpha * alpha * (1.0f / (lc * lc) - 1.0))) * 0.5;
				float L_v = (-1.0f + std::sqrt(1.0f + alpha * alpha * (1.0f / (lv * lv) - 1.0))) * 0.5;
				float G = 1.0f / ((1.0f + L_l) * (1.0f + L_v));

				float fx = (4.0f * InDir.dot(-Normal) * OutDir.dot(Normal));

				return power * (G * D) / fx;
			}
			virtual float PDF(const FUNCTION_PARAM & param) {
				float cos = std::abs(param.OutDir.dot(param.Normal) + 0.00001);
				if (IfReflect(param.Normal, param.InDir, param.OutDir)) {
					return param.Reflectance * Sampling::GGX_Sampling_PDF(param.OutDir, param.Normal, param.InDir, param.Roughness);
				}
				else {
					return (1.0f - param.Reflectance) * Sampling::GGX_Sampling_PDF(ReflectByN(param.OutDir, param.Normal), param.Normal, param.InDir, param.Roughness);
				}
			}
		};

	}
}
