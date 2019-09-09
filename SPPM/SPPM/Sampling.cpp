#include"Sampling.h"
#include "RayUtility.h"

using namespace PTUtility;

namespace Sampling {
	Vec3 Cos_Sampling(RandomMT& MT, const Vec3& Normal, float* pdf) {
		Vec3 result;
		Vec3 e1, e2, e3;
		e1 = Normal;

		if (abs(e1.x()) > 0.000001) {
			e2 = Vec3(0, 1, 0).cross(e1).normalized();
		}
		else {
			e2 = Vec3(1, 0, 0).cross(e1).normalized();
		}
		e3 = e1.cross(e2).normalized();

		float ph = MT.genrand64_real1() * 2.0f * PTUtility::PI;
		float th = MT.genrand64_real1();
		float th2 = sqrt(th);

		result = e1 * sqrt(1.000000001 - th) + e2 * cos(ph) * th2 + e3 * sin(ph) * th2;
		result.normalize();
		if (pdf) * pdf = std::max(Sampling::sMin, result.dot(Normal) / PTUtility::PI);
		return result;
	}
	float Cos_Sampling_PDF(const Vec3& outDir, const Vec3& Normal) {
		float c = outDir.dot(Normal);
		if (c > 0) {
			return c / PI;
		}
		else {
			return sMin;
		}
	}

	Vec3 Bi_Cos_Sampling(RandomMT & MT, const Vec3 & Normal, float Reflectance, float* pdf) {
		Vec3 result;
		if (MT.genrand64_real1() < Reflectance) {
			//Reflect

			result = Cos_Sampling(MT, Normal, pdf);
			if (pdf) {
				*pdf *= Reflectance;
			}
		}
		else {
			//Transmit

			result = Cos_Sampling(MT, -Normal, pdf);
			if (pdf) {
				*pdf *= (1.0 - Reflectance);
			}
		}
		return result;
	}

	Vec3 Sphere_Sampling(RandomMT & MT, float* pdf) {
		float ph = MT.genrand64_real1() * 2.0f * PI;
		float th = MT.genrand64_real1() * PI;
		th = acos(MT.genrand64_real1() * 2.0f - 1.0f);

		Vec3 result(sinf(th) * cosf(ph), sinf(th) * sinf(ph), cosf(th));
		result.normalize();
		if (pdf) * pdf = 1.0 / (4.0 * PI);
		return result;
	}
	Vec3 HemSphere_Sampling(RandomMT & MT, const Vec3 & Normal, float* pdf) {
		Vec3 e1, e2, e3;
		e1 = Normal;
		if (abs(e1.x()) > 0.00001) {
			e2 = Vec3(0, 1, 0).cross(e1).normalized();
		}
		else {
			e2 = Vec3(1, 0, 0).cross(e1).normalized();
		}
		e3 = e1.cross(e2).normalized();

		float ph = MT.genrand64_real1() * 2.0f * PI;
		float th = MT.genrand64_real1() * PI * 0.5f;
		th = acos(MT.genrand64_real1());

		Vec3 result(e1 * cosf(th) + e2 * cos(ph) * sinf(th) + e3 * sin(ph) * sinf(th));
		result.normalize();
		if (pdf) * pdf = 1.0 / (2.0 * PI);

		return result;
	}
	Vec3 Phong_Sampling(RandomMT & MT, const Vec3 & Normal, const Vec3 & InDir, float Power, float* pdf) {
		Vec3 e1, e2, e3;
		Vec3 result;

		e1 = Reflect_Sampling(Normal, InDir);

		if (abs(e1.x()) < 0.00001) {
			e2 = Vec3(1, 0, 0).cross(e1).normalized();
		}
		else {
			e2 = Vec3(0, 1, 0).cross(e1).normalized();
		}
		e3 = e1.cross(e2).normalized();

		float ph = MT.genrand64_real1() * 2.0f * PTUtility::PI;
		float th = MT.genrand64_real1();
		float cth2 = pow(1 - th, 1.0f / (Power + 1.0f));

		float cph = cos(ph);
		float sph = sin(ph);
		float sth2 = sqrt(1.0001f - cth2 * cth2);

		result = Vec3(e3 * sth2 * cph + e2 * sth2 * sph + e1 * cth2).normalized();
		if (result.dot(Normal) < 0) {
			if (pdf)* pdf = 0.00001f;
		}
		else {
			if (pdf)* pdf = std::pow(std::max(0.0f, std::min(1.0f, result.dot(e1))), Power) * ((Power + 1.0f) / (2.0f * PTUtility::PI));
		}
		return result;
	}

	Vec3 GGX_Sampling(RandomMT & MT, const Vec3 & Normal, const Vec3 & InDir, float Alpha, float* pdf)
	{
		Vec3 mN, result;
		Vec3 e1 = Normal;
		float phi = MT.genrand64_real1() * 2.0f * PTUtility::PI;
		float e = MT.genrand64_real1();
		float cth = std::sqrt((1.0 - e) / (e * (Alpha * Alpha - 1.0f) + 1.0f));

		Vec3 e2, e3;
		if (abs(e1.x()) > 0.000001) {
			e2 = Vec3(0, 1, 0).cross(e1).normalized();
		}
		else {
			e2 = Vec3(1, 0, 0).cross(e1).normalized();
		}
		e3 = e1.cross(e2).normalized();

		float cph = cos(phi);
		float sph = sin(phi);
		float sth = std::sqrt(1.0f - cth * cth);
		mN = Vec3(e3 * sth * cph + e2 * sth * sph + e1 * cth).normalized();

		if (mN.dot(InDir) > 0.0) {
			*pdf = sMin;
			return RUtil::INVALID_DIR;
		}
		result = Reflect_Sampling(mN, InDir);
		if (result.dot(Normal) < 0.0) {
			*pdf = sMin;
			return RUtil::INVALID_DIR;
		}
		float wm_N = mN.dot(Normal); float wo_N = result.dot(Normal); float wo_wm = result.dot(mN);
		*pdf = (Alpha * Alpha) / (PI * std::pow((1.0f - (1.0f - Alpha * Alpha) * wm_N * wm_N), 2)) * wm_N / (4.0f * wo_wm);
		return result;
	}
	float GGX_Sampling_PDF(const Vec3& OutDir, const Vec3& Normal, const Vec3& InDir, float Alpha) {

		const float a = Alpha;

		if (RUtil::IsReflect(InDir, OutDir, Normal)) {

			const Vec3 h = RUtil::GetMicrofacetNormal_FromReflection(InDir, OutDir);
			const float h_n = h.dot(Normal);
			if (h_n < 0.0f) { return sMin; }

			const float wi_h = -InDir.dot(h);
			if (wi_h < 0.0f) { return sMin; }

			const float wo_h = OutDir.dot(h);
			if (wo_h < 0.0f) { return sMin; }

			const float jc = 1.0f / (4.0f * wo_h);
			return (a * a / PI * std::pow(1.0f - (1.0f - a * a) * h_n * h_n, -2.0f)) * h_n * jc;
		}
		else {
			return sMin;
		}
	}
	Vec3 GGXGlass_Sampling(RandomMT & MT, const Vec3 & Normal, const Vec3 & InDir, float Alpha, float In_Index, float Out_Index, float* pdf)
	{
		constexpr float SmallValue = 1e-12;

		Vec3 mN, result = Normal;
		Vec3 NN = InDir.dot(Normal) < 0 ? Normal : -Normal;
		*pdf = 1.0f;
		float Ii = In_Index; float Io = Out_Index;

		Vec3 e1 = NN;
		float phi = MT.genrand64_real1() * 2.0f * PTUtility::PI;
		float e = MT.genrand64_real1();
		float cth = std::sqrt((1.0000001 - e) / (e * (Alpha * Alpha - 0.99999) + 1.0f));

		Vec3 e2, e3;
		if (abs(e1.x()) > 0.000001) {
			e2 = Vec3(0, 1, 0).cross(e1).normalized();
		}
		else {
			e2 = Vec3(1, 0, 0).cross(e1).normalized();
		}
		e3 = e1.cross(e2).normalized();

		float cph = cos(phi);
		float sph = sin(phi);
		float sth = std::sqrt(1.00000001f - cth * cth);
		mN = Vec3(e3 * sth * cph + e2 * sth * sph + e1 * cth).normalized();
		float wi_wm = -InDir.dot(mN);
		float wm_N = mN.dot(NN);
		float wi_N = -InDir.dot(NN);

		if (wi_N < SmallValue) { *pdf = sMin; return RUtil::INVALID_DIR; }
		if (wi_wm < SmallValue) { *pdf = sMin; return RUtil::INVALID_DIR; }
		if (wm_N < SmallValue) { *pdf = sMin; return RUtil::INVALID_DIR; }

		float w = 1.0f;
		float reflectance = Fresnel_Reflectance(mN, InDir, Ii, Io);
		if (reflectance > MT.genrand64_real1()) {
			*pdf = reflectance;
			result = Reflect_Sampling(mN, InDir);

			float wo_wm = result.dot(mN);
			if (wo_wm < SmallValue) { return RUtil::INVALID_DIR; }

			if (result.dot(NN) * InDir.dot(NN) > SmallValue) {
				*pdf = sMin;
				return RUtil::INVALID_DIR;
			}
			float wo_N = result.dot(NN);
			*pdf = reflectance * (wm_N / (4.0f * wo_wm) * Alpha * Alpha / (PTUtility::PI * std::pow(1.0f - (1.0f - Alpha * Alpha) * wm_N * wm_N, 2.0f)));
		}
		else {
			float eta = Ii / Io;
			result = RUtil::Refract(InDir, mN, Ii, Io);

			float wo_wm = -result.dot(mN);

			float wo_N = -result.dot(NN);
			if (wi_N * wo_N < SmallValue || wo_N < SmallValue || wo_wm < SmallValue) {
				*pdf = sMin;
				return RUtil::INVALID_DIR;
			}
			*pdf = (1.0f - reflectance) * wm_N * (Io * Io * wo_wm / (std::pow(Ii * wi_wm + Io * wo_wm, 2))) * Alpha * Alpha / (PI * std::pow(1.0f - (1.0f - Alpha * Alpha) * wm_N * wm_N, 2.0f));
			// *pdf = (1.0f - reflectance) * (wi_wm * wo_wm * Io * Io / (wo_N * std::pow(Ii * wi_wm + Io * wo_wm, 2))) * Alpha * Alpha / (PTUtility::PI * std::pow(1.0f - (1.0f - Alpha * Alpha) * wm_N * wm_N, 2.0f));
		}

		return result;
	}
	float GGXGlass_Sampling_PDF(const Vec3 & OutDir, const Vec3 & Normal, const Vec3 & InDir, float Alpha, float In_Index, float Out_Index) {
		const float a = Alpha;
		const float Ii = In_Index; const float Io = Out_Index;

		if (RUtil::IsReflect(InDir, OutDir, Normal)) {

			const Vec3 h = RUtil::GetMicrofacetNormal_FromReflection(InDir, OutDir);
			const float h_n = h.dot(Normal);
			if (h_n < 0.0f) { return sMin; }

			const float wi_h = -InDir.dot(h);
			if (wi_h < 0.0f) { return sMin; }

			const float wo_h = OutDir.dot(h);
			if (wo_h < 0.0f) { return sMin; }

			const float reflectance = Fresnel_Reflectance(h, InDir, Ii, Io);

			const float jc = 1.0f / (4.0f * wo_h);

			return reflectance * h_n * (a * a / PI * std::pow(1.0f - (1.0f - a * a) * h_n * h_n, -2.0f)) * jc;
		}
		else {

			const Vec3 h = RUtil::GetMicrofacetNormal_FromRefraction(InDir, OutDir, In_Index, Out_Index);
			const float h_n = h.dot(Normal);
			if (h_n < 0.0f) { return sMin; }

			const float wi_h = -InDir.dot(h);
			if (wi_h < 0.0f) { return sMin; }

			const float wo_h = -OutDir.dot(h);
			if (wo_h < 0.0f) { return sMin; }

			const float reflectance = Fresnel_Reflectance(h, InDir, Ii, Io);

			const float jc = Io * Io * wo_h * std::pow(Ii * wi_h + Io * wo_h, -2.0f);
			return (1.0f - reflectance) * h_n * (a * a / PI * std::pow(1.0f - (1.0f - a * a) * h_n * h_n, -2.0f)) * jc;
		}
	}




	Vec3 Reflect_Sampling(const Vec3 & Normal, const Vec3 & InDir) {
		return (InDir - 2.0f * (Normal.dot(InDir)) * Normal).normalized();
	}

	float Fresnel_Reflectance(const Vec3 & Normal, const Vec3 & InDir, float In_Index, float Out_Index) {
		float n1_div_n2 = (In_Index / Out_Index);
		float cosA = std::abs(Normal.dot(InDir));
		float sinA = std::sqrt(1.0000001f - cosA * cosA);
		float sinB = n1_div_n2 * sinA;
		if (sinB > 0.999999) {
			return 1.0f;
		}
		float cosB = std::sqrt(1.0000001f - sinB * sinB);

		float tp, rp, ts, rs;
		tp = (2.0f * In_Index * cosA) / (Out_Index * cosA + In_Index * cosB);
		rp = (Out_Index * cosA - In_Index * cosB) / (Out_Index * cosA + In_Index * cosB);
		ts = (2.0f * In_Index * cosA) / (In_Index * cosA + Out_Index * cosB);
		rs = (In_Index * cosA - Out_Index * cosB) / (In_Index * cosA + Out_Index * cosB);
		return (0.5f * rs * rs) + (0.5f * rp * rp);
	}

	float FresnelConductor_Reflectance(const Vec3 & Normal, const Vec3 & InDir, float Index, float absorption) {
		const float cos = -InDir.dot(Normal);
		const float idk2 = (Index * Index + absorption * absorption);
		const float icos2 = 2.0f * Index * cos;

		float rp_u = (idk2 * cos * cos + 1.0001);
		float rp = (rp_u - icos2) / (rp_u + icos2);

		float rs_u = (idk2 + cos * cos);
		float rs = (rs_u - icos2) / (rs_u + icos2);

		return 0.5f * (rs * rs + rp * rp);
	}

	Vec3 Refract_Sampling(const Vec3 & Normal, const Vec3 & InDir, float In_Index, float Out_Index) {

		float eta = In_Index / Out_Index;
		Vec3 NN = (InDir.dot(Normal) > 0) ? -Normal : Normal;
		float LN = -InDir.dot(NN);

		float sq = std::max(0.00000001f, 1.0f - eta * eta * (1.0f - LN * LN));
		return (-NN * std::sqrt(sq) - eta * (-InDir - LN * NN)).normalized();
	}

	Vec3 PointOnTriangle(RandomMT & MT, const std::array<Vec3, 3> & points, float* pdf) {
		float u = MT.genrand64_real1();
		float v = MT.genrand64_real1();

		if (v > u) { u *= 0.5f; v -= u; }
		else { v *= 0.5f; u -= v; }
		//if (u + v > 1.0f) { u = 1.0f - u; v = 1.0f - v; }

		if (pdf) { *pdf = 2.0f / std::abs((points[1] - points[0]).cross(points[2] - points[0]).norm()); }
		return u * points[0] + v * points[1] + (1.0f - u - v) * points[2];
	}
};