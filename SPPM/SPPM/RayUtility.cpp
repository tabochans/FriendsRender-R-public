#include "RayUtility.h"

using namespace PTUtility;

namespace RUtil {

	Vec3 Reflect(const Vec3& inDir, const Vec3& normal) {
		return (inDir - 2.0f * (normal.dot(inDir)) * normal).normalized();
	}
	Vec3 Refract(const Vec3& inDir, const Vec3& normal, float index_from, float index_into) {
		const Vec3 NN = (inDir.dot(normal) < 0) ? normal : -normal;
		const float eta = index_from / index_into;
		const float LN = -inDir.dot(NN);

		const float sq = std::max(0.0f, 1.0f - eta * eta * (1.0f - LN * LN));
		return (-NN * std::sqrt(sq) - eta * (-inDir - LN * NN)).normalized();
	}
	Vec3 GetMicrofacetNormal_FromReflection(const Vec3& inDir, const Vec3& outDir) {
		return (-inDir + outDir).normalized();
	}
	Vec3 GetMicrofacetNormal_FromRefraction(const Vec3& inDir, const Vec3& outDir, float index_from, float index_into) {
		Vec3 h;
		float c = std::abs(inDir.dot(outDir));
		float eta = index_into / index_from;
		float isq = eta * eta - 2.0f * eta * c + 1.0f;
		if (isq < 1e-8) {
			return Vec3::Zero();
		}
		float ss = 1.0f / std::sqrt(isq);
		h = (-eta * ss * outDir + ss * inDir).normalized();
		if (index_into > index_from) { return h; }
		else { return -h; }
	}
	bool IsReflect(const Vec3& inDir, const Vec3& outDir, const Vec3& normal) {
		return normal.dot(inDir)* normal.dot(outDir) < 0.0f;
	}

	bool isValidDirection(const PTUtility::Vec3& dir) {
		return dir.norm2() > 0.9999f;
	}
};