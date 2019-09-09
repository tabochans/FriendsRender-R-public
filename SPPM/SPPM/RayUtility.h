#pragma once
#include "Vec.h"
#include "Primitive.h"

namespace RUtil {

	const PTUtility::Vec3 INVALID_DIR(1e-3, 1e-3, 1e-3);

	PTUtility::Vec3 Reflect(const PTUtility::Vec3& inDir, const PTUtility::Vec3& normal);
	PTUtility::Vec3 Refract(const PTUtility::Vec3& inDir, const PTUtility::Vec3& normal, float index_from, float index_into);
	PTUtility::Vec3 GetMicrofacetNormal_FromReflection(const PTUtility::Vec3& inDir, const PTUtility::Vec3& outDir);
	PTUtility::Vec3 GetMicrofacetNormal_FromRefraction(const PTUtility::Vec3& inDir, const PTUtility::Vec3& outDir, float index_from, float index_into);
	bool isValidDirection(const PTUtility::Vec3& dir);

	bool IsReflect(const PTUtility::Vec3& inDir, const PTUtility::Vec3& outDir, const PTUtility::Vec3& normal);

};
