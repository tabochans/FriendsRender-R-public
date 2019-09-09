#pragma once
#include"Vec.h"
#include"MT.h"
#include <array>

namespace Sampling{
	const float DeltaPDF = 1.0f;
	const float sMin = 1e-16f;

	float Fresnel_Reflectance(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index);
	float FresnelConductor_Reflectance(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Index, float absorption);

	PTUtility::Vec3 Cos_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, float* pdf = nullptr);
	float Cos_Sampling_PDF(const PTUtility::Vec3& outDir, const PTUtility::Vec3& Normal);

	PTUtility::Vec3 Bi_Cos_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, float Reflectance, float* pdf = nullptr);
	float Bi_Cos_Sampling_PDF(const PTUtility::Vec3& outDir, const PTUtility::Vec3& Normal, float Reflectance);
	
	PTUtility::Vec3 Sphere_Sampling(RandomMT& MT, float* pdf = nullptr);
	float Sphere_Sampling_PDF(const PTUtility::Vec3& outDir);
	
	PTUtility::Vec3 HemSphere_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, float* pdf = nullptr);
	float HemSphere_Sampling_PDF(const PTUtility::Vec3& outDir, const PTUtility::Vec3& Normal);

	PTUtility::Vec3 Phong_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Power, float* pdf = nullptr);
	float Phong_Sampling_PDF(const PTUtility::Vec3& outDir, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Power);

	PTUtility::Vec3 GGX_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Alpha, float* pdf = nullptr);
	float GGX_Sampling_PDF(const PTUtility::Vec3& outDir, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Alpha);

	PTUtility::Vec3 GGXGlass_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Alpha, float In_Index, float Out_Index, float* pdf = nullptr);
	float GGXGlass_Sampling_PDF(const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Alpha, float In_Index, float Out_Index);

	PTUtility::Vec3 Reflect_Sampling(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir);
	float Reflect_Sampling_PDF(const PTUtility::Vec3& outDir, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir);

	PTUtility::Vec3 Refract_Sampling(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index);
	float Refract_Sampling_PDF(const PTUtility::Vec3& outDir, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index);

	PTUtility::Vec3 PointOnTriangle(RandomMT& MT, const std::array<PTUtility::Vec3, 3>& points, float* pdf = nullptr);
	float PointOnTriangle_PDF(const PTUtility::Vec3& position, const std::array<PTUtility::Vec3, 3>& points);




}