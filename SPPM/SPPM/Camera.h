#pragma once
#include <algorithm>
#include "Vec.h"
#include "Primitive.h"
#include "MT.h"
#include "CameraSampler.h"
#include "Texture.h"
#include "TableSampler.h"
#include <fstream>

struct iCamera {
	PTUtility::Vec3 _CameraPos;
	PTUtility::Vec3 _CameraDir;

	PTUtility::Vec3 _e1;
	PTUtility::Vec3 _e2;

	iCameraSsmpler* _CameraSampler;

	float _Fov;
	float _PlaneWidth;
	float _PlaneHeight;
	float _LensWidth;
	float _LensHeight;
	float _AspectRatio;
	float _F;
	int _ScreenWidth;
	int _ScreenHeight;

	float _DistanceToLens;

	virtual Primitive::Ray SampleCameraRay(float Image_u, float Image_v, int index, float& pdf) const = 0;

	iCamera(
		int ScreenWidth, int ScreenHeight, 
		const PTUtility::Vec3& CameraPos, const PTUtility::Vec3& Target, float Fov, float F, float DistanceToLens, float AspectRatio, iCameraSsmpler* CameraSampler) :
		_ScreenWidth(ScreenWidth), _ScreenHeight(ScreenHeight), _DistanceToLens(DistanceToLens), 
		_CameraPos(CameraPos), _CameraDir((Target - CameraPos).normalized()), _Fov(PTUtility::PI * Fov / 360.0f), _F(F), _AspectRatio(AspectRatio), _CameraSampler(CameraSampler) {

		_PlaneWidth = 2.0 * DistanceToLens * tan(_Fov) * _AspectRatio;
		_PlaneHeight = 2.0 * DistanceToLens * tan(_Fov);
		_LensWidth = _PlaneWidth;
		_LensHeight = _PlaneHeight;

		if (_CameraDir.y() > 0.99) {
			_e1 = PTUtility::Vec3(1, 0, 0);
			_e2 = PTUtility::Vec3(0, 0, 1);
		}
		else {
			_e1 = PTUtility::Vec3(0, 1, 0).cross(_CameraDir).normalized();
			_e2 = _CameraDir.cross(_e1).normalized();
		}
	}
};

struct OrthogonalCamera : public iCamera {
	virtual Primitive::Ray SampleCameraRay(float Image_u, float Image_v, int index, float& pdf) const {
		const PTUtility::Vec3 offset = _CameraSampler->Sample(index) * PTUtility::Vec3(_PlaneWidth, _PlaneHeight, 0) / PTUtility::Vec3(_ScreenWidth, _ScreenHeight, 1.0f);
		const PTUtility::Vec3 ImagePos = _CameraPos - _CameraDir * _DistanceToLens + (Image_u - 0.5f + offset.x()) * _e1 * _PlaneWidth + (Image_v - 0.5f + offset.y()) * _e2 * _PlaneHeight;
		pdf = 1.0f;

		return Primitive::Ray(ImagePos, _CameraDir);
	}

	OrthogonalCamera(int ScreenWidth, int ScreenHeight, 
		float ImagePlaneWidth, float ImagePlaneHeight, 
		const PTUtility::Vec3 & CameraPos, const PTUtility::Vec3 & Target, float AspectRatio, iCameraSsmpler * CameraSampler) :
		iCamera(ScreenWidth, ScreenHeight, CameraPos, Target, 90.0f, 1, ImagePlaneWidth, AspectRatio, CameraSampler) {
	}
};

struct PinholeCamera : public iCamera {
	virtual Primitive::Ray SampleCameraRay(float Image_u, float Image_v, int index, float& pdf) const {

		const PTUtility::Vec3 offset = _CameraSampler->Sample(index) * PTUtility::Vec3(_PlaneWidth, _PlaneHeight, 0) / PTUtility::Vec3(_ScreenWidth, _ScreenHeight, 1.0f);
		const PTUtility::Vec3 LensPos = _CameraPos;
		const PTUtility::Vec3 ImagePos = _CameraPos - _CameraDir * _DistanceToLens + (Image_u - 0.5f + offset.x()) * _e1 * _PlaneWidth + (Image_v - 0.5f + offset.y()) * _e2 * _PlaneHeight;

		pdf = 1.0f;

		return Primitive::Ray(ImagePos, (LensPos - ImagePos).normalized());
	}

	PinholeCamera(int ScreenWidth, int ScreenHeight, const PTUtility::Vec3& CameraPos, const PTUtility::Vec3& Target, float Fov, float F, float DistanceToLens, float AspectRatio, iCameraSsmpler* CameraSampler) :
		iCamera(ScreenWidth, ScreenHeight, CameraPos, Target, Fov, F, DistanceToLens, AspectRatio, CameraSampler) {
	}
};

struct ThinLensCamera : public iCamera {

private:

	struct UV_L {
		PTUtility::Vec2 UV;
		float L;
	};

	TableSampler<UV_L> m_Table;
	float m_HoleRatio;

public:

	virtual Primitive::Ray SampleCameraRay(float Image_u, float Image_v, int index, float& pdf) const {

		PTUtility::Vec3 LensUV; float pdf_area = 1.0f;
		if (m_HoleRatio > 0.999) {
			LensUV = _CameraSampler->Sample(index);
		}
		else {
			LensUV = PTUtility::Vec3(m_Table.Sample(_CameraSampler->Sample(index).xy()).UV, 0.0f);
		}
		pdf_area = 1.0f / (_LensWidth * _LensHeight);

		const PTUtility::Vec3 LensPos = _CameraPos + (LensUV.x() - 0.5f) * _e1 * _LensWidth + (LensUV.y() - 0.5f) * _e2 * _LensHeight;
		const PTUtility::Vec3 LensCenter = _CameraPos;
		const PTUtility::Vec3 ImagePos = _CameraPos - _CameraDir * _DistanceToLens + (Image_u - 0.5f) * _e1 * _PlaneWidth + (Image_v - 0.5f) * _e2 * _PlaneHeight;

		const float A = _DistanceToLens;
		const float B = (A * _F) / (A - _F);
		const PTUtility::Vec3 target = ImagePos + (LensCenter - ImagePos) * (A + B) / A;

		
		float cs = _CameraDir.dot((LensPos - ImagePos).normalized());
		float d2 = (LensPos - ImagePos).norm2();

		pdf = pdf_area * (d2 / _F / _F) / cs;

		return Primitive::Ray(LensPos, (target - LensPos).normalized());
	}

	ThinLensCamera(
		int ScreenWidth, int ScreenHeight, 
		const PTUtility::Vec3 & CameraPos, const PTUtility::Vec3 & Target, float Fov, float F, float DistanceToLens, float AspectRatio, iCameraSsmpler * CameraSampler, const char* DiaphragmImageFileName) :
		iCamera(ScreenWidth, ScreenHeight, CameraPos, Target, Fov, F, DistanceToLens, AspectRatio, CameraSampler) {

		if (DiaphragmImageFileName != nullptr) {
			Texture2D Dtex;
			TextureUtility::LoadTextureFromFile(Dtex, DiaphragmImageFileName);

			UV_L* img = new UV_L[Dtex.GetWidth() * Dtex.GetHeight()];

			int h = 0;
			for (int i = 0; i < Dtex.GetWidth() * Dtex.GetHeight(); ++i) {
				if (Dtex.GetRawData()[i][0] < 1e-6) {
					++h;
				}
				img[i].UV = PTUtility::Vec2((i % Dtex.GetWidth()) / (float)Dtex.GetWidth(), 1.0 - (i / Dtex.GetWidth()) / (float)Dtex.GetHeight());
				img[i].L = 1.0 - Dtex.GetRawData()[i][0];
			}
			m_HoleRatio = h / (float)(Dtex.GetWidth() * Dtex.GetHeight());

			m_Table.Init(img, Dtex.GetWidth() * Dtex.GetHeight(), [](const UV_L & x) {return x.L; });

			delete[] img;
		}
		else {
			m_HoleRatio = 1.0f;
		}
	}
};
