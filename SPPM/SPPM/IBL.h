#pragma once
#include "TableSampler.h"
#include "Texture.h"
#include <functional>
#include <fstream>
#include <cmath>
#include "QRandSequence.h"
#include "MT.h"

class IBL {
public:
	enum Mapping {
		CUBE = 0,
		SPHERE = 1,
		CYLINDER = 2,
		NONE = 3,
	};

private:

	static const int NumSample = 128 * 128;

	TableSampler<PTUtility::Vec3> m_Table;
	Texture2D m_Texture;
	Mapping m_Mapping;

	static constexpr float TanA = 0.1f;
	float m_TotalFlux = 0.0f;

	void createPoint2_rec(const std::array<PTUtility::Vec3, 3>& triangle, PTUtility::Vec3* points, int& index, int depth) {
		if (depth <= 0) {
			points[index++] = (triangle[0] + triangle[1] + triangle[2]).normalized();
			return;
		}

		PTUtility::Vec3 dp[3];
		for (int i = 0; i < 3; ++i) {
			dp[i] = (triangle[(i + 1) % 3] + triangle[(i + 2) % 3]).normalized();
		}
		
		const PTUtility::Vec3 T[4][3] = {
			{dp[0],dp[1],dp[2]}, 
			{triangle[0],dp[1],dp[2]},
			{triangle[1],dp[2],dp[0]},
			{triangle[2],dp[0],dp[1]},
		};

		for (int i = 0; i < 4; ++i) {
			createPoint2_rec({T[i][0], T[i][1] , T[i][2] }, points, index, depth - 1);
		}
	}

	PTUtility::Vec3* createPoint(int NumSample) {
		QRandSequence::Hammersley s(NumSample);
		PTUtility::Vec3* v = new PTUtility::Vec3[NumSample];

		for (int i = 0; i < NumSample; ++i) {
			float ph = s.Sample(i).x() * 2.0f * PTUtility::PI;
			float th = acos(s.Sample(i).y() * 2.0f - 1.0f);
			v[i] = PTUtility::Vec3(sinf(th) * cosf(ph), sinf(th) * sinf(ph), cosf(th));
		}

		return v;
	}

	PTUtility::Vec3* createPoint2(int NumDivide) {
		PTUtility::Vec3* v = new PTUtility::Vec3[(unsigned int)std::pow(4, (float)NumDivide)];

		constexpr float S = 2.0f * PTUtility::PI / 3.0f;
		const float R = std::sqrt(8.0f / 3.0f);

		PTUtility::Vec3 p[4];
		p[0] = PTUtility::Vec3(1.0, 0, 0);
		p[1] = PTUtility::Vec3(-1.0f / 3.0f, -0.5f * R, -std::sqrt(3) / 6.0f * R);
		p[2] = PTUtility::Vec3(-1.0f / 3.0f, 0.5f * R, -std::sqrt(3) / 6.0f * R);
		p[3] = PTUtility::Vec3(-1.0f / 3.0f, 0, std::sqrt(3) / 3.0f * R);

		int NumPoints = 0;
		for (int i = 0; i < 4; ++i) {
			createPoint2_rec({ p[(0 + i) % 4], p[(1 + i) % 4] , p[(2 + i) % 4] }, v, NumPoints, NumDivide - 1);
		}

		return v;
	}

public:

	float GetTotalFlux() const { return m_TotalFlux; }

	PTUtility::Vec3 GetUVFromDirection(const PTUtility::Vec3& dir) const {
		switch (m_Mapping) {
		case CUBE:
		{
			return PTUtility::Vec3(0, 0, 0);
		}
		case SPHERE:
		{
			if (std::abs(dir[0]) < 1e-6 && std::abs(dir[2]) < 1e-6) { return PTUtility::Vec3::Zero(); }
			PTUtility::Vec3 DirXZ = PTUtility::Vec3(dir[0], 0, dir[2]).normalized();
			float u = (DirXZ.z() >= 0 ? acos(DirXZ.x()) : 2.0f * PTUtility::PI - acos(DirXZ.x())) / (2.0f * PTUtility::PI);
			float v = acos(dir.y()) / PTUtility::PI;

			return PTUtility::Vec3(u, v, 0);
		}
		case CYLINDER:
		{
			const PTUtility::Vec3 DirXZ = PTUtility::Vec3(dir[0], 0, dir[2]).normalized();
			float u = (DirXZ.z() >= 0 ? acos(DirXZ.x()) : 2.0f * PTUtility::PI - acos(DirXZ.x())) / (2.0f * PTUtility::PI);

			float c = dir.y(); float s = std::sqrt(1.0f - c * c);
			float t = TanA;
			if (s > std::abs(c) * TanA + 0.0001f) { t = s / c; }
			float v = 0.5f - 1.0f / t * std::sqrt(1.0f + TanA * TanA - 0.25f);

			return PTUtility::Vec3(u, v, 0);
		}
		}

		return PTUtility::Vec3(0, 0, 0);
	}

	TColor GetIBLColorFromDirection(const PTUtility::Vec3& dir) const{
		if (m_Mapping == NONE) {
			TColor(0, 0, 0, 1);
		}

		PTUtility::Vec3 uvw = GetUVFromDirection(dir);
		return TextureUtility::GetColor_UV(m_Texture, uvw[0], uvw[1]);
	}

	PTUtility::Vec3 GetLightDirFromUV(float u, float v) const {

		switch (m_Mapping) {
		case CUBE:
		{
			return PTUtility::Vec3(0, 0, 0);
		}
		case SPHERE:
		{
			float diry = cos(PTUtility::PI * v);
			float xxzz = 1.0f - diry * diry;

			float dirx = u < 0.5f ? cos(2.0f * PTUtility::PI * u) : cos(2.0f * PTUtility::PI * (1.0f - u));
			float dirz = (u < 0.5f ? 1.0f : -1.0f) * std::sqrt(1.0f - dirx * dirx);
			dirz *= std::sqrt(xxzz); dirx *= std::sqrt(xxzz);

			return -PTUtility::Vec3(dirx, diry, dirz).normalized();
		}
		case CYLINDER:
		{
			float diry = 0.5f * (1 - 2.0f * v) * std::pow(v * v - v + TanA * TanA + 1.0f, -0.5f);
			float xxzz = 1.0f - diry * diry;

			float dirx = u < 0.5f ? cos(2.0f * PTUtility::PI * u) : cos(2.0f * PTUtility::PI * (1.0f - u));
			float dirz = (u < 0.5f ? 1.0f : -1.0f) * std::sqrt(1.0f - dirx * dirx);
			dirz *= std::sqrt(xxzz); dirx *= std::sqrt(xxzz);

			return -PTUtility::Vec3(dirx, diry, dirz).normalized();
		}
		}
	}

	bool Init(const Texture2D& IBLTexture, Mapping mappingMethod, std::function<TColor(const TColor& color)> ColorConvertFunction) {
		TextureUtility::CopyTexture(m_Texture, IBLTexture);

		unsigned int X = IBLTexture.GetWidth();
		unsigned int Y = IBLTexture.GetHeight();

		for (int i = 0; i < X; ++i) {
			for (int j = 0; j < Y; ++j) {
				int IDX = j * X + i;
				m_Texture.GetRawData()[j * X + i] = ColorConvertFunction(IBLTexture.GetRawData()[j * X + i]);
			}
		}

		m_TotalFlux = 0.0f;

		m_Mapping = mappingMethod;
		if (m_Mapping == CYLINDER) {

			PTUtility::Vec3* temp = new PTUtility::Vec3[m_Texture.GetRawDataSize() / sizeof(TColor)];
			for (int x = 0; x < m_Texture.GetWidth(); ++x) {
				for (int y = 0; y < m_Texture.GetHeight(); ++y) {
					temp[x + y * m_Texture.GetWidth()] = PTUtility::Vec3(x / (float)m_Texture.GetWidth(), y / (float)m_Texture.GetHeight(), ColorConvertFunction(TextureUtility::GetColor_Index(m_Texture, x, y)).norm());
				}
			}

			m_Table.Init(temp, m_Texture.GetRawDataSize() / sizeof(TColor), [&ColorConvertFunction](const PTUtility::Vec3 & c) {
				return c[2];
			});

			m_Mapping = CYLINDER;

			delete[] temp;
		}
		else if (m_Mapping == SPHERE) {
			
			PTUtility::Vec3* temp = createPoint(NumSample);

			//int NumDivide = 8;
			//int NumSample = std::pow(4, NumDivide);
			//PTUtility::Vec3* temp = createPoint2(NumDivide);

			for (int i = 0; i < NumSample; ++i) {
				PTUtility::Vec3 uvw = GetUVFromDirection(temp[i]);
				
				temp[i][0] = uvw.x(); temp[i][1] = uvw.y();
				temp[i][2] = ColorConvertFunction(TextureUtility::GetColor_UV(m_Texture, uvw[0], uvw[1]))[0];
				m_TotalFlux += temp[i][2];
			}
			m_TotalFlux /= (float)NumSample;

			m_Table.Init(temp, NumSample, [](const PTUtility::Vec3 & c) {
				return c[2];
			});

			delete[] temp;
		}


		return true;
	}

	int SamplePixelOnIBL(RandomMT& MT, PTUtility::Vec3& LightDir, TColor& IBLColor, float& pdf_Area) const {
		PTUtility::Vec3 UVL = m_Table.Sample(MT);
		LightDir = GetLightDirFromUV(UVL.x(), UVL.y());
		IBLColor = TextureUtility::GetColor_UV(m_Texture, UVL.x(), UVL.y());
		pdf_Area = m_Table.GetPDF(UVL) * (float)(NumSample);

		return 1;
	}

	explicit operator bool() { return m_Mapping != NONE;}
	explicit operator bool() const { return m_Mapping != NONE; }


	IBL() : m_Mapping(NONE) {}
	~IBL() {}

};
