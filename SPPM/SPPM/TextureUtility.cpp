#include "TextureUtility.h"
#include "TextureSampler.h"
#include "stb_image.hpp"
#include "stb_image_write.h"
#include "Texture.h"
#include "Vec.h"

using namespace PTUtility;

namespace TextureUtility {

	bool LoadTextureFromFile(Texture2D& texture, const std::string& fileName) {
		unsigned char* data = nullptr;
		int X, Y, C = 0;

		data = stbi_load(fileName.c_str(), &X, &Y, &C, 0);
		if (!data) {
			return false;
		}
		texture.Init(X, Y);

		//Load Image to m_Image;
		for (int i = 0; i < X; ++i) {
			for (int j = Y - 1; j >= 0; --j) {
				TColor color(0.0f, 0.0f, 0.0f, 1.0f);
				int IDX = j * X + i;
				for (int c = 0; c < C; ++c) {
					color[c] = data[IDX * C + c] / 255.0f;
				}
				texture.GetRawData()[j * X + i] = color;
			}
		}
		stbi_image_free(data);
		return true;
	}

	void ExportBMP(const Texture2D& texture, const std::string& fileName){
		std::unique_ptr<unsigned char[]> data(new unsigned char[texture.GetHeight() * texture.GetWidth() * 4]);

		for (int i = 0; i < texture.GetWidth(); ++i) {
			for (int j = 0; j < texture.GetHeight(); ++j) {
				TColor color = TextureUtility::GetColor_UV(texture, i / (float)texture.GetWidth(), j / (float)texture.GetHeight());

				for (int c = 0; c < 4; ++c) {
					data.get()[(i + j * texture.GetWidth()) * 4 + c] = std::max(0, std::min(255, int(color[c] * 255)));
				}
			}
		}
		stbi_write_bmp(fileName.c_str(), texture.GetWidth(), texture.GetHeight(), 4, data.get());
	}

	void ExportPNG(const Texture2D& texture, const std::string& fileName) {
		std::unique_ptr<unsigned char[]> data(new unsigned char[texture.GetHeight() * texture.GetWidth() * 4]);

		for (int i = 0; i < texture.GetWidth(); ++i) {
			for (int j = 0; j < texture.GetHeight(); ++j) {
				TColor color = TextureUtility::GetColor_UV(texture, i / (float)texture.GetWidth(), j / (float)texture.GetHeight());

				for (int c = 0; c < 4; ++c) {
					data.get()[(i + j * texture.GetWidth()) * 4 + c] = std::max(0, std::min(255, int(color[c] * 255)));
				}
			}
		}
		stbi_write_png(fileName.c_str(), texture.GetWidth(), texture.GetHeight(), 4, data.get(), 0);
	}

	void Gamma(Texture2D& texture, const Texture2D& source, float gamma) {
		texture.Init(source.GetWidth(), source.GetHeight());
		unsigned int X = texture.GetWidth();
		unsigned int Y = texture.GetHeight();

		for (int i = 0; i < X; ++i) {
			for (int j = 0; j < Y; ++j) {
				int IDX = j * X + i;

				TColor c = source.GetRawData()[j * X + i];
				texture.GetRawData()[j * X + i] = TColor(std::pow(c[0], gamma), std::pow(c[1], gamma), std::pow(c[2], gamma), c[3]);
			}
		}
	}


	bool CopyTexture(Texture2D& destination, const Texture2D& source) {
		destination.Init(source.GetWidth(), source.GetHeight());
		unsigned int X = destination.GetWidth();
		unsigned int Y = destination.GetHeight();

		for (int i = 0; i < X; ++i) {
			for (int j = 0; j < Y; ++j) {
				int IDX = j * X + i;
				destination.GetRawData()[j * X + i] = source.GetRawData()[j * X + i];
			}
		}
		return true;
	}
	bool SetColor(Texture2D& texture, const TColor& color) {
		unsigned int X = texture.GetWidth();
		unsigned int Y = texture.GetHeight();

		for (int i = 0; i < X; ++i) {
			for (int j = 0; j < Y; ++j) {
				int IDX = j * X + i;
				texture.GetRawData()[j * X + i] = color;
			}
		}
		return true;
	}
	bool SetColorFromFunction(Texture2D& texture, std::function<TColor(unsigned int, unsigned int)> function) {
		unsigned int X = texture.GetWidth();
		unsigned int Y = texture.GetHeight();

		for (int i = 0; i < X; ++i) {
			for (int j = 0; j < Y; ++j) {
				int IDX = j * X + i;
				texture.GetRawData()[j * X + i] = function(i, j);
			}
		}
		return true;
	}
	TColor GetColor_UV(const Texture2D& texture, float u, float v) {
		static Sampler::LinearSampler<TColor, Texture2D> sampler;
		return sampler.Sample(texture, u, v);
	}
	TColor GetColor_Index(const Texture2D& texture, unsigned int X, unsigned int Y) {
		return texture.GetRawData()[X + texture.GetWidth() * Y];
	}

	bool CreateNormalmapFromHightmap(Texture2D& normalmap, const Texture2D& heightmap) {
		const int Width = heightmap.GetWidth();
		const int Height = heightmap.GetHeight();

		static auto SX = [Width](int i) { return std::max(0, std::min((int)Width - 1, i)); };
		static auto SY = [Height](int i) { return std::max(0, std::min((int)Height - 1, i)); };

		constexpr int ksize = 1;
		const float pX = 1.0f / (float)Width;
		const float pY = 1.0f / (float)Height;
		const float hscale = 4.0f / std::sqrt((float)Width * (float)Height);

		normalmap.Init(Width, Height);

		for (int y = 0; y < Height; ++y) {
			for (int x = 0; x < Width; ++x) {

				const Vec3 H00(x * pX, y * pY, GetColor_Index(heightmap, SX(x), SY(y))[0] * hscale);
				const Vec3 H01((x + 1) * pX, y * pY, GetColor_Index(heightmap, SX(x + 1), SY(y))[0] * hscale);
				const Vec3 H10(x * pX, (y + 1) * pY, GetColor_Index(heightmap, SX(x), SY(y + 1))[0] * hscale);
				const Vec3 H11((x + 1) * pX, (y + 1) * pY, GetColor_Index(heightmap, SX(x + 1), SY(y + 1))[0] * hscale);

				const Vec3 u = (H01 - H00).normalized();
				const Vec3 v = (H10 - H00).normalized();

				const Vec3 normal = u.cross(v).normalized();
				const Vec3 n = (Vec3::Ones() + normal) * 0.5f;

				normalmap.GetRawData()[x + y * Width] = TColor(n[0], n[1], n[2], 1.0f);
			}
		}
		return true;
	}
	PTUtility::Vec3 GetNormalFromNormalmap(const Texture2D& normalmap, float u, float v) {
		TColor n = GetColor_UV(normalmap, u, v);
		return Vec3(2.0f * n[0] - 1.0f, 2.0f * n[1] - 1.0f, 2.0f * n[2] - 1.0f).normalized();
	}

};