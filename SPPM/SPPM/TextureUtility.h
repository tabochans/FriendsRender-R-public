#pragma once
#include "TextureSampler.h"
#include <string>
#include <functional>
#include "Vec.h"
#include "Vec.h"

class Texture2D;
typedef PTUtility::Vec4 TColor;

namespace TextureUtility {
	
	bool LoadTextureFromFile(Texture2D& texture, const std::string& fileName);

	void Gamma(Texture2D& texture, const Texture2D& source, float gamma);
	void ExportBMP(const Texture2D& texture, const std::string& fileName);
	void ExportPNG(const Texture2D& texture, const std::string& fileName);

	bool CopyTexture(Texture2D& destination, const Texture2D& source);
	bool SetColor(Texture2D& texture, const TColor& color);
	bool SetColorFromFunction(Texture2D& texture, std::function<TColor(unsigned int, unsigned int)> function);
	TColor GetColor_UV(const Texture2D& texture, float u, float v);
	TColor GetColor_Index(const Texture2D& texture, unsigned int X, unsigned int Y);

	bool CreateNormalmapFromHightmap(Texture2D& normalmap, const Texture2D& heightmap);
	PTUtility::Vec3 GetNormalFromNormalmap(const Texture2D& normalmap, float u, float v);

};
