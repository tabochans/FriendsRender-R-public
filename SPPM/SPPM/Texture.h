#pragma once
#include <array>
#include "Vec.h"
#include <memory>
#include "TextureUtility.h"

/*
Describe texture which contains four normalized elements. 
*/
class Texture2D {
public:

private:

	unsigned int m_TextureWidth;
	unsigned int m_TextureHeight;

	std::unique_ptr<TColor[]> m_pData;

public:
	void Init(unsigned int Width, unsigned int Height) {
		m_TextureWidth = Width; m_TextureHeight = Height;
		m_pData.reset(new TColor[m_TextureWidth * m_TextureHeight]);
	}
	unsigned int GetWidth() const { return m_TextureWidth; }
	unsigned int GetHeight() const { return m_TextureHeight; }

	Texture2D() : m_TextureWidth(0), m_TextureHeight(0){
	}
	~Texture2D() {
	}

	TColor* GetRawData() {
		return m_pData.get();
	}
	const TColor* GetRawData() const{
		return m_pData.get();
	}
	unsigned int GetRawDataSize() const {
		return m_TextureHeight * m_TextureWidth * sizeof(TColor);
	}
};


namespace TextureUtility {



}