#pragma once
#include<vector>
#include<algorithm>

namespace Sampler{
	template<typename C, typename T>
	class iSampler{
	private:
		//iSampler& operator=(const iSampler&);
		//iSampler(const iSampler&);
	public:
		iSampler(){}
		virtual C Sample(const T& texture, float u, float v)const = 0;
	};

	template<typename C, typename T>
	class PointSampler : public iSampler<C, T>{
	private:
	public:
		PointSampler(){}

		virtual C Sample(const T& texture, float u, float v)const;
		virtual ~PointSampler();
	};

	template<typename C, typename T>
	class LinearSampler : public iSampler<C, T>{
	private:
	public:
		LinearSampler(){}
		virtual C Sample(const T& texture, float u, float v)const;
		virtual ~LinearSampler();
	};

}

template<typename C, typename T>
Sampler::PointSampler<C, T>::~PointSampler(){

}

template<typename C, typename T>
C Sampler::PointSampler<C, T>::Sample(const T& texture, float u, float v)const{

	int Width = texture.GetWidth();
	int Height = texture.GetHeight();

	int x = std::min<int>(std::max<int>(0, Width * u + 0.5), Width - 1);
	int y = std::min<int>(std::max<int>(0, Height * v + 0.5), Height - 1);

	return texture.GetRawData()[x + y * Width];
}



template<typename C, typename T>
Sampler::LinearSampler<C, T>::~LinearSampler(){
}

template<typename C, typename T>
C Sampler::LinearSampler<C, T>::Sample(const T& texture, float u, float v)const{
	int Width = texture.GetWidth();
	int Height = texture.GetHeight();

	int x1 = std::abs(int(Width * u)) % Width;
	int y1 = std::abs(int(Height * v)) % Height;

	const C& t_00 = texture.GetRawData()[x1 + y1 * Width];
	const C& t_10 = texture.GetRawData()[std::min<int>(Width - 1, x1 + 1) + y1 * Width];
	const C& t_01 = texture.GetRawData()[x1 + std::min<int>(Height - 1, y1 + 1) * Width];
	const C& t_11 = texture.GetRawData()[std::min<int>(Width - 1, x1 + 1) + std::min<int>(Height - 1, y1 + 1) * Width];

	float w = (std::abs(u * Width) - std::abs(int(Width * u)));
	float h = (std::abs(v * Height) - std::abs(int(Height * v)));

	C up = t_00 * (1.0 - w) + t_10 * (w);
	C down = t_01 * (1.0 - w) + t_11 * (w);
	return up * (1.0 - h) + down *(h);
}