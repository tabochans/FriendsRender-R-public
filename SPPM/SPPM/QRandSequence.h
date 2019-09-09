#pragma once
#include "Vec.h"
#include <cmath>

namespace QRandSequence {

	template<int B>
	class HaltonCore {
	private:
	public:
		static float Sample(int n) {
			float result = 0.0f;
		
			if (n <= 0) {
				return result;
			}

			int keta = std::log2(n) / std::log2(B) + 1;
			for (int i = 0; i < keta; ++i) {
				result += (n % B) * std::pow(B, -(i + 1));
				n /= B;
			}

			return result;
		}
	};

	class Halton {
	private:

	public:
		explicit Halton() {}
		PTUtility::Vec3 Sample(int i) {
			return PTUtility::Vec3(HaltonCore<2>::Sample(i), HaltonCore<3>::Sample(i), HaltonCore<5>::Sample(i));
		}
	};

	class Hammersley {
	private:
		int m_NumSequence;

	public:
		explicit Hammersley(int NumSample) : m_NumSequence(NumSample){}
		PTUtility::Vec3 Sample(int i) {
			return PTUtility::Vec3(i / (float)m_NumSequence, HaltonCore<2>::Sample(i), HaltonCore<3>::Sample(i));
		}
	};

}
