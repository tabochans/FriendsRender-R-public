#pragma once
#include "MT.h"
#include "Vec.h"
#include "QRandSequence.h"

class iCameraSsmpler {
private:

	int m_ID;
	int m_NumSample;

public:

	virtual PTUtility::Vec3 Sample(int count) = 0;

	explicit iCameraSsmpler(int ID, int NumSample) {}
};

class UniformSample : public iCameraSsmpler {
private:
	RandomMT m_MT;

public:

	virtual PTUtility::Vec3 Sample(int count) {
		return PTUtility::Vec3(m_MT.genrand64_real1(), m_MT.genrand64_real1(), m_MT.genrand64_real1());
	}
	explicit UniformSample(int ID) : iCameraSsmpler(ID, 1), m_MT(ID){
		
	}
};

class HaltonSample : public iCameraSsmpler {
private:
	QRandSequence::Halton m_Halton;

public:
	virtual PTUtility::Vec3 Sample(int count) {
		return m_Halton.Sample(count);
	}
	explicit HaltonSample(int ID) : iCameraSsmpler(ID, 1), m_Halton() {
	}
};

class HammersleySample : public iCameraSsmpler {
private:
	QRandSequence::Hammersley m_Hammersley;

public:
	virtual PTUtility::Vec3 Sample(int count) {
		return m_Hammersley.Sample(count);
	}
	explicit HammersleySample(int ID, int NumSample) : iCameraSsmpler(ID, NumSample), m_Hammersley(NumSample) {
	}
};
