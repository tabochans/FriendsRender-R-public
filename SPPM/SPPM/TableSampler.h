#pragma once
#include <utility>
#include <deque>
#include <memory>
#include <ostream>
#include <functional>
#include "MT.h"

template<typename T>
class TableSampler {

private:

	struct Bin {
		float probLeftSample;
		int ID_Left;
		int ID_Right;
		Bin(float p, int IDLeft, int IDRight) : probLeftSample(p), ID_Left(IDLeft), ID_Right(IDRight){}
		Bin() : probLeftSample(1), ID_Left(-1), ID_Right(-1) {}
	};
	std::unique_ptr<Bin[]> m_Table;
	std::unique_ptr<T[]> m_Data;
	int m_NumElements;
	float m_Average;
	std::function<float(const T&)> m_evaluateFunction;

public:

	const T* GetRawData() const {
		return m_Data.get();
	}
	int GetNumElements() const {
		return m_NumElements;
	}

	void PrintTable(std::ostream& ost) const{
		ost << "Table : " << std::endl;
		for (int i = 0; i < m_NumElements; ++i) {
			Bin& b = m_Table[i];
			ost << "Left<" << b.ID_Left << "," << b.probLeftSample << ">  " << "Right<" << b.ID_Right << "," << 1.0f - b.probLeftSample << ">  " << std::endl;
		}
	}

	void Init(const T* data, int NumElements, std::function<float(const T&)> f) {

		if (NumElements <= 0) {
			return;
		}

		m_NumElements = NumElements;
		m_Table.reset(new Bin[m_NumElements]);
		m_Data.reset(new T[m_NumElements]);
		m_evaluateFunction = f;
		m_Average = 0.0f;

		// calculate average
		for (int i = 0; i < m_NumElements; ++i) {
			m_Data[i] = data[i];
			m_Average += std::max(0.0f, f(m_Data[i]));
		}
		m_Average /= m_NumElements;

		// separate into two groups
		std::deque<std::pair<float, int>> bigger, smaller;
		for (int i = 0; i < m_NumElements; ++i) {
			float value = std::max(0.0f, f(m_Data[i]));

			if (value >= m_Average) {
				bigger.push_back(std::pair<float, int>(value / m_Average, i));
			}
			else {
				smaller.push_back(std::pair<float, int>(value / m_Average, i));
			}
		}

		int ID_S = 0;
		int ID_B = 0;
		for (int i = 0; i < m_NumElements; ++i) {
			if (ID_S < smaller.size()) {
				// if smaller elements exist, table is filled with one of those elements. 

				m_Table[i].ID_Left = smaller[ID_S].second;
				m_Table[i].probLeftSample = smaller[ID_S].first;
				ID_S++;

				if (ID_B < bigger.size()) {
					// the rest is filled with bigger element. 

					m_Table[i].ID_Right = bigger[ID_B].second;

					bigger[ID_B].first -= (1.0f - m_Table[i].probLeftSample);
					if (bigger[ID_B].first < 1.0f) {
						smaller.push_back(bigger[ID_B++]);
					}
				}
				else {
					// when only one smaller element is remained. 

					if (smaller.size() == ID_S) {
						m_Table[i].ID_Right = -1;
						m_Table[i].probLeftSample = 1.0f;
					}
					else {
						m_Table[i].ID_Right = smaller[ID_S].second;
					}
				}
			}
			else {
				// only bigger elements are remained. 

				m_Table[i].ID_Left = bigger[ID_B].second;
				m_Table[i].probLeftSample = 1.0f;

				bigger[ID_B].first -= 1.0f;
				if (bigger[ID_B].first < 1.0f) {
					smaller.push_back(bigger[ID_B++]);
				}
			}
		}
	}
	
	float GetPDF(const T& t) const {
		return m_evaluateFunction(t) / m_Average / (float)m_NumElements;
	}

	const T& Sample(RandomMT& MT) const {
		const Bin& bin = m_Table[m_NumElements * MT.genrand64_real2()];
		return (MT.genrand64_real1() < bin.probLeftSample) ? m_Data[bin.ID_Left] : m_Data[bin.ID_Right];
	}

	const T& Sample(const PTUtility::Vec2& randNumber) const {
		const Bin& bin = m_Table[m_NumElements * randNumber[0]];
		return (randNumber[1] < bin.probLeftSample) ? m_Data[bin.ID_Left] : m_Data[bin.ID_Right];
	}

	TableSampler(const TableSampler& ref) : 
		m_NumElements(ref.m_NumElements), m_Average(ref.m_Average), m_evaluateFunction(ref.m_evaluateFunction){

		if (ref.m_NumElements > 0) {
			m_Table = std::make_unique<Bin[]>(ref.m_NumElements);
			m_Data = std::make_unique<T[]>(ref.m_NumElements);
			for (int i = 0; i < ref.m_NumElements; ++i) {
				m_Data[i] = ref.m_Data[i];
				m_Table[i] = ref.m_Table[i];
			}
		}
	}
	TableSampler& operator=(const TableSampler& ref) {
		m_NumElements = ref.m_NumElements;
		m_Average = ref.m_Average;
		m_evaluateFunction = ref.m_evaluateFunction;
		
		if (ref.m_NumElements > 0) {
			m_Table = std::make_unique<Bin[]>(ref.m_NumElements);
			m_Data = std::make_unique<T[]>(ref.m_NumElements);
			for (int i = 0; i < ref.m_NumElements; ++i) {
				m_Data[i] = ref.m_Data[i];
				m_Table[i] = ref.m_Table[i];
			}
		}
		return *this;
	}

	TableSampler() : m_NumElements(0), m_Average(0){
	}

	~TableSampler() {
	}
};
