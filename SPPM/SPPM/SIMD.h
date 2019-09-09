#pragma once
#include <iostream>
#include <immintrin.h>
#include <array>

namespace mSIMD {

	class Double4 {
	private:
		__m256d Data;

	public:

		inline const Double4 operator+(const Double4& ref) const {
			return _mm256_add_pd(Data, ref.Data);
		}
		inline Double4& operator+=(const Double4& ref) {
			*this = *this + ref;
			return *this;
		}
		inline const Double4 operator-(const Double4& ref) const {
			return _mm256_sub_pd(Data, ref.Data);
		}
		inline Double4& operator-=(const Double4& ref) {
			*this = *this - ref;
			return *this;
		}
		inline const Double4 operator*(const Double4& ref) const {
			return _mm256_mul_pd(Data, ref.Data);
		}
		inline Double4& operator*=(const Double4& ref) {
			*this = *this * ref;
			return *this;
		}
		inline const Double4 operator/(const Double4& ref) const {
			return _mm256_div_pd(Data, ref.Data);
		}
		inline Double4& operator/=(const Double4& ref) {
			*this = *this / ref;
			return *this;
		}
		inline Double4 operator-() const {
			return Double4(0.0) - *this;
		}
		inline Double4 operator+() const {
			return *this;
		}
		Double4& operator=(const Double4& ref) {
			this->Data = ref.Data;
			return *this;
		}
		inline double operator[](size_t id) const {
			const double* pd;
			pd = reinterpret_cast<const double*>(&Data);
			return pd[id];
		}

		inline uint32_t operator>(const Double4& ref) const{
			return _mm256_movemask_pd(_mm256_cmp_pd(this->Data, ref.Data, _CMP_GT_OS));
		}
		inline uint32_t operator<(const Double4& ref) const {
			return _mm256_movemask_pd(_mm256_cmp_pd(this->Data, ref.Data, _CMP_LT_OS));
		}
		static bool GetResult_i(uint32_t result, int i) {
			return result & (1 << i);
		}
		static Double4 max(const Double4& a, const Double4& b) {
			return _mm256_max_pd(a.Data, b.Data);
		}
		static Double4 min(const Double4& a, const Double4& b) {
			return _mm256_min_pd(a.Data, b.Data);
		}
		
		Double4(__m256d data) : Data(data){
		}
		//Double4(const std::array<double, 4>& data) { Data = _mm256_setr_pd(data[0], data[1], data[2], data[3]); }
		Double4() {
			Data = _mm256_set1_pd(0.0);
		}
		Double4(double d) {
			Data = _mm256_set1_pd(d);
		}

		void PrintDouble4(std::ostream& ost) const {
			ost << (*this)[0] << " " << (*this)[1] << " " << (*this)[2] << " " << (*this)[3] << std::endl;
		}
	};

	class Float8 {
	private:
		__m256 Data;

	public:

		inline const Float8 operator+(const Float8& ref) const {
			return _mm256_add_ps(Data, ref.Data);
		}
		inline Float8& operator+=(const Float8& ref) {
			*this = *this + ref;
			return *this;
		}
		inline const Float8 operator-(const Float8& ref) const {
			return _mm256_sub_ps(Data, ref.Data);
		}
		inline Float8& operator-=(const Float8& ref) {
			*this = *this - ref;
			return *this;
		}
		inline const Float8 operator*(const Float8& ref) const {
			return _mm256_mul_ps(Data, ref.Data);
		}
		inline Float8& operator*=(const Float8& ref) {
			*this = *this * ref;
			return *this;
		}
		inline const Float8 operator/(const Float8& ref) const {
			return _mm256_div_ps(Data, ref.Data);
		}
		inline Float8& operator/=(const Float8& ref) {
			*this = *this / ref;
			return *this;
		}
		inline Float8 operator-() const {
			return Float8(0.0) - *this;
		}
		inline Float8 operator+() const {
			return *this;
		}
		Float8& operator=(const Float8& ref) {
			this->Data = ref.Data;
			return *this;
		}
		inline double operator[](size_t id) const {
			const float* pd;
			pd = reinterpret_cast<const float*>(&Data);
			return pd[id];
		}

		inline uint32_t operator>(const Float8& ref) const {
			return _mm256_movemask_ps(_mm256_cmp_ps(this->Data, ref.Data, _CMP_GT_OS));
		}
		inline uint32_t operator<(const Float8& ref) const {
			return _mm256_movemask_ps(_mm256_cmp_ps(this->Data, ref.Data, _CMP_LT_OS));
		}
		static bool GetResult_i(uint32_t result, int i) {
			return result & (1 << i);
		}
		static Float8 max(const Float8& a, const Float8& b) {
			return _mm256_max_ps(a.Data, b.Data);
		}
		static Float8 min(const Float8& a, const Float8& b) {
			return _mm256_min_ps(a.Data, b.Data);
		}

		Float8(__m256 data) : Data(data) {
		}
		//Float8(const std::array<float, 8> & data) { Data = _mm256_setr_ps(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]); }
		Float8() {
			Data = _mm256_set1_ps(0.0);
		}
		Float8(float d) {
			Data = _mm256_set1_ps(d);
		}

		void PrintFloat8(std::ostream& ost) const {
			ost << (*this)[0] << " " << (*this)[1] << " " << (*this)[2] << " " << (*this)[3] << " " 
				<< (*this)[4] << " " << (*this)[5] << " " << (*this)[6] << " " << (*this)[7] << std::endl;
		}
	};

};
