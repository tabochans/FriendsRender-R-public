#pragma once
#include<cmath>

namespace PTUtility {

	class Vec2;
	class Vec3;
	class Vec4;

	constexpr float PI = 3.14159;

	class Vec2 {
	private:
		float a[2];

	public:
		static Vec2 Ones() {
			return Vec2(1, 1);
		}
		static Vec2 Zero() {
			return Vec2(0, 0);
		}

		inline const Vec2 operator+(const Vec2& ref)const {
			return Vec2(a[0] + ref.x(), a[1] + ref.y());
		}
		inline Vec2& operator+=(const Vec2 & ref) {
			a[0] += ref.x();
			a[1] += ref.y();
			return *this;
		}

		inline const Vec2 operator-(const Vec2 & ref)const {
			return Vec2(a[0] - ref.x(), a[1] - ref.y());
		}
		inline Vec2& operator-=(const Vec2 & ref) {
			a[0] -= ref.x();
			a[1] -= ref.y();
			return *this;
		}

		inline Vec2& operator*=(const Vec2 & ref) {
			a[0] *= ref.x();
			a[1] *= ref.y();
			return *this;
		}
		inline Vec2 operator*(const Vec2 & ref) const {
			return PTUtility::Vec2(a[0] * ref.x(), a[1] * ref.y());
		}

		inline Vec2& operator/=(const Vec2 & ref) {
			a[0] /= ref.x();
			a[1] /= ref.y();
			return *this;
		}
		inline Vec2 operator/(const Vec2 & ref) const {
			return PTUtility::Vec2(a[0] / ref.x(), a[1] / ref.y());
		}

		inline const Vec2 operator-()const {
			return Vec2(-a[0], -a[1]);
		}
		inline const Vec2 operator+()const {
			return *this;
		}

		Vec2& operator=(const Vec2 & ref) {
			a[0] = ref.x();
			a[1] = ref.y();
			return *this;
		}

		//Other operator
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		float dot(const Vec2 & ref)const {
			return a[0] * ref.x() + a[1] * ref.y();
		}
		float norm()const {
			return sqrt(a[0] * a[0] + a[1] * a[1]);
		}
		float norm2()const {
			return a[0] * a[0] + a[1] * a[1];
		}

		void normalize() {
			float d = norm();
			a[0] /= d;
			a[1] /= d;
		}
		Vec2 normalized()const {
			float d = norm();
			return Vec2(a[0] / d, a[1] / d);
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Geter 
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline float x()const {
			return a[0];
		}
		inline float y()const {
			return a[1];
		}
		inline float operator[](size_t id)const {
			return a[id];
		}
		inline float operator()(size_t id)const {
			return a[id];
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Seter
		inline float& x() {
			return a[0];
		}
		inline float& y() {
			return a[1];
		}
		inline float& operator[](size_t id) {
			return a[id];
		}
		inline float& operator()(size_t id) {
			return a[id];
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Copy Constructor
		Vec2(const Vec2 & ref) {
			a[0] = ref.x();
			a[1] = ref.y();
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Constructor
		explicit Vec2() {
			a[0] = a[1] = 0.0f;
		}
		explicit Vec2(float x, float y) {
			a[0] = x;
			a[1] = y;
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Destructor
		~Vec2() {}
	};

	inline const Vec2 operator*(float left, const Vec2 & right) {
		return Vec2(left * right.x(), left * right.y());
	}
	inline const Vec2 operator*(const Vec2 & left, float right) {
		return Vec2(right * left.x(), right * left.y());
	}
	inline const Vec2 operator*=(Vec2 & left, float right) {
		left.x() *= right;
		left.y() *= right;
		return left;
	}
	inline const Vec2 operator/=(Vec2 & left, float right) {
		left.x() /= right;
		left.y() /= right;
		return left;
	}
	inline const Vec2 operator/(const Vec2 & left, float right) {
		return Vec2(left.x() / right, left.y() / right);
	}


	class Vec3 {
	private:
		float a[3];

	public:
		inline static Vec3 Ones() {
			return Vec3(1, 1, 1);
		}
		inline static Vec3 Zero() {
			return Vec3(0, 0, 0);
		}
		inline const Vec3 operator+(const Vec3& ref) const {
			return Vec3(a[0] + ref.x(), a[1] + ref.y(), a[2] + ref.z());
		}
		inline Vec3& operator+=(const Vec3 & ref) {
			*this = *this + ref;
			return *this;
		}
		inline const Vec3 operator-(const Vec3 & ref) const {
			return Vec3(a[0] - ref.x(), a[1] - ref.y(), a[2] - ref.z());
		}
		inline Vec3& operator-=(const Vec3 & ref) {
			*this = *this - ref;
			return *this;
		}
		inline Vec3 operator*(const Vec3 & ref) const {
			return PTUtility::Vec3(a[0] * ref.x(), a[1] * ref.y(), a[2] * ref.z());
		}
		inline Vec3& operator*=(const Vec3 & ref) {
			*this = *this * ref;
			return *this;
		}
		inline Vec3 operator/(const Vec3 & ref) const {
			return PTUtility::Vec3(a[0] / ref.x(), a[1] / ref.y(), a[2] / ref.z());
		}
		inline Vec3& operator/=(const Vec3 & ref) {
			*this = *this / ref;
			return *this;
		}
		inline const Vec3 operator-() const {
			return Vec3(-a[0], -a[1], -a[2]);
		}
		inline const Vec3 operator+() const {
			return *this;
		}
		Vec3& operator=(const Vec3 & ref) {
			a[0] = ref.x();
			a[1] = ref.y();
			a[2] = ref.z();
			return *this;
		}

		//Other operator
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		float dot(const Vec3 & ref)const {
			return a[0] * ref.x() + a[1] * ref.y() + a[2] * ref.z();
		}
		Vec3 cross(const Vec3 & ref)const {
			return Vec3(
				a[1] * ref.z() - a[2] * ref.y(),
				a[2] * ref.x() - a[0] * ref.z(),
				a[0] * ref.y() - a[1] * ref.x());
		}
		float norm()const {
			return sqrt(norm2());
		}
		float norm2()const {
			return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
		}
		void normalize() {
			*this = this->normalized();
		}
		Vec3 normalized()const {
			float d = norm();
			return Vec3(a[0] / d, a[1] / d, a[2] / d);
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Geter 
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline float x()const {
			return a[0];
		}
		inline float y()const {
			return a[1];
		}
		inline float z()const {
			return a[2];
		}
		inline float operator[](size_t id)const {
			return a[id];
		}
		inline float operator()(size_t id)const {
			return a[id];
		}

		const Vec2 xy() const;
		const Vec2 yz() const;
		const Vec2 xz() const;

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Seter
		inline float& x() {
			return a[0];
		}
		inline float& y() {
			return a[1];
		}
		inline float& z() {
			return a[2];
		}
		inline float& operator[](size_t id) {
			return a[id];
		}
		inline float& operator()(size_t id) {
			return a[id];
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Copy Constructor
		Vec3(const Vec3 & ref) : a{ ref.x() ,ref.y(),ref.z() } {}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Constructor
		explicit Vec3() : a{ 0,0,0 } {}
		explicit Vec3(float x, float y, float z) : a{ x, y, z } {}
		explicit Vec3(const Vec2& v, float z);
		explicit Vec3(float x, const Vec2& v);
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Destructor
		~Vec3() {}
	};

	inline const Vec3 operator*(float left, const Vec3 & right) {
		return Vec3(left * right.x(), left * right.y(), left * right.z());
	}
	inline const Vec3 operator*(const Vec3 & left, float right) {
		return Vec3(right * left.x(), right * left.y(), right * left.z());
	}
	inline const Vec3 operator*=(Vec3 & left, float right) {
		left.x() *= right;
		left.y() *= right;
		left.z() *= right;
		return left;
	}
	inline const Vec3 operator/=(Vec3 & left, float right) {
		left.x() /= right;
		left.y() /= right;
		left.z() /= right;
		return left;
	}
	inline const Vec3 operator/(const Vec3 & left, float right) {
		return Vec3(left.x() / right, left.y() / right, left.z() / right);
	}


	class Vec4 {
	private:
		float a[4];

	public:
		inline static Vec4 Ones() {
			return Vec4(1, 1, 1, 1);
		}
		inline static Vec4 Zero() {
			return Vec4(0, 0, 0, 0);
		}
		inline const Vec4 operator+(const Vec4& ref) const {
			return Vec4(a[0] + ref.x(), a[1] + ref.y(), a[2] + ref.z(), a[3] + ref.w());
		}
		inline Vec4& operator+=(const Vec4 & ref) {
			*this = *this + ref;
			return *this;
		}
		inline const Vec4 operator-(const Vec4 & ref) const {
			return Vec4(a[0] - ref.x(), a[1] - ref.y(), a[2] - ref.z(), a[3] - ref.w());
		}
		inline Vec4& operator-=(const Vec4 & ref) {
			*this = *this - ref;
			return *this;
		}
		inline Vec4 operator*(const Vec4 & ref) const {
			return PTUtility::Vec4(a[0] * ref.x(), a[1] * ref.y(), a[2] * ref.z(), a[3] * ref.w());
		}
		inline Vec4& operator*=(const Vec4 & ref) {
			*this = *this * ref;
			return *this;
		}
		inline Vec4 operator/(const Vec4 & ref) const {
			return PTUtility::Vec4(a[0] / ref.x(), a[1] / ref.y(), a[2] / ref.z(), a[3] / ref.w());
		}
		inline Vec4& operator/=(const Vec4 & ref) {
			*this = *this / ref;
			return *this;
		}
		inline const Vec4 operator-() const {
			return Vec4(-a[0], -a[1], -a[2], -a[3]);
		}
		inline const Vec4 operator+() const {
			return *this;
		}
		Vec4& operator=(const Vec4 & ref) {
			a[0] = ref.x();
			a[1] = ref.y();
			a[2] = ref.z();
			a[3] = ref.w();
			return *this;
		}

		//Other operator
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		float dot(const Vec4 & ref)const {
			return a[0] * ref.x() + a[1] * ref.y() + a[2] * ref.z() + a[3] * ref.w();
		}
		float norm()const {
			return sqrt(norm2());
		}
		float norm2()const {
			return a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3];
		}
		void normalize() {
			*this = this->normalized();
		}
		Vec4 normalized()const {
			float d = norm();
			return Vec4(a[0] / d, a[1] / d, a[2] / d, a[3] / d);
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Geter 
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline float x()const {
			return a[0];
		}
		inline float y()const {
			return a[1];
		}
		inline float z()const {
			return a[2];
		}
		inline float w()const {
			return a[3];
		}
		inline float operator[](size_t id)const {
			return a[id];
		}
		inline float operator()(size_t id)const {
			return a[id];
		}

		const Vec3 xyz() const;
		const Vec3 xyw() const;
		const Vec3 yzw() const;
		const Vec3 xzw() const;

		const Vec2 xy() const;
		const Vec2 xz() const;
		const Vec2 xw() const;
		const Vec2 yz() const;
		const Vec2 yw() const;
		const Vec2 zw() const;


		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Seter
		inline float& x() {
			return a[0];
		}
		inline float& y() {
			return a[1];
		}
		inline float& z() {
			return a[2];
		}
		inline float& w() {
			return a[3];
		}
		inline float& operator[](size_t id) {
			return a[id];
		}
		inline float& operator()(size_t id) {
			return a[id];
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Copy Constructor
		Vec4(const Vec4 & ref) : a{ ref.x() ,ref.y(),ref.z(), ref.w() } {}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Constructor
		explicit Vec4() : a{ 0,0,0,0 } {}
		explicit Vec4(float x, float y, float z, float w) : a{ x, y, z, w } {}
		explicit Vec4(const Vec3& v, float w);
		explicit Vec4(float x, const Vec3& v);
		explicit Vec4(const Vec2& v1, const Vec2& v2);
		explicit Vec4(const Vec2& v, float z, float w);
		explicit Vec4(float x, const Vec2& v, float w);
		explicit Vec4(float x, float y, const Vec2& v);
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Destructor
		~Vec4() {}
	};

	inline const Vec4 operator*(float left, const Vec4 & right) {
		return Vec4(left * right.x(), left * right.y(), left * right.z(), left * right.w());
	}
	inline const Vec4 operator*(const Vec4 & left, float right) {
		return Vec4(right * left.x(), right * left.y(), right * left.z(), right * left.w());
	}
	inline const Vec4 operator*=(Vec4 & left, float right) {
		left.x() *= right;
		left.y() *= right;
		left.z() *= right;
		left.w() *= right;
		return left;
	}
	inline const Vec4 operator/=(Vec4 & left, float right) {
		left.x() /= right;
		left.y() /= right;
		left.z() /= right;
		left.w() /= right;
		return left;
	}
	inline const Vec4 operator/(const Vec4 & left, float right) {
		return Vec4(left.x() / right, left.y() / right, left.z() / right, left.w() / right);
	}
}