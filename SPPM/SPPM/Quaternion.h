#pragma once
#include <cmath>
#include "Vec.h"

namespace PTUtility {

	class Quaternion {
	private:
		// a[0] + a[1]*i + a[2]*j + a[3]*k
		float a[4];

	public:
		Quaternion() : a{0,0,0,0} {}
		Quaternion(float x, float y, float z, float w) : a{ x, y, z, w } {}
		~Quaternion() {}

		Quaternion(const Quaternion& ref) : a{ref.a[0], ref.a[1], ref.a[2],ref.a[3] } {}
		Quaternion& operator=(const Quaternion& ref) {
			a[0] = ref.a[0]; a[1] = ref.a[1]; a[2] = ref.a[2]; a[3] = ref.a[3];
			return *this;
		}

		const Quaternion operator+(const Quaternion& ref) const {
			return Quaternion(a[0] + ref.a[0], a[1] + ref.a[1], a[2] + ref.a[2], a[3] + ref.a[3]);
		}
		const Quaternion operator+(const Quaternion& ref) {
			return (const Quaternion)(*this).operator+(ref);
		}
		const Quaternion operator-() const {
			return Quaternion(-a[0], -a[1], -a[2], -a[3]);
		}
		const Quaternion operator-() {
			return (const Quaternion)(*this).operator-();
		}
		const Quaternion operator-(const Quaternion& ref) const {
			return (*this) + (-ref);
		}
		const Quaternion operator-(const Quaternion& ref) {
			return (const Quaternion)(*this) + (-ref);
		}
		const Quaternion operator*(const Quaternion& ref) const {
			return Quaternion(
				a[0] * ref.a[0] - a[1] * ref.a[1] - a[2] * ref.a[2] - a[3] * ref.a[3], 
				a[0] * ref.a[1] + a[1] * ref.a[0] + a[2] * ref.a[3] - a[3] * ref.a[2],
				a[0] * ref.a[2] - a[1] * ref.a[3] + a[2] * ref.a[0] + a[3] * ref.a[1],
				a[0] * ref.a[3] + a[1] * ref.a[2] - a[2] * ref.a[1] + a[3] * ref.a[0]);
		}
		const Quaternion operator*(const Quaternion& ref) {
			return (const Quaternion)(*this)* ref;
		}

		float norm() const {
			return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
		}
		float norm() { return ((const Quaternion)(*this)).norm(); }

		const Quaternion star() const {
			return Quaternion(a[0], -a[1], -a[2], -a[3]);
		}
		const Quaternion star() {
			return ((const Quaternion)(*this)).star();
		}

		const Quaternion inv() const;
		const Quaternion inv();

		float operator[](int i) const {
			return a[i];
		}
		float& operator[](int i) {
			return a[i];
		}

		void normalize();
		Quaternion normalized() const;
		Quaternion normalized();
	};


	Quaternion operator*(const Quaternion& q, float v);
	Quaternion operator/(const Quaternion& q, float v);
	Quaternion RotationQuaternion(const PTUtility::Vec3& axis, float radian);
	Quaternion RotationQuaternion(const PTUtility::Vec3& axis, float cos, float sin);
};
