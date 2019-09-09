#include "Quaternion.h"
#include "Vec.h"

namespace PTUtility {


	Quaternion operator*(const Quaternion& q, float v) {
		return Quaternion(q[0] * v, q[1] * v, q[2] * v, q[3] * v);
	}
	Quaternion operator/(const Quaternion& q, float v) {
		return Quaternion(q[0] / v, q[1] / v, q[2] / v, q[3] / v);
	}
	Quaternion RotationQuaternion(const PTUtility::Vec3& axis, float radian) {
		return RotationQuaternion(axis, cosf(radian * 0.5f), sinf(radian * 0.5f));
	}
	Quaternion RotationQuaternion(const PTUtility::Vec3& axis, float cos, float sin) {
		return Quaternion(cos, axis[0] * sin, axis[1] * sin, axis[2] * sin);
	}

	const Quaternion Quaternion::inv() const {
		return this->star() / (this->norm() * this->norm());
	}
	const Quaternion Quaternion::inv() {
		return ((const Quaternion)(*this)).inv();
	}

	void Quaternion::normalize() {
		*this = *this / this->norm();
	}

	Quaternion Quaternion::normalized() const {
		return *this / this->norm();
	}
	Quaternion Quaternion::normalized() {
		return (const Quaternion)* this / this->norm();
	}

}