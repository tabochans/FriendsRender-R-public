#include "Vec.h"

namespace PTUtility {

	const Vec2 Vec3::xy() const { return Vec2(x(), y()); }
	const Vec2 Vec3::yz() const { return Vec2(y(), z()); }
	const Vec2 Vec3::xz() const { return Vec2(x(), z()); }

	Vec3::Vec3(const Vec2& v, float z) : a{v[0], v[1], z} {}
	Vec3::Vec3(float x, const Vec2& v) : a{ x, v[0], v[1]} {}

	const Vec3 Vec4::xyz() const { return Vec3(x(), y(), z()); }
	const Vec3 Vec4::xyw() const{ return Vec3(x(), y(), w()); }
	const Vec3 Vec4::yzw() const{ return Vec3(y(), z(), w()); }
	const Vec3 Vec4::xzw() const{ return Vec3(x(), z(), z()); }
	const Vec2 Vec4::xy() const{ return Vec2(x(), y()); }
	const Vec2 Vec4::xz() const{ return Vec2(x(), z()); }
	const Vec2 Vec4::xw() const{ return Vec2(x(), w()); }
	const Vec2 Vec4::yz() const{ return Vec2(y(), z()); }
	const Vec2 Vec4::yw() const{ return Vec2(y(), w()); }
	const Vec2 Vec4::zw() const{ return Vec2(z(), w()); }

	Vec4::Vec4(const Vec3& v, float w) : a{v[0], v[1], v[2], w} {}
	Vec4::Vec4(float x, const Vec3& v) : a{ x, v[0], v[1], v[2]} {}
	Vec4::Vec4(const Vec2& v1, const Vec2& v2) : a{v1[0], v1[1], v2[0], v2[1]} {}
	Vec4::Vec4(const Vec2& v, float z, float w) : a{ v[0], v[1], z,w } {}
	Vec4::Vec4(float x, const Vec2& v, float w) : a{x, v[0], v[1], w} {}
	Vec4::Vec4(float x, float y, const Vec2& v) : a{x, y, v[0], v[1]} {}

};