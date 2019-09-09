#pragma once
#include "Vec.h"
#include <array>

namespace PTUtility {

	class Matrix2;
	class Matrix3;
	class Matrix4;

	Matrix4 CreateMatrixFromColumns(const std::array<Vec4, 4>& colums);
	Matrix4 CreateMatrixFromRows(const std::array<Vec4, 4>& rows);
	float Determinant(const Matrix4& mat);

	Matrix3 CreateMatrixFromColumns(const std::array<Vec3, 3>& colums);
	Matrix3 CreateMatrixFromRows(const std::array<Vec3, 3>& rows);
	float Determinant(const Matrix3& mat);

	Matrix2 CreateMatrixFromColumns(const std::array<Vec2, 2>& colums);
	Matrix2 CreateMatrixFromRows(const std::array<Vec2, 2>& rows);
	float Determinant(const Matrix2& mat);

	class Matrix2 {
	private:
		std::array<Vec2, 2> a;

	public:

		const Vec2 operator[](size_t i) const { return a[i]; }
		Vec2& operator[](size_t i) { return a[i]; }

		explicit Matrix2(const std::array<float, 4>& _a) : a({
			Vec2(_a[0], _a[1]),
			Vec2(_a[2], _a[3])}) {}
		explicit Matrix2() : a({Vec2(), Vec2()}) {}
		Matrix2(const Matrix2& r) : a{ r[0], r[1] } {}

		Matrix2& operator=(const Matrix2& r) {
			a[0] = r[0]; a[1] = r[1];
			return *this;
		}

		Matrix2 T() const {
			return Matrix2({
				a[0][0], a[1][0],
				a[0][1], a[1][1]}
			);
		}
		Matrix2 inv() const {
			return Matrix2({
				a[1][1], -a[0][1], 
				a[1][0], a[0][0]
				}) / Determinant(*this);
		}

		float Cofactor(int row, int column) const{
			return a[(row + 1) % 2][(column + 1) % 2] * ((row + column) % 2 ? -1 : 1);
		}

		Matrix2 CofactorMatrix() const {
			return Matrix2({ Cofactor(0, 0), Cofactor(0, 1),Cofactor(1, 0),Cofactor(1, 1) }).T();
		}


		Matrix2 operator-() const {
			return Matrix2({
				-a[0][0], -a[0][1],
				-a[1][0], -a[1][1]}
			);
		}
		Matrix2 operator+(const Matrix2 & r) const {
			Vec2 v[3] = { a[0] + r[0], a[1] + r[1], a[2] + r[2] };
			return Matrix2({
				v[0][0], v[0][1],
				v[1][0], v[1][1]}
			);
		}
		Matrix2 operator+(const Matrix2 & r) {
			return (const Matrix2)(*this) + r;
		}

		Matrix2 operator-(const Matrix2 & r) const {
			return *this - r;
		}
		Matrix2 operator-(const Matrix2 & r) {
			return (const Matrix2)(*this) - r;
		}

		Matrix2 operator*(const Matrix2 & r) const {
			Matrix2 result;
			const Matrix2 tR = r.T();
			for (int i = 0; i < 2; ++i) {
				for (int j = 0; j < 2; ++j) {
					result[i][j] = a[i].dot(tR[j]);
				}
			}
			return result;
		}
		Matrix2 operator*(const Matrix2 & r) {
			return (const Matrix2)(*this)* r;
		}

		Matrix2& operator+=(const Matrix2 & r) {
			*this = *this + r;
			return *this;
		}

		Matrix2& operator-=(const Matrix2 & r) {
			*this = *this - r;
			return *this;
		}

		Matrix2& operator*=(const Matrix2 & r) {
			*this = *this * r;
			return *this;
		}

		Matrix2 operator*(float x) const {
			return Matrix2({
				x * a[0][0], x * a[0][1],
				x * a[1][0], x * a[1][1]}
			);
		}
		Matrix2 operator*(float x) {
			return (const Matrix2)(*this)* x;
		}

		Matrix2 operator/(float x) const {
			const float ix = 1.0f / x;
			return (const Matrix2)(*this)* ix;
		}
		Matrix2 operator/(float x) {
			return (const Matrix2)(*this) / x;
		}

		Vec2 operator*(const Vec2& v) const {
			return Vec2(a[0].dot(v), a[1].dot(v));
		}
		Vec2 operator*(const Vec2& v) {
			return (const Matrix2)(*this)* v;
		}

		static const Matrix2 E() {
			return Matrix2({1, 0, 0, 1});
		}
		static const Matrix2 O() {
			return Matrix2();
		}
	};

	class Matrix3 {
	private:
		std::array<Vec3, 3> a;

	public:

		const Vec3 operator[](size_t i) const { return a[i]; }
		Vec3& operator[](size_t i) { return a[i]; }

		explicit Matrix3(const std::array<float, 9>& _a) : a({
			Vec3(_a[0], _a[1],_a[2]),
			Vec3(_a[3], _a[4],_a[5]),
			Vec3(_a[6], _a[7],_a[8]) }) {}
		explicit Matrix3() : a({Vec3(), Vec3(), Vec3()}) {}
		Matrix3(const Matrix3& r) : a{ r[0], r[1], r[2] } {}

		Matrix3& operator=(const Matrix3& r) {
			a[0] = r[0]; a[1] = r[1]; a[2] = r[2];
			return *this;
		}

		Matrix3 T() const {
			return Matrix3({
				a[0][0], a[1][0], a[2][0],
				a[0][1], a[1][1], a[2][1],
				a[0][2], a[1][2], a[2][2] }
			);
		}

		Matrix3 inv() const {
			Matrix3 mat = CofactorMatrix();
			float det =Determinant(*this);
			return mat / det;
		}


		float Cofactor(int row, int column) const {

			Matrix2 mat;

			for (int i = 0, r = 0; i < 3; ++i) {
				if (i == row) { continue; }

				for (int j = 0, c = 0; j < 3; ++j) {
					if (j == column) { continue; }

					mat[r][c] = a[i][j];
					++c;
				}
				++r;
			}
			
			return Determinant(mat)* ((row + column) % 2 ? -1 : 1);
		}

		Matrix3 CofactorMatrix() const {
			return Matrix3({
				Cofactor(0, 0),  Cofactor(0, 1),Cofactor(0, 2),
				Cofactor(1, 0),  Cofactor(1, 1),Cofactor(1, 2),
				Cofactor(2, 0),  Cofactor(2, 1),Cofactor(2, 2) }).T();
		}



		Matrix3 operator-() const {
			return Matrix3({
				-a[0][0], -a[0][1], -a[0][2],
				-a[1][0], -a[1][1], -a[1][2],
				-a[2][0], -a[2][1], -a[2][2]}
			);
		}
		Matrix3 operator+(const Matrix3 & r) const {
			Vec3 v[3] = { a[0] + r[0], a[1] + r[1], a[2] + r[2]};
			return Matrix3({
				v[0][0], v[0][1], v[0][2],
				v[1][0], v[1][1], v[1][2],
				v[2][0], v[2][1], v[2][2]}
			);
		}
		Matrix3 operator+(const Matrix3 & r) {
			return (const Matrix3)(*this) + r;
		}

		Matrix3 operator-(const Matrix3 & r) const {
			return *this - r;
		}
		Matrix3 operator-(const Matrix3 & r) {
			return (const Matrix3)(*this) - r;
		}

		Matrix3 operator*(const Matrix3 & r) const {
			Matrix3 result;
			const Matrix3 tR = r.T();
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					result[i][j] = a[i].dot(tR[j]);
				}
			}
			return result;
		}
		Matrix3 operator*(const Matrix3 & r) {
			return (const Matrix3)(*this)* r;
		}

		Matrix3& operator+=(const Matrix3 & r) {
			*this = *this + r;
			return *this;
		}

		Matrix3& operator-=(const Matrix3 & r) {
			*this = *this - r;
			return *this;
		}

		Matrix3& operator*=(const Matrix3 & r) {
			*this = *this * r;
			return *this;
		}

		Matrix3 operator*(float x) const {
			return Matrix3({
				x * a[0][0], x * a[0][1], x * a[0][2],
				x * a[1][0], x * a[1][1], x * a[1][2],
				x * a[2][0], x * a[2][1], x * a[2][2]}
			);
		}
		Matrix3 operator*(float x) {
			return (const Matrix3)(*this)* x;
		}

		Matrix3 operator/(float x) const {
			const float ix = 1.0f / x;
			return (const Matrix3)(*this)* ix;
		}
		Matrix3 operator/(float x) {
			return (const Matrix3)(*this) / x;
		}

		Vec3 operator*(const Vec3& v) const {
			return Vec3(a[0].dot(v), a[1].dot(v), a[2].dot(v));
		}
		Vec3 operator*(const Vec3& v) {
			return (const Matrix3)(*this)* v;
		}

		static const Matrix3 E() {
			return Matrix3({
				1, 0, 0,
				0, 1, 0,
				0, 0, 1 });
		}
		static const Matrix3 O() {
			return Matrix3();
		}
	};

	class Matrix4 {
	private:
		std::array<Vec4, 4> a;

	public:

		const Vec4 operator[](size_t i) const { return a[i]; }
		Vec4& operator[](size_t i) { return a[i]; }

		explicit Matrix4(const std::array<float, 16>& _a) : a({
			Vec4(_a[0], _a[1],_a[2],_a[3]), 
			Vec4(_a[4], _a[5],_a[6],_a[7]), 
			Vec4(_a[8], _a[9],_a[10],_a[11]), 
			Vec4(_a[12], _a[13],_a[14],_a[15])}) {}
		explicit Matrix4() : a({Vec4(),Vec4(),Vec4(),Vec4()}) {}
		Matrix4(const Matrix4& r) : a{r[0], r[1], r[2], r[3]} {}
		Matrix4(const Matrix2& m00, const Matrix2& m01, const Matrix2& m10, const Matrix2& m11);

		Matrix4& operator=(const Matrix4& r) { 
			a[0] = r[0]; a[1] = r[1]; a[2] = r[2]; a[3] = r[3];
			return *this;
		}

		Matrix4 T() const {
			return Matrix4({
				a[0][0], a[1][0], a[2][0], a[3][0],
				a[0][1], a[1][1], a[2][1], a[3][1],
				a[0][2], a[1][2], a[2][2], a[3][2],
				a[0][3], a[1][3], a[2][3], a[3][3] }
			);
		}

		Matrix4 inv() const {
			return CofactorMatrix() / Determinant(*this);
		}


		float Cofactor(int row, int column) const {
			Matrix3 mat;

			for (int i = 0, r = 0; i < 4; ++i) {
				if (i == row) { continue; }

				for (int j = 0, c = 0; j < 4; ++j) {
					if (j == column) { continue; }

					mat[r][c] = a[i][j];
					++c;
				}
				++r;
			}
			return Determinant(mat) * ((row + column) % 2 ? -1 : 1);
		}

		Matrix4 CofactorMatrix() const {
			return Matrix4({
				Cofactor(0, 0),  Cofactor(0, 1),Cofactor(0, 2), Cofactor(0, 3),
				Cofactor(1, 0),  Cofactor(1, 1),Cofactor(1, 2), Cofactor(1, 3),
				Cofactor(2, 0),  Cofactor(2, 1),Cofactor(2, 2), Cofactor(2, 3),
				Cofactor(3, 0),  Cofactor(3, 1),Cofactor(3, 2), Cofactor(3, 3) }).T();
		}

		Matrix4 operator-() const {
			return Matrix4({
				-a[0][0], -a[0][1], -a[0][2], -a[0][3],
				-a[1][0], -a[1][1], -a[1][2], -a[1][3],
				-a[2][0], -a[2][1], -a[2][2], -a[2][3],
				-a[3][0], -a[3][1], -a[3][2], -a[3][3] }
			);
		}
		Matrix4 operator+(const Matrix4& r) const {
			Vec4 v[4] = { a[0] + r[0], a[1] + r[1], a[2] + r[2], a[3] + r[3] };
			return Matrix4({
				v[0][0], v[0][1], v[0][2], v[0][3],
				v[1][0], v[1][1], v[1][2], v[1][3],
				v[2][0], v[2][1], v[2][2], v[2][3],
				v[3][0], v[3][1], v[3][2], v[3][3] }
			);
		}
		Matrix4 operator+(const Matrix4& r) {
			return (const Matrix4)(*this) + r;
		}

		Matrix4 operator-(const Matrix4& r) const {
			return *this - r;
		}
		Matrix4 operator-(const Matrix4& r) {
			return (const Matrix4)(*this) - r;
		}

		Matrix4 operator*(const Matrix4& r) const {
			Matrix4 result;
			const Matrix4 tR = r.T();
			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 4; ++j) {
					result[i][j] = a[i].dot(tR[j]);
				}
			}
			return result;
		}
		Matrix4 operator*(const Matrix4& r) {
			return (const Matrix4)(*this)* r;
		}
		
		Matrix4& operator+=(const Matrix4& r) {
			*this = *this + r;
			return *this;
		}

		Matrix4& operator-=(const Matrix4& r) {
			*this = *this - r;
			return *this;
		}

		Matrix4& operator*=(const Matrix4& r) {
			*this = *this * r;
			return *this;
		}

		Matrix4 operator*(float x) const {
			return Matrix4({
				x * a[0][0], x * a[0][1], x * a[0][2], x * a[0][3],
				x * a[1][0], x * a[1][1], x * a[1][2], x * a[1][3],
				x * a[2][0], x * a[2][1], x * a[2][2], x * a[2][3],
				x * a[3][0], x * a[3][1], x * a[3][2], x * a[3][3] }
			);
		}
		Matrix4 operator*(float x) {
			return (const Matrix4)(*this)* x;
		}

		Matrix4 operator/(float x) const {
			const float ix = 1.0f / x;
			return (const Matrix4)(*this) * ix;
		}
		Matrix4 operator/(float x) {
			return (const Matrix4)(*this) / x;
		}

		Vec4 operator*(const Vec4& v) const{
			return Vec4(a[0].dot(v), a[1].dot(v), a[2].dot(v), a[3].dot(v));
		}
		Vec4 operator*(const Vec4& v) {
			return (const Matrix4)(*this)* v;
		}

		static const Matrix4 E() {
			return Matrix4({
				1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1 });
		}
		static const Matrix4 O() {
			return Matrix4();
		}
	};

}