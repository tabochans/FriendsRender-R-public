#include "Matrix.h"


namespace PTUtility {

	Matrix4 CreateMatrixFromColumns(const std::array<Vec4, 4>& colums) {
		return CreateMatrixFromRows(colums).T();
	}
	Matrix4 CreateMatrixFromRows(const std::array<Vec4, 4>& rows) {
		return Matrix4({
			rows[0][0], rows[0][1], rows[0][2], rows[0][3],
			rows[1][0], rows[1][1], rows[1][2], rows[1][3],
			rows[2][0], rows[2][1], rows[2][2], rows[2][3],
			rows[3][0], rows[3][1], rows[3][2], rows[3][3] }
		);
	}
	float Determinant(const Matrix4& mat) {
		return mat[0][0] * mat.Cofactor(0, 0) + mat[1][0] * mat.Cofactor(1, 0) + mat[2][0] * mat.Cofactor(2, 0) + mat[3][0] * mat.Cofactor(3, 0);
	}


	Matrix3 CreateMatrixFromColumns(const std::array<Vec3, 3>& colums) {
		return CreateMatrixFromRows(colums).T();
	}
	Matrix3 CreateMatrixFromRows(const std::array<Vec3, 3>& rows) {
		return Matrix3({
			rows[0][0], rows[0][1], rows[0][2],
			rows[1][0], rows[1][1], rows[1][2],
			rows[2][0], rows[2][1], rows[2][2] }
		);
	}
	float Determinant(const Matrix3& mat) {
		return mat[0][0] * mat.Cofactor(0, 0) + mat[1][0] * mat.Cofactor(1, 0) + mat[2][0] * mat.Cofactor(2, 0);
	}

	Matrix2 CreateMatrixFromColumns(const std::array<Vec2, 2>& colums) {
		return CreateMatrixFromRows(colums).T();
	}
	Matrix2 CreateMatrixFromRows(const std::array<Vec2, 2>& rows) {
		return Matrix2({
			rows[0][0], rows[0][1], 
			rows[1][0], rows[1][1] }
		);
	}
	float Determinant(const Matrix2& mat) {
		return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
	}


	Matrix4::Matrix4(const Matrix2& m00, const Matrix2& m01, const Matrix2& m10, const Matrix2& m11) : a{
		Vec4(m00[0][0],m00[0][1],m01[0][0],m01[0][1]),
		Vec4(m00[1][0],m00[1][1],m01[1][0],m01[1][1]),
		Vec4(m10[0][0],m10[0][1],m11[0][0],m11[0][1]),
		Vec4(m10[1][0],m10[1][1],m11[1][0],m11[1][1])} {
	}

};