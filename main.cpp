#include <Novice.h>
#include <imgui.h>
#include <cmath>
#include <cstring>
#include <cstdint>

// 3次元ベクトルの構造体
struct Vector3 {
	float x;
	float y;
	float z;

	// 複合代入演算子
	Vector3& operator+=(const Vector3& v) {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	Vector3& operator-=(const Vector3& v) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	Vector3& operator*=(float s) {
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}

	Vector3& operator/=(float s) {
		if (s != 0.0f) {
			x /= s;
			y /= s;
			z /= s;
		}
		return *this;
	}
};

// 4x4行列の構造体
struct Matrix4x4 {
	float m[4][4];
};

// ベクトルの加算
Vector3 Add(const Vector3& v1, const Vector3& v2) {
	Vector3 result{};

	result.x = v1.x + v2.x;
	result.y = v1.y + v2.y;
	result.z = v1.z + v2.z;

	return result;
}

// ベクトルの減算
Vector3 Subtract(const Vector3& v1, const Vector3& v2) {
	Vector3 result{};

	result.x = v1.x - v2.x;
	result.y = v1.y - v2.y;
	result.z = v1.z - v2.z;

	return result;
}

// ベクトルのスカラー倍
Vector3 Multiply(float scalar, const Vector3& vector) {
	Vector3 result{};

	result.x = scalar * vector.x;
	result.y = scalar * vector.y;
	result.z = scalar * vector.z;

	return result;
}

// 行列の加算
Matrix4x4 Add(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result{};

	for (int32_t row = 0; row < 4; ++row) {
		for (int32_t column = 0; column < 4; ++column) {
			result.m[row][column] = m1.m[row][column] + m2.m[row][column];
		}
	}

	return result;
}

// 行列の減算
Matrix4x4 Subtract(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result{};

	for (int32_t row = 0; row < 4; ++row) {
		for (int32_t column = 0; column < 4; ++column) {
			result.m[row][column] = m1.m[row][column] - m2.m[row][column];
		}
	}

	return result;
}

// 行列の積
Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result{};

	for (int32_t row = 0; row < 4; ++row) {
		for (int32_t column = 0; column < 4; ++column) {
			for (int32_t element = 0; element < 4; ++element) {
				result.m[row][column] += m1.m[row][element] * m2.m[element][column];
			}
		}
	}

	return result;
}

// ベクトルの二項演算子
Vector3 operator+(const Vector3& v1, const Vector3& v2) {
	return Add(v1, v2);
}

Vector3 operator-(const Vector3& v1, const Vector3& v2) {
	return Subtract(v1, v2);
}

Vector3 operator*(float s, const Vector3& v) {
	return Multiply(s, v);
}

Vector3 operator*(const Vector3& v, float s) {
	return s * v;
}

Vector3 operator/(const Vector3& v, float s) {
	if (s == 0.0f) {
		return {};
	}

	return Multiply(1.0f / s, v);
}

// ベクトルの単項演算子
Vector3 operator-(const Vector3& v) {
	Vector3 result{};

	result.x = -v.x;
	result.y = -v.y;
	result.z = -v.z;

	return result;
}

Vector3 operator+(const Vector3& v) {
	return v;
}

// 行列の二項演算子
Matrix4x4 operator+(const Matrix4x4& m1, const Matrix4x4& m2) {
	return Add(m1, m2);
}

Matrix4x4 operator-(const Matrix4x4& m1, const Matrix4x4& m2) {
	return Subtract(m1, m2);
}

Matrix4x4 operator*(const Matrix4x4& m1, const Matrix4x4& m2) {
	return Multiply(m1, m2);
}

// x軸回転行列
Matrix4x4 MakeRotateXMatrix(float radian) {
	Matrix4x4 result{};

	float sin = std::sin(radian);
	float cos = std::cos(radian);

	result.m[0][0] = 1.0f;
	result.m[1][1] = cos;
	result.m[1][2] = sin;
	result.m[2][1] = -sin;
	result.m[2][2] = cos;
	result.m[3][3] = 1.0f;

	return result;
}

// y軸回転行列
Matrix4x4 MakeRotateYMatrix(float radian) {
	Matrix4x4 result{};

	float sin = std::sin(radian);
	float cos = std::cos(radian);

	result.m[0][0] = cos;
	result.m[0][2] = -sin;
	result.m[1][1] = 1.0f;
	result.m[2][0] = sin;
	result.m[2][2] = cos;
	result.m[3][3] = 1.0f;

	return result;
}

// z軸回転行列
Matrix4x4 MakeRotateZMatrix(float radian) {
	Matrix4x4 result{};

	float sin = std::sin(radian);
	float cos = std::cos(radian);

	result.m[0][0] = cos;
	result.m[0][1] = sin;
	result.m[1][0] = -sin;
	result.m[1][1] = cos;
	result.m[2][2] = 1.0f;
	result.m[3][3] = 1.0f;

	return result;
}

// ImGuiでVector3を表示する
void ImGuiTextVector3(const char* label, const Vector3& v) {
	ImGui::Text("%s:%f, %f, %f", label, v.x, v.y, v.z);
}

// ImGuiでMatrix4x4を表示する
void ImGuiTextMatrix4x4(const char* label, const Matrix4x4& matrix) {
	ImGui::Text(
		"%s:\n"
		"%f, %f, %f, %f\n"
		"%f, %f, %f, %f\n"
		"%f, %f, %f, %f\n"
		"%f, %f, %f, %f",
		label,
		matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
		matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
		matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
		matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]);
}

const char kWindowTitle[] = "LC1C_14_コウケンリュウ";

static const int kWindowWidth = 1280;
static const int kWindowHeight = 720;

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	Vector3 a{ 0.2f, 1.0f, 0.0f };
	Vector3 b{ 2.4f, 3.1f, 1.2f };
	Vector3 rotate{ 0.4f, 1.43f, -0.8f };

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		std::memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		// 二項演算子を使う
		Vector3 c = a + b;
		Vector3 d = a - b;
		Vector3 e = a * 2.4f;
		Vector3 f = 2.4f * a;
		Vector3 g = b / 2.0f;

		// 単項演算子を使う
		Vector3 h = -a;
		Vector3 i = +b;

		// 複合代入演算子を使う
		Vector3 j = a;
		j += b;

		Vector3 k = a;
		k -= b;

		Vector3 l = a;
		l *= 2.0f;

		Vector3 n = b;
		n /= 2.0f;

		// 行列の演算子を使う
		Matrix4x4 rotateXMatrix = MakeRotateXMatrix(rotate.x);
		Matrix4x4 rotateYMatrix = MakeRotateYMatrix(rotate.y);
		Matrix4x4 rotateZMatrix = MakeRotateZMatrix(rotate.z);

		Matrix4x4 addMatrix = rotateXMatrix + rotateYMatrix;
		Matrix4x4 subtractMatrix = rotateXMatrix - rotateYMatrix;
		Matrix4x4 rotateMatrix = rotateXMatrix * rotateYMatrix * rotateZMatrix;

		ImGui::Begin("Window");

		ImGui::DragFloat3("a", &a.x, 0.01f);
		ImGui::DragFloat3("b", &b.x, 0.01f);
		ImGui::DragFloat3("rotate", &rotate.x, 0.01f);

		ImGui::SeparatorText("Vector3 Binary Operator");
		ImGuiTextVector3("c = a + b", c);
		ImGuiTextVector3("d = a - b", d);
		ImGuiTextVector3("e = a * 2.4f", e);
		ImGuiTextVector3("f = 2.4f * a", f);
		ImGuiTextVector3("g = b / 2.0f", g);

		ImGui::SeparatorText("Vector3 Unary Operator");
		ImGuiTextVector3("h = -a", h);
		ImGuiTextVector3("i = +b", i);

		ImGui::SeparatorText("Vector3 Compound Assignment Operator");
		ImGuiTextVector3("j = a; j += b", j);
		ImGuiTextVector3("k = a; k -= b", k);
		ImGuiTextVector3("l = a; l *= 2.0f", l);
		ImGuiTextVector3("n = b; n /= 2.0f", n);

		ImGui::SeparatorText("Matrix4x4 Operator");
		ImGuiTextMatrix4x4("addMatrix = rotateXMatrix + rotateYMatrix", addMatrix);
		ImGuiTextMatrix4x4("subtractMatrix = rotateXMatrix - rotateYMatrix", subtractMatrix);
		ImGuiTextMatrix4x4("rotateMatrix = rotateXMatrix * rotateYMatrix * rotateZMatrix", rotateMatrix);

		ImGui::End();

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///


		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}