#include <Novice.h>
#include <cmath>
#include <cstring>
#include <cstdint>

// 3次元ベクトルの構造体
struct Vector3 {
	float x;
	float y;
	float z;
};

// 4x4行列の構造体
struct Matrix4x4 {
	float m[4][4];
};

// ベクトルのクロス積
Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	Vector3 result{};

	result.x = v1.y * v2.z - v1.z * v2.y;
	result.y = v1.z * v2.x - v1.x * v2.z;
	result.z = v1.x * v2.y - v1.y * v2.x;

	return result;
}

// 行列の積
Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result{};

	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			for (int element = 0; element < 4; ++element) {
				result.m[row][column] += m1.m[row][element] * m2.m[element][column];
			}
		}
	}

	return result;
}

// ベクトルと行列の積
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	Vector3 result{};

	result.x = vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + matrix.m[3][0];
	result.y = vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + matrix.m[3][1];
	result.z = vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + matrix.m[3][2];

	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + matrix.m[3][3];

	if (w != 0.0f) {
		result.x /= w;
		result.y /= w;
		result.z /= w;
	}

	return result;
}

// 平行移動行列
Matrix4x4 MakeTranslateMatrix(const Vector3& translate) {
	Matrix4x4 result{};

	result.m[0][0] = 1.0f;
	result.m[1][1] = 1.0f;
	result.m[2][2] = 1.0f;
	result.m[3][0] = translate.x;
	result.m[3][1] = translate.y;
	result.m[3][2] = translate.z;
	result.m[3][3] = 1.0f;

	return result;
}

// 拡大縮小行列
Matrix4x4 MakeScaleMatrix(const Vector3& scale) {
	Matrix4x4 result{};

	result.m[0][0] = scale.x;
	result.m[1][1] = scale.y;
	result.m[2][2] = scale.z;
	result.m[3][3] = 1.0f;

	return result;
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

// 3次元アフィン変換行列
Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate) {
	Matrix4x4 scaleMatrix = MakeScaleMatrix(scale);
	Matrix4x4 rotateXMatrix = MakeRotateXMatrix(rotate.x);
	Matrix4x4 rotateYMatrix = MakeRotateYMatrix(rotate.y);
	Matrix4x4 rotateZMatrix = MakeRotateZMatrix(rotate.z);
	Matrix4x4 translateMatrix = MakeTranslateMatrix(translate);
	Matrix4x4 rotateMatrix = Multiply(rotateXMatrix, Multiply(rotateYMatrix, rotateZMatrix));

	return Multiply(Multiply(scaleMatrix, rotateMatrix), translateMatrix);
}

// 逆行列
Matrix4x4 Inverse(const Matrix4x4& matrix) {
	Matrix4x4 result{};

	float a = matrix.m[0][0];
	float b = matrix.m[0][1];
	float c = matrix.m[0][2];
	float d = matrix.m[1][0];
	float e = matrix.m[1][1];
	float f = matrix.m[1][2];
	float g = matrix.m[2][0];
	float h = matrix.m[2][1];
	float i = matrix.m[2][2];

	float determinant =
		a * (e * i - f * h) -
		b * (d * i - f * g) +
		c * (d * h - e * g);

	if (determinant == 0.0f) {
		result.m[0][0] = 1.0f;
		result.m[1][1] = 1.0f;
		result.m[2][2] = 1.0f;
		result.m[3][3] = 1.0f;

		return result;
	}

	float inverseDeterminant = 1.0f / determinant;

	result.m[0][0] = (e * i - f * h) * inverseDeterminant;
	result.m[0][1] = (c * h - b * i) * inverseDeterminant;
	result.m[0][2] = (b * f - c * e) * inverseDeterminant;

	result.m[1][0] = (f * g - d * i) * inverseDeterminant;
	result.m[1][1] = (a * i - c * g) * inverseDeterminant;
	result.m[1][2] = (c * d - a * f) * inverseDeterminant;

	result.m[2][0] = (d * h - e * g) * inverseDeterminant;
	result.m[2][1] = (b * g - a * h) * inverseDeterminant;
	result.m[2][2] = (a * e - b * d) * inverseDeterminant;

	result.m[3][0] = -(
		matrix.m[3][0] * result.m[0][0] +
		matrix.m[3][1] * result.m[1][0] +
		matrix.m[3][2] * result.m[2][0]);

	result.m[3][1] = -(
		matrix.m[3][0] * result.m[0][1] +
		matrix.m[3][1] * result.m[1][1] +
		matrix.m[3][2] * result.m[2][1]);

	result.m[3][2] = -(
		matrix.m[3][0] * result.m[0][2] +
		matrix.m[3][1] * result.m[1][2] +
		matrix.m[3][2] * result.m[2][2]);

	result.m[3][3] = 1.0f;

	return result;
}

// 透視投影行列
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
	Matrix4x4 result{};

	float cot = 1.0f / std::tan(fovY / 2.0f);

	result.m[0][0] = cot / aspectRatio;
	result.m[1][1] = cot;
	result.m[2][2] = farClip / (farClip - nearClip);
	result.m[2][3] = 1.0f;
	result.m[3][2] = (-nearClip * farClip) / (farClip - nearClip);

	return result;
}

// ビューポート変換行列
Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
	Matrix4x4 result{};

	result.m[0][0] = width / 2.0f;
	result.m[1][1] = -height / 2.0f;
	result.m[2][2] = maxDepth - minDepth;
	result.m[3][0] = left + width / 2.0f;
	result.m[3][1] = top + height / 2.0f;
	result.m[3][2] = minDepth;
	result.m[3][3] = 1.0f;

	return result;
}

// 数値表示用の設定
static const int kRowHeight = 20;

// 3次元ベクトルの数値表示関数
void VectorScreenPrintf(int x, int y, const Vector3& vector) {
	Novice::ScreenPrintf(x, y, "%s", "Cross");
	Novice::ScreenPrintf(x, y + kRowHeight, "x : %6.02f", vector.x);
	Novice::ScreenPrintf(x, y + kRowHeight * 2, "y : %6.02f", vector.y);
	Novice::ScreenPrintf(x, y + kRowHeight * 3, "z : %6.02f", vector.z);
}

const char kWindowTitle[] = "LC1C_14_コウケンリュウ";

static const int kWindowWidth = 1280;
static const int kWindowHeight = 720;

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	const Vector3 kLocalVertices[3] = {
		{ 0.0f, 1.0f, 0.0f },
		{ 1.0f, -1.0f, 0.0f },
		{ -1.0f, -1.0f, 0.0f },
	};

	Vector3 rotate{};
	Vector3 translate{ 0.0f, 0.0f, 0.0f };
	Vector3 cameraPosition{ 0.0f, 0.0f, -10.0f };

	Vector3 v1{ 1.2f, -3.9f, 2.5f };
	Vector3 v2{ 2.8f, 0.4f, -1.3f };
	Vector3 cross = Cross(v1, v2);

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

		if (keys[DIK_S] != 0) {
			translate.z += 0.05f;
		}

		if (keys[DIK_W] != 0) {
			translate.z -= 0.05f;
		}

		if (keys[DIK_D] != 0) {
			translate.x -= 0.05f;
		}

		if (keys[DIK_A] != 0) {
			translate.x += 0.05f;
		}

		rotate.y += 0.03f;

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// クロス積の確認用
		VectorScreenPrintf(0, 0, cross);

		// 各種行列の計算
		Matrix4x4 worldMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, rotate, translate);
		Matrix4x4 cameraMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, cameraPosition);
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		Matrix4x4 worldViewProjectionMatrix = Multiply(worldMatrix, Multiply(viewMatrix, projectionMatrix));
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		Vector3 screenVertices[3]{};

		for (uint32_t i = 0; i < 3; ++i) {
			Vector3 ndcVertex = Transform(kLocalVertices[i], worldViewProjectionMatrix);
			screenVertices[i] = Transform(ndcVertex, viewportMatrix);
		}

		// 描画
		Novice::DrawTriangle(
			int(screenVertices[0].x), int(screenVertices[0].y),
			int(screenVertices[1].x), int(screenVertices[1].y),
			int(screenVertices[2].x), int(screenVertices[2].y),
			RED,
			kFillModeSolid);

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