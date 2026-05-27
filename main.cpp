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
};

// 4x4行列の構造体
struct Matrix4x4 {
	float m[4][4];
};

// 線分の構造体
struct Segment {
	Vector3 origin; // 始点
	Vector3 diff;   // 終点への差分ベクトル
};

// 三角形の構造体
struct Triangle {
	Vector3 vertices[3]; // 頂点
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

// ベクトルの内積
float Dot(const Vector3& v1, const Vector3& v2) {
	float result = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

	return result;
}

// ベクトルのクロス積
Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	Vector3 result{};

	result.x = v1.y * v2.z - v1.z * v2.y;
	result.y = v1.z * v2.x - v1.x * v2.z;
	result.z = v1.x * v2.y - v1.y * v2.x;

	return result;
}

// ベクトルの長さ
float Length(const Vector3& vector) {
	float result = std::sqrt(Dot(vector, vector));

	return result;
}

// ベクトルの正規化
Vector3 Normalize(const Vector3& vector) {
	Vector3 result{};

	float length = Length(vector);

	if (length != 0.0f) {
		result.x = vector.x / length;
		result.y = vector.y / length;
		result.z = vector.z / length;
	} else {
		result.y = 1.0f;
	}

	return result;
}

// 三角形と線分の衝突判定
bool IsCollision(const Triangle& triangle, const Segment& segment) {
	// 三角形の各辺を表すベクトルを求める
	Vector3 v01 = Subtract(triangle.vertices[1], triangle.vertices[0]);
	Vector3 v12 = Subtract(triangle.vertices[2], triangle.vertices[1]);
	Vector3 v20 = Subtract(triangle.vertices[0], triangle.vertices[2]);

	// 三角形が存在する平面の法線を求める
	Vector3 normal = Normalize(Cross(v01, v12));

	// まず垂直判定を行うために、法線と線の内積を求める
	float dot = Dot(normal, segment.diff);

	// 垂直＝平行であるので、衝突しているはずがない
	if (dot == 0.0f) {
		return false;
	}

	// 三角形が存在する平面の距離を求める
	float distance = Dot(triangle.vertices[0], normal);

	// tを求める
	float t = (distance - Dot(segment.origin, normal)) / dot;

	// tの値が線分の範囲外なら衝突していない
	if (t < 0.0f || 1.0f < t) {
		return false;
	}

	// 衝突点を求める
	Vector3 point = Add(segment.origin, Multiply(t, segment.diff));

	// 各頂点から衝突点までのベクトルを求める
	Vector3 v0p = Subtract(point, triangle.vertices[0]);
	Vector3 v1p = Subtract(point, triangle.vertices[1]);
	Vector3 v2p = Subtract(point, triangle.vertices[2]);

	// 各辺を結んだベクトルと、頂点と衝突点を結んだベクトルのクロス積を取る
	Vector3 cross01 = Cross(v01, v0p);
	Vector3 cross12 = Cross(v12, v1p);
	Vector3 cross20 = Cross(v20, v2p);

	// すべての小三角形のクロス積と法線が同じ方向を向いていたら衝突
	if (Dot(cross01, normal) >= 0.0f &&
		Dot(cross12, normal) >= 0.0f &&
		Dot(cross20, normal) >= 0.0f) {
		return true;
	}

	return false;
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

// 線分の描画
void DrawLine3D(const Vector3& start, const Vector3& end, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 startScreen = Transform(Transform(start, viewProjectionMatrix), viewportMatrix);
	Vector3 endScreen = Transform(Transform(end, viewProjectionMatrix), viewportMatrix);

	Novice::DrawLine(
		int(startScreen.x),
		int(startScreen.y),
		int(endScreen.x),
		int(endScreen.y),
		color);
}

// グリッドの描画
void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f;                               // Gridの半分の幅
	const uint32_t kSubdivision = 10;                                // 分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / kSubdivision; // 1つ分の長さ

	// 奥から手前への線を順々に引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		// 上の情報を使ってワールド座標系上の始点と終点を求める
		float x = -kGridHalfWidth + kGridEvery * xIndex;
		Vector3 start{ x, 0.0f, -kGridHalfWidth };
		Vector3 end{ x, 0.0f, kGridHalfWidth };

		// 変換した座標を使って表示
		uint32_t color = 0xAAAAAAFF;
		if (xIndex == kSubdivision / 2) {
			color = 0x000000FF;
		}

		DrawLine3D(start, end, viewProjectionMatrix, viewportMatrix, color);
	}

	// 左から右も同じように順々に引いていく
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		// 奥から手前が左右に変わるだけ
		float z = -kGridHalfWidth + kGridEvery * zIndex;
		Vector3 start{ -kGridHalfWidth, 0.0f, z };
		Vector3 end{ kGridHalfWidth, 0.0f, z };

		// 変換した座標を使って表示
		uint32_t color = 0xAAAAAAFF;
		if (zIndex == kSubdivision / 2) {
			color = 0x000000FF;
		}

		DrawLine3D(start, end, viewProjectionMatrix, viewportMatrix, color);
	}
}

// 三角形の描画
void DrawTriangle(const Triangle& triangle, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 screenVertices[3]{};

	for (int32_t index = 0; index < 3; ++index) {
		screenVertices[index] = Transform(Transform(triangle.vertices[index], viewProjectionMatrix), viewportMatrix);
	}

	Novice::DrawTriangle(
		int(screenVertices[0].x),
		int(screenVertices[0].y),
		int(screenVertices[1].x),
		int(screenVertices[1].y),
		int(screenVertices[2].x),
		int(screenVertices[2].y),
		color,
		kFillModeWireFrame);
}

// 線分の描画
void DrawSegment(const Segment& segment, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 start = segment.origin;
	Vector3 end = Add(segment.origin, segment.diff);

	DrawLine3D(start, end, viewProjectionMatrix, viewportMatrix, color);
}

const char kWindowTitle[] = "LC1C_14_コウケンリュウ";

static const int kWindowWidth = 1280;
static const int kWindowHeight = 720;

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	Vector3 cameraTranslate{ 0.0f, 1.9f, -6.49f };
	Vector3 cameraRotate{ 0.26f, 0.0f, 0.0f };

	Triangle triangle{
		{
			{ -1.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f },
			{ 1.0f, 0.0f, 0.0f },
		}
	};

	Segment segment{
		{ 0.0f, 0.49f, -1.0f },
		{ 0.0f, 0.5f, 2.0f },
	};

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

		ImGui::Begin("Window");
		ImGui::DragFloat3("CameraTranslate", &cameraTranslate.x, 0.01f);
		ImGui::DragFloat3("CameraRotate", &cameraRotate.x, 0.01f);
		ImGui::DragFloat3("Triangle.v0", &triangle.vertices[0].x, 0.01f);
		ImGui::DragFloat3("Triangle.v1", &triangle.vertices[1].x, 0.01f);
		ImGui::DragFloat3("Triangle.v2", &triangle.vertices[2].x, 0.01f);
		ImGui::DragFloat3("Segment.Origin", &segment.origin.x, 0.01f);
		ImGui::DragFloat3("Segment.Diff", &segment.diff.x, 0.01f);
		ImGui::End();

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		Matrix4x4 cameraMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		Matrix4x4 viewProjectionMatrix = Multiply(viewMatrix, projectionMatrix);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		DrawGrid(viewProjectionMatrix, viewportMatrix);
		DrawTriangle(triangle, viewProjectionMatrix, viewportMatrix, 0xFFFFFFFF);

		uint32_t segmentColor = 0xFFFFFFFF;
		if (IsCollision(triangle, segment)) {
			segmentColor = 0xFF0000FF;
		}

		DrawSegment(segment, viewProjectionMatrix, viewportMatrix, segmentColor);

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