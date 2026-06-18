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

// 球の構造体
struct Sphere {
	Vector3 center; // 中心点
	float radius;  // 半径
};

// AABBの構造体
struct AABB {
	Vector3 min; // 最小点
	Vector3 max; // 最大点
};

// OBBの構造体
struct OBB {
	Vector3 center;          // 中心点
	Vector3 orientations[3]; // 座標軸
	Vector3 size;            // 座標軸方向の長さの半分
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

// 値を範囲内に収める
float Clamp(float value, float min, float max) {
	float result = value;

	if (result < min) {
		result = min;
	}

	if (result > max) {
		result = max;
	}

	return result;
}

// 度数法からラジアンへ変換
float DegreeToRadian(float degree) {
	const float pi = 3.14159265358979323846f;
	float result = degree * pi / 180.0f;

	return result;
}

// AABBのmin/maxを整える
AABB MakeCorrectAABB(const AABB& aabb) {
	AABB result{};

	if (aabb.min.x <= aabb.max.x) {
		result.min.x = aabb.min.x;
		result.max.x = aabb.max.x;
	} else {
		result.min.x = aabb.max.x;
		result.max.x = aabb.min.x;
	}

	if (aabb.min.y <= aabb.max.y) {
		result.min.y = aabb.min.y;
		result.max.y = aabb.max.y;
	} else {
		result.min.y = aabb.max.y;
		result.max.y = aabb.min.y;
	}

	if (aabb.min.z <= aabb.max.z) {
		result.min.z = aabb.min.z;
		result.max.z = aabb.max.z;
	} else {
		result.min.z = aabb.max.z;
		result.max.z = aabb.min.z;
	}

	return result;
}

// AABBと球の衝突判定
bool IsCollision(const AABB& aabb, const Sphere& sphere) {
	AABB correctAABB = MakeCorrectAABB(aabb);

	// 最近接点を求める
	Vector3 closestPoint{
		Clamp(sphere.center.x, correctAABB.min.x, correctAABB.max.x),
		Clamp(sphere.center.y, correctAABB.min.y, correctAABB.max.y),
		Clamp(sphere.center.z, correctAABB.min.z, correctAABB.max.z),
	};

	// 最近接点と球の中心との距離を求める
	float distance = Length(Subtract(closestPoint, sphere.center));

	// 距離が半径よりも小さければ衝突
	if (distance <= sphere.radius) {
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

// OBBのWorldMatrixを作る
Matrix4x4 MakeOBBWorldMatrix(const OBB& obb) {
	Matrix4x4 result{};

	result.m[0][0] = obb.orientations[0].x;
	result.m[0][1] = obb.orientations[0].y;
	result.m[0][2] = obb.orientations[0].z;

	result.m[1][0] = obb.orientations[1].x;
	result.m[1][1] = obb.orientations[1].y;
	result.m[1][2] = obb.orientations[1].z;

	result.m[2][0] = obb.orientations[2].x;
	result.m[2][1] = obb.orientations[2].y;
	result.m[2][2] = obb.orientations[2].z;

	result.m[3][0] = obb.center.x;
	result.m[3][1] = obb.center.y;
	result.m[3][2] = obb.center.z;
	result.m[3][3] = 1.0f;

	return result;
}

// OBBと球の衝突判定
bool IsCollision(const OBB& obb, const Sphere& sphere) {
	Matrix4x4 obbWorldMatrix = MakeOBBWorldMatrix(obb);
	Matrix4x4 obbWorldMatrixInverse = Inverse(obbWorldMatrix);

	// 球の中心点をOBBのローカル空間に変換する
	Vector3 centerInOBBLocalSpace = Transform(sphere.center, obbWorldMatrixInverse);

	AABB aabbOBBLocal{
		{ -obb.size.x, -obb.size.y, -obb.size.z },
		{ obb.size.x, obb.size.y, obb.size.z },
	};

	Sphere sphereOBBLocal{
		centerInOBBLocalSpace,
		sphere.radius,
	};

	// ローカル空間で衝突判定
	if (IsCollision(aabbOBBLocal, sphereOBBLocal)) {
		return true;
	}

	return false;
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

// 球の描画
void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	const float pi = 3.14159265358979323846f;
	const uint32_t kSubdivision = 16;                 // 分割数
	const float kLonEvery = 2.0f * pi / kSubdivision; // 経度分割1つ分の角度
	const float kLatEvery = pi / kSubdivision;        // 緯度分割1つ分の角度

	// 緯度の方向に分割
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {
		float lat = -pi / 2.0f + kLatEvery * latIndex; // 現在の緯度

		// 経度の方向に分割
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
			float lon = lonIndex * kLonEvery; // 現在の経度

			// world座標系でのa,b,cを求める
			Vector3 a{
				sphere.center.x + sphere.radius * std::cos(lat) * std::cos(lon),
				sphere.center.y + sphere.radius * std::sin(lat),
				sphere.center.z + sphere.radius * std::cos(lat) * std::sin(lon)
			};

			Vector3 b{
				sphere.center.x + sphere.radius * std::cos(lat + kLatEvery) * std::cos(lon),
				sphere.center.y + sphere.radius * std::sin(lat + kLatEvery),
				sphere.center.z + sphere.radius * std::cos(lat + kLatEvery) * std::sin(lon)
			};

			Vector3 c{
				sphere.center.x + sphere.radius * std::cos(lat) * std::cos(lon + kLonEvery),
				sphere.center.y + sphere.radius * std::sin(lat),
				sphere.center.z + sphere.radius * std::cos(lat) * std::sin(lon + kLonEvery)
			};

			// ab,acで線を引く
			DrawLine3D(a, b, viewProjectionMatrix, viewportMatrix, color);
			DrawLine3D(a, c, viewProjectionMatrix, viewportMatrix, color);
		}
	}
}

// OBBの描画
void DrawOBB(const OBB& obb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Matrix4x4 obbWorldMatrix = MakeOBBWorldMatrix(obb);

	Vector3 localVertices[8]{
		{ -obb.size.x, -obb.size.y, -obb.size.z },
		{ obb.size.x, -obb.size.y, -obb.size.z },
		{ -obb.size.x, obb.size.y, -obb.size.z },
		{ obb.size.x, obb.size.y, -obb.size.z },
		{ -obb.size.x, -obb.size.y, obb.size.z },
		{ obb.size.x, -obb.size.y, obb.size.z },
		{ -obb.size.x, obb.size.y, obb.size.z },
		{ obb.size.x, obb.size.y, obb.size.z },
	};

	Vector3 worldVertices[8]{};

	for (int32_t index = 0; index < 8; ++index) {
		worldVertices[index] = Transform(localVertices[index], obbWorldMatrix);
	}

	// -z側の面を描画する
	DrawLine3D(worldVertices[0], worldVertices[1], viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(worldVertices[1], worldVertices[3], viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(worldVertices[3], worldVertices[2], viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(worldVertices[2], worldVertices[0], viewProjectionMatrix, viewportMatrix, color);

	// +z側の面を描画する
	DrawLine3D(worldVertices[4], worldVertices[5], viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(worldVertices[5], worldVertices[7], viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(worldVertices[7], worldVertices[6], viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(worldVertices[6], worldVertices[4], viewProjectionMatrix, viewportMatrix, color);

	// 前後の面をつなぐ線を描画する
	DrawLine3D(worldVertices[0], worldVertices[4], viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(worldVertices[1], worldVertices[5], viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(worldVertices[2], worldVertices[6], viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(worldVertices[3], worldVertices[7], viewProjectionMatrix, viewportMatrix, color);
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

	Vector3 rotate{ 0.0f, 0.0f, 0.0f };

	OBB obb{
		{ -1.0f, 0.0f, 0.0f },
		{
			{ 1.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f },
		},
		{ 0.5f, 0.5f, 0.5f },
	};

	Sphere sphere{
		{ 0.0f, 0.0f, 0.0f },
		0.5f,
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
		ImGui::DragFloat3("obb.center", &obb.center.x, 0.01f);
		ImGui::DragFloat("rotateX", &rotate.x, 1.0f, -360.0f, 360.0f, "%.0f deg");
		ImGui::DragFloat("rotateY", &rotate.y, 1.0f, -360.0f, 360.0f, "%.0f deg");
		ImGui::DragFloat("rotateZ", &rotate.z, 1.0f, -360.0f, 360.0f, "%.0f deg");
		ImGui::InputFloat3("obb.orientations[0]", &obb.orientations[0].x, "%.3f", ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat3("obb.orientations[1]", &obb.orientations[1].x, "%.3f", ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat3("obb.orientations[2]", &obb.orientations[2].x, "%.3f", ImGuiInputTextFlags_ReadOnly);
		ImGui::DragFloat3("obb.size", &obb.size.x, 0.01f);
		ImGui::DragFloat3("sphere.center", &sphere.center.x, 0.01f);
		ImGui::DragFloat("sphere.radius", &sphere.radius, 0.01f);
		ImGui::End();

		Vector3 rotateRadian{
			DegreeToRadian(rotate.x),
			DegreeToRadian(rotate.y),
			DegreeToRadian(rotate.z),
		};

		// 回転行列を生成
		Matrix4x4 rotateMatrix = Multiply(
			MakeRotateXMatrix(rotateRadian.x),
			Multiply(MakeRotateYMatrix(rotateRadian.y), MakeRotateZMatrix(rotateRadian.z)));

		// 回転行列から軸を抽出
		obb.orientations[0].x = rotateMatrix.m[0][0];
		obb.orientations[0].y = rotateMatrix.m[0][1];
		obb.orientations[0].z = rotateMatrix.m[0][2];

		obb.orientations[1].x = rotateMatrix.m[1][0];
		obb.orientations[1].y = rotateMatrix.m[1][1];
		obb.orientations[1].z = rotateMatrix.m[1][2];

		obb.orientations[2].x = rotateMatrix.m[2][0];
		obb.orientations[2].y = rotateMatrix.m[2][1];
		obb.orientations[2].z = rotateMatrix.m[2][2];

		for (int32_t index = 0; index < 3; ++index) {
			obb.orientations[index] = Normalize(obb.orientations[index]);
		}

		if (obb.size.x < 0.0f) {
			obb.size.x = 0.0f;
		}

		if (obb.size.y < 0.0f) {
			obb.size.y = 0.0f;
		}

		if (obb.size.z < 0.0f) {
			obb.size.z = 0.0f;
		}

		if (sphere.radius < 0.0f) {
			sphere.radius = 0.0f;
		}

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

		uint32_t obbColor = 0xFFFFFFFF;
		if (IsCollision(obb, sphere)) {
			obbColor = 0xFF0000FF;
		}

		DrawOBB(obb, viewProjectionMatrix, viewportMatrix, obbColor);
		DrawSphere(sphere, viewProjectionMatrix, viewportMatrix, 0xFFFFFFFF);

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