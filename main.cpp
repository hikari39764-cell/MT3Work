#include <Novice.h>
#include <imgui.h>
#include <cmath>
#include <cassert>
#include <cstdint>

const char kWindowTitle[] = "MT3";

const int kWindowWidth = 1280;
const int kWindowHeight = 720;

const float kPi = 3.14159265358979323846f;

struct Vector3 {
	float x;
	float y;
	float z;
};

struct Matrix4x4 {
	float m[4][4];
};

struct Sphere {
	Vector3 center; // 中心点
	float radius;  // 半径
};

struct Pendulum {
	Vector3 anchor;              // アンカーポイント。固定された端の位置
	float length;                // 紐の長さ
	float angle;                 // 現在の角度
	float angularVelocity;       // 角速度
	float angularAcceleration;   // 角加速度
};

Vector3 Add(const Vector3& v1, const Vector3& v2) {
	return {
		v1.x + v2.x,
		v1.y + v2.y,
		v1.z + v2.z,
	};
}

Vector3 Subtract(const Vector3& v1, const Vector3& v2) {
	return {
		v1.x - v2.x,
		v1.y - v2.y,
		v1.z - v2.z,
	};
}

Vector3 Multiply(float scalar, const Vector3& v) {
	return {
		scalar * v.x,
		scalar * v.y,
		scalar * v.z,
	};
}

float Dot(const Vector3& v1, const Vector3& v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

float Length(const Vector3& v) {
	return std::sqrt(Dot(v, v));
}

Vector3 Normalize(const Vector3& v) {
	float length = Length(v);
	if (length == 0.0f) {
		return { 0.0f, 0.0f, 0.0f };
	}

	return {
		v.x / length,
		v.y / length,
		v.z / length,
	};
}

Matrix4x4 MakeIdentity4x4() {
	Matrix4x4 result{};

	result.m[0][0] = 1.0f;
	result.m[1][1] = 1.0f;
	result.m[2][2] = 1.0f;
	result.m[3][3] = 1.0f;

	return result;
}

Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result{};

	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			result.m[row][column] =
				m1.m[row][0] * m2.m[0][column] +
				m1.m[row][1] * m2.m[1][column] +
				m1.m[row][2] * m2.m[2][column] +
				m1.m[row][3] * m2.m[3][column];
		}
	}

	return result;
}

Matrix4x4 MakeScaleMatrix(const Vector3& scale) {
	Matrix4x4 result{};

	result.m[0][0] = scale.x;
	result.m[1][1] = scale.y;
	result.m[2][2] = scale.z;
	result.m[3][3] = 1.0f;

	return result;
}

Matrix4x4 MakeRotateXMatrix(float radian) {
	Matrix4x4 result = MakeIdentity4x4();

	result.m[1][1] = std::cos(radian);
	result.m[1][2] = std::sin(radian);
	result.m[2][1] = -std::sin(radian);
	result.m[2][2] = std::cos(radian);

	return result;
}

Matrix4x4 MakeRotateYMatrix(float radian) {
	Matrix4x4 result = MakeIdentity4x4();

	result.m[0][0] = std::cos(radian);
	result.m[0][2] = -std::sin(radian);
	result.m[2][0] = std::sin(radian);
	result.m[2][2] = std::cos(radian);

	return result;
}

Matrix4x4 MakeRotateZMatrix(float radian) {
	Matrix4x4 result = MakeIdentity4x4();

	result.m[0][0] = std::cos(radian);
	result.m[0][1] = std::sin(radian);
	result.m[1][0] = -std::sin(radian);
	result.m[1][1] = std::cos(radian);

	return result;
}

Matrix4x4 MakeTranslateMatrix(const Vector3& translate) {
	Matrix4x4 result = MakeIdentity4x4();

	result.m[3][0] = translate.x;
	result.m[3][1] = translate.y;
	result.m[3][2] = translate.z;

	return result;
}

Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate) {
	Matrix4x4 scaleMatrix = MakeScaleMatrix(scale);

	Matrix4x4 rotateXMatrix = MakeRotateXMatrix(rotate.x);
	Matrix4x4 rotateYMatrix = MakeRotateYMatrix(rotate.y);
	Matrix4x4 rotateZMatrix = MakeRotateZMatrix(rotate.z);

	Matrix4x4 rotateMatrix = Multiply(rotateXMatrix, Multiply(rotateYMatrix, rotateZMatrix));
	Matrix4x4 translateMatrix = MakeTranslateMatrix(translate);

	Matrix4x4 worldMatrix = Multiply(Multiply(scaleMatrix, rotateMatrix), translateMatrix);

	return worldMatrix;
}

Matrix4x4 Inverse(const Matrix4x4& matrix) {
	Matrix4x4 result{};
	float buffer[4][8]{};

	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			buffer[row][column] = matrix.m[row][column];
		}

		buffer[row][row + 4] = 1.0f;
	}

	for (int column = 0; column < 4; ++column) {
		int pivot = column;
		float pivotSize = std::fabs(buffer[column][column]);

		for (int row = column + 1; row < 4; ++row) {
			float size = std::fabs(buffer[row][column]);
			if (size > pivotSize) {
				pivot = row;
				pivotSize = size;
			}
		}

		assert(pivotSize != 0.0f);
		if (pivotSize == 0.0f) {
			return MakeIdentity4x4();
		}

		if (pivot != column) {
			for (int i = 0; i < 8; ++i) {
				float temp = buffer[column][i];
				buffer[column][i] = buffer[pivot][i];
				buffer[pivot][i] = temp;
			}
		}

		float pivotValue = buffer[column][column];

		for (int i = 0; i < 8; ++i) {
			buffer[column][i] /= pivotValue;
		}

		for (int row = 0; row < 4; ++row) {
			if (row == column) {
				continue;
			}

			float scale = buffer[row][column];

			for (int i = 0; i < 8; ++i) {
				buffer[row][i] -= buffer[column][i] * scale;
			}
		}
	}

	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			result.m[row][column] = buffer[row][column + 4];
		}
	}

	return result;
}

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

Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	Vector3 result{};

	result.x =
		vector.x * matrix.m[0][0] +
		vector.y * matrix.m[1][0] +
		vector.z * matrix.m[2][0] +
		1.0f * matrix.m[3][0];

	result.y =
		vector.x * matrix.m[0][1] +
		vector.y * matrix.m[1][1] +
		vector.z * matrix.m[2][1] +
		1.0f * matrix.m[3][1];

	result.z =
		vector.x * matrix.m[0][2] +
		vector.y * matrix.m[1][2] +
		vector.z * matrix.m[2][2] +
		1.0f * matrix.m[3][2];

	float w =
		vector.x * matrix.m[0][3] +
		vector.y * matrix.m[1][3] +
		vector.z * matrix.m[2][3] +
		1.0f * matrix.m[3][3];

	if (w != 0.0f) {
		result.x /= w;
		result.y /= w;
		result.z /= w;
	}

	return result;
}

void DrawLine3D(
	const Vector3& start,
	const Vector3& end,
	const Matrix4x4& viewProjectionMatrix,
	const Matrix4x4& viewportMatrix,
	uint32_t color) {

	Vector3 screenStart = Transform(Transform(start, viewProjectionMatrix), viewportMatrix);
	Vector3 screenEnd = Transform(Transform(end, viewProjectionMatrix), viewportMatrix);

	Novice::DrawLine(
		static_cast<int>(screenStart.x),
		static_cast<int>(screenStart.y),
		static_cast<int>(screenEnd.x),
		static_cast<int>(screenEnd.y),
		color);
}

void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f;        // Gridの半分の幅
	const uint32_t kSubdivision = 10;        // 分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision); // 1つ分の長さ

	// 奥から手前への線を順々に引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		float x = -kGridHalfWidth + kGridEvery * float(xIndex);

		Vector3 start{ x, 0.0f, -kGridHalfWidth };
		Vector3 end{ x, 0.0f, kGridHalfWidth };

		uint32_t color = 0xAAAAAAFF;

		// 原点を通る線だけ少し濃くする
		if (xIndex == kSubdivision / 2) {
			color = 0x000000FF;
		}

		DrawLine3D(start, end, viewProjectionMatrix, viewportMatrix, color);
	}

	// 左から右も同じように順々に引いていく
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		float z = -kGridHalfWidth + kGridEvery * float(zIndex);

		Vector3 start{ -kGridHalfWidth, 0.0f, z };
		Vector3 end{ kGridHalfWidth, 0.0f, z };

		uint32_t color = 0xAAAAAAFF;

		// 原点を通る線だけ少し濃くする
		if (zIndex == kSubdivision / 2) {
			color = 0x000000FF;
		}

		DrawLine3D(start, end, viewProjectionMatrix, viewportMatrix, color);
	}
}

void DrawSphere(
	const Sphere& sphere,
	const Matrix4x4& viewProjectionMatrix,
	const Matrix4x4& viewportMatrix,
	uint32_t color) {

	const uint32_t kSubdivision = 16;             // 分割数
	const float kLonEvery = 2.0f * kPi / float(kSubdivision); // 経度分割1つ分の角度
	const float kLatEvery = kPi / float(kSubdivision);        // 緯度分割1つ分の角度

	// 緯度の方向に分割 -π/2 ～ π/2
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {
		float lat = -kPi / 2.0f + kLatEvery * float(latIndex);

		// 経度の方向に分割 0 ～ 2π
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
			float lon = kLonEvery * float(lonIndex);

			// 現在の点
			Vector3 a{
				sphere.center.x + sphere.radius * std::cos(lat) * std::cos(lon),
				sphere.center.y + sphere.radius * std::sin(lat),
				sphere.center.z + sphere.radius * std::cos(lat) * std::sin(lon)
			};

			// 緯度方向に1つ進んだ点
			Vector3 b{
				sphere.center.x + sphere.radius * std::cos(lat + kLatEvery) * std::cos(lon),
				sphere.center.y + sphere.radius * std::sin(lat + kLatEvery),
				sphere.center.z + sphere.radius * std::cos(lat + kLatEvery) * std::sin(lon)
			};

			// 経度方向に1つ進んだ点
			Vector3 c{
				sphere.center.x + sphere.radius * std::cos(lat) * std::cos(lon + kLonEvery),
				sphere.center.y + sphere.radius * std::sin(lat),
				sphere.center.z + sphere.radius * std::cos(lat) * std::sin(lon + kLonEvery)
			};

			DrawLine3D(a, b, viewProjectionMatrix, viewportMatrix, color);
			DrawLine3D(a, c, viewProjectionMatrix, viewportMatrix, color);
		}
	}
}

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Vector3 cameraTranslate{ 0.0f, 1.9f, -6.49f };
	Vector3 cameraRotate{ 0.26f, 0.0f, 0.0f };

	Pendulum pendulum{};
	pendulum.anchor = { 0.0f, 1.0f, 0.0f };
	pendulum.length = 0.8f;
	pendulum.angle = 0.7f;
	pendulum.angularVelocity = 0.0f;
	pendulum.angularAcceleration = 0.0f;

	Sphere pendulumSphere{};
	pendulumSphere.radius = 0.05f;

	bool isMove = false;

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		const float deltaTime = 1.0f / 60.0f;
		const float gravityAcceleration = 9.8f;

		ImGui::Begin("Window");

		// Startボタンで動かし始める
		if (ImGui::Button("Start")) {
			isMove = true;
		}

		ImGui::SameLine();

		// Resetボタンで初期状態に戻す
		if (ImGui::Button("Reset")) {
			isMove = false;
			pendulum.anchor = { 0.0f, 1.0f, 0.0f };
			pendulum.length = 0.8f;
			pendulum.angle = 0.7f;
			pendulum.angularVelocity = 0.0f;
			pendulum.angularAcceleration = 0.0f;
		}

		ImGui::DragFloat3("CameraTranslate", &cameraTranslate.x, 0.01f);
		ImGui::DragFloat3("CameraRotate", &cameraRotate.x, 0.01f);

		ImGui::DragFloat3("Pendulum.Anchor", &pendulum.anchor.x, 0.01f);
		ImGui::DragFloat("Pendulum.Length", &pendulum.length, 0.01f);
		ImGui::DragFloat("Pendulum.Angle", &pendulum.angle, 0.01f);
		ImGui::DragFloat("Pendulum.AngularVelocity", &pendulum.angularVelocity, 0.01f);
		ImGui::Text("AngularAcceleration : %.3f", pendulum.angularAcceleration);

		ImGui::End();

		// Spaceキーで動かし始める
		if (preKeys[DIK_SPACE] == 0 && keys[DIK_SPACE] != 0) {
			isMove = true;
		}

		// Rキーで初期状態に戻す
		if (preKeys[DIK_R] == 0 && keys[DIK_R] != 0) {
			isMove = false;
			pendulum.anchor = { 0.0f, 1.0f, 0.0f };
			pendulum.length = 0.8f;
			pendulum.angle = 0.7f;
			pendulum.angularVelocity = 0.0f;
			pendulum.angularAcceleration = 0.0f;
		}

		// 紐の長さが0にならないようにする
		if (pendulum.length <= 0.01f) {
			pendulum.length = 0.01f;
		}

		// 振り子の角度を更新する
		if (isMove) {
			pendulum.angularAcceleration =
				-(gravityAcceleration / pendulum.length) * std::sin(pendulum.angle);

			pendulum.angularVelocity += pendulum.angularAcceleration * deltaTime;
			pendulum.angle += pendulum.angularVelocity * deltaTime;
		}

		// 振り子の先端位置を求める
		pendulumSphere.center.x = pendulum.anchor.x + std::sin(pendulum.angle) * pendulum.length;
		pendulumSphere.center.y = pendulum.anchor.y - std::cos(pendulum.angle) * pendulum.length;
		pendulumSphere.center.z = pendulum.anchor.z;

		Matrix4x4 cameraMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(
			0.45f,
			float(kWindowWidth) / float(kWindowHeight),
			0.1f,
			100.0f);

		Matrix4x4 viewProjectionMatrix = Multiply(viewMatrix, projectionMatrix);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(
			0.0f,
			0.0f,
			float(kWindowWidth),
			float(kWindowHeight),
			0.0f,
			1.0f);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// Gridを描画
		DrawGrid(viewProjectionMatrix, viewportMatrix);

		// アンカーを描画
		Sphere anchorSphere{ pendulum.anchor, 0.025f };
		DrawSphere(anchorSphere, viewProjectionMatrix, viewportMatrix, 0x000000FF);

		// 紐を描画
		DrawLine3D(
			pendulum.anchor,
			pendulumSphere.center,
			viewProjectionMatrix,
			viewportMatrix,
			0xFFFFFFFF);

		// 振り子の先端を描画
		DrawSphere(pendulumSphere, viewProjectionMatrix, viewportMatrix, 0xFFFFFFFF);

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