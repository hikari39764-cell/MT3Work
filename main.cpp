#include <Novice.h>
#include <imgui.h>
#include <cmath>
#include <cstdint>
#include <cstring>

const char kWindowTitle[] = "MT3";

const int kWindowWidth = 1280;
const int kWindowHeight = 720;

const float kPi = 3.14159265358979323846f;

const uint32_t kWhite = 0xFFFFFFFF;
const uint32_t kBlack = 0x000000FF;
const uint32_t kGray = 0xAAAAAAFF;
const uint32_t kBlue = 0x0000FFFF;
const uint32_t kRed = 0xFF0000FF;

struct Vector3 {
	float x;
	float y;
	float z;
};

struct Matrix4x4 {
	float m[4][4];
};

struct Sphere {
	Vector3 center; //!< 中心点
	float radius;  //!< 半径
};

struct ConicalPendulum {
	Vector3 anchor;         // アンカーポイント。固定された端の位置
	float length;           // 紐の長さ
	float halfApexAngle;    // 円錐の頂角の半分
	float angle;            // 現在の角度
	float angularVelocity;  // 角速度ω
};

struct Camera {
	Vector3 translate;
	Vector3 rotate;
};

Vector3 Add(const Vector3& v1, const Vector3& v2) {
	return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

Vector3 Subtract(const Vector3& v1, const Vector3& v2) {
	return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

Vector3 Multiply(float scalar, const Vector3& v) {
	return { scalar * v.x, scalar * v.y, scalar * v.z };
}

float Length(const Vector3& v) {
	return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3 Normalize(const Vector3& v) {
	float length = Length(v);

	if (length == 0.0f) {
		return { 0.0f, 0.0f, 0.0f };
	}

	return { v.x / length, v.y / length, v.z / length };
}

Vector3 operator+(const Vector3& v1, const Vector3& v2) {
	return Add(v1, v2);
}

Vector3 operator-(const Vector3& v1, const Vector3& v2) {
	return Subtract(v1, v2);
}

Vector3 operator*(float scalar, const Vector3& v) {
	return Multiply(scalar, v);
}

Vector3 operator*(const Vector3& v, float scalar) {
	return scalar * v;
}

Vector3 operator/(const Vector3& v, float scalar) {
	return { v.x / scalar, v.y / scalar, v.z / scalar };
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
			for (int i = 0; i < 4; ++i) {
				result.m[row][column] += m1.m[row][i] * m2.m[i][column];
			}
		}
	}

	return result;
}

Matrix4x4 MakeTranslateMatrix(const Vector3& translate) {
	Matrix4x4 result = MakeIdentity4x4();

	result.m[3][0] = translate.x;
	result.m[3][1] = translate.y;
	result.m[3][2] = translate.z;

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

Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
	Matrix4x4 result{};

	result.m[0][0] = 1.0f / aspectRatio / std::tan(fovY / 2.0f);
	result.m[1][1] = 1.0f / std::tan(fovY / 2.0f);
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

Matrix4x4 MakeViewMatrix(const Camera& camera) {
	Matrix4x4 translateMatrix = MakeTranslateMatrix({ -camera.translate.x, -camera.translate.y, -camera.translate.z });
	Matrix4x4 rotateXMatrix = MakeRotateXMatrix(-camera.rotate.x);
	Matrix4x4 rotateYMatrix = MakeRotateYMatrix(-camera.rotate.y);
	Matrix4x4 rotateZMatrix = MakeRotateZMatrix(-camera.rotate.z);

	Matrix4x4 rotateMatrix = Multiply(rotateZMatrix, Multiply(rotateYMatrix, rotateXMatrix));

	return Multiply(translateMatrix, rotateMatrix);
}

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

void DrawLine3D(
	const Vector3& start,
	const Vector3& end,
	const Matrix4x4& viewProjectionMatrix,
	const Matrix4x4& viewportMatrix,
	uint32_t color) {

	Vector3 screenStart = Transform(Transform(start, viewProjectionMatrix), viewportMatrix);
	Vector3 screenEnd = Transform(Transform(end, viewProjectionMatrix), viewportMatrix);

	Novice::DrawLine(
		int(screenStart.x),
		int(screenStart.y),
		int(screenEnd.x),
		int(screenEnd.y),
		color);
}

void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f;
	const uint32_t kSubdivision = 10;
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision);

	// 奥から手前への線を順々に引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		float x = -kGridHalfWidth + kGridEvery * float(xIndex);

		Vector3 start{ x, 0.0f, -kGridHalfWidth };
		Vector3 end{ x, 0.0f, kGridHalfWidth };

		uint32_t color = (xIndex == kSubdivision / 2) ? kBlack : kGray;
		DrawLine3D(start, end, viewProjectionMatrix, viewportMatrix, color);
	}

	// 左から右も同じように順々に引いていく
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		float z = -kGridHalfWidth + kGridEvery * float(zIndex);

		Vector3 start{ -kGridHalfWidth, 0.0f, z };
		Vector3 end{ kGridHalfWidth, 0.0f, z };

		uint32_t color = (zIndex == kSubdivision / 2) ? kBlack : kGray;
		DrawLine3D(start, end, viewProjectionMatrix, viewportMatrix, color);
	}
}

void DrawSphere(
	const Sphere& sphere,
	const Matrix4x4& viewProjectionMatrix,
	const Matrix4x4& viewportMatrix,
	uint32_t color) {

	const uint32_t kSubdivision = 16;
	const float kLonEvery = 2.0f * kPi / float(kSubdivision);
	const float kLatEvery = kPi / float(kSubdivision);

	// 緯度の方向に分割 -π/2 ～ π/2
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {
		float lat = -kPi / 2.0f + kLatEvery * float(latIndex);

		// 経度の方向に分割 0 ～ 2π
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
			float lon = kLonEvery * float(lonIndex);

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

			DrawLine3D(a, b, viewProjectionMatrix, viewportMatrix, color);
			DrawLine3D(a, c, viewProjectionMatrix, viewportMatrix, color);
		}
	}
}

void DrawCircleXZ(
	const Vector3& center,
	float radius,
	const Matrix4x4& viewProjectionMatrix,
	const Matrix4x4& viewportMatrix,
	uint32_t color) {

	const uint32_t kSubdivision = 64;
	const float kEvery = 2.0f * kPi / float(kSubdivision);

	for (uint32_t index = 0; index < kSubdivision; ++index) {
		float theta1 = kEvery * float(index);
		float theta2 = kEvery * float(index + 1);

		Vector3 p1{
			center.x + std::cos(theta1) * radius,
			center.y,
			center.z - std::sin(theta1) * radius
		};

		Vector3 p2{
			center.x + std::cos(theta2) * radius,
			center.y,
			center.z - std::sin(theta2) * radius
		};

		DrawLine3D(p1, p2, viewProjectionMatrix, viewportMatrix, color);
	}
}

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Camera camera{};
	camera.translate = { 0.0f, 1.9f, -6.49f };
	camera.rotate = { 0.26f, 0.0f, 0.0f };

	ConicalPendulum conicalPendulum{};
	conicalPendulum.anchor = { 0.0f, 1.0f, 0.0f };
	conicalPendulum.length = 0.8f;
	conicalPendulum.halfApexAngle = 0.7f;
	conicalPendulum.angle = 0.0f;
	conicalPendulum.angularVelocity = 0.0f;

	Sphere bob{};
	bob.center = { 0.0f, 0.0f, 0.0f };
	bob.radius = 0.05f;

	Sphere anchorSphere{};
	anchorSphere.center = conicalPendulum.anchor;
	anchorSphere.radius = 0.025f;

	bool isStart = false;

	const float kDeltaTime = 1.0f / 60.0f;
	const float kGravity = 9.8f;

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

		ImGui::Begin("Window");

		if (ImGui::Button("Start")) {
			isStart = true;
		}

		ImGui::DragFloat3("CameraTranslate", &camera.translate.x, 0.01f);
		ImGui::DragFloat3("CameraRotate", &camera.rotate.x, 0.01f);

		ImGui::DragFloat3("Anchor", &conicalPendulum.anchor.x, 0.01f);
		ImGui::DragFloat("Length", &conicalPendulum.length, 0.01f);
		ImGui::DragFloat("HalfApexAngle", &conicalPendulum.halfApexAngle, 0.01f);
		ImGui::DragFloat("Angle", &conicalPendulum.angle, 0.01f);

		ImGui::Text("AngularVelocity : %.3f", conicalPendulum.angularVelocity);

		ImGui::End();

		// 値が壊れないように最低限補正しておく
		if (conicalPendulum.length < 0.01f) {
			conicalPendulum.length = 0.01f;
		}

		if (conicalPendulum.halfApexAngle < 0.01f) {
			conicalPendulum.halfApexAngle = 0.01f;
		}

		if (conicalPendulum.halfApexAngle > 1.4f) {
			conicalPendulum.halfApexAngle = 1.4f;
		}

		if (isStart) {
			// 円錐振り子の角速度を求める
			conicalPendulum.angularVelocity =
				std::sqrt(kGravity / (conicalPendulum.length * std::cos(conicalPendulum.halfApexAngle)));

			// 角度を進める
			conicalPendulum.angle += conicalPendulum.angularVelocity * kDeltaTime;

			if (conicalPendulum.angle >= 2.0f * kPi) {
				conicalPendulum.angle -= 2.0f * kPi;
			}
		}

		// 半径と高さを求める
		float radius = std::sin(conicalPendulum.halfApexAngle) * conicalPendulum.length;
		float height = std::cos(conicalPendulum.halfApexAngle) * conicalPendulum.length;

		// ボブの位置を求める
		bob.center.x = conicalPendulum.anchor.x + std::cos(conicalPendulum.angle) * radius;
		bob.center.y = conicalPendulum.anchor.y - height;
		bob.center.z = conicalPendulum.anchor.z - std::sin(conicalPendulum.angle) * radius;

		anchorSphere.center = conicalPendulum.anchor;

		Matrix4x4 viewMatrix = MakeViewMatrix(camera);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		Matrix4x4 viewProjectionMatrix = Multiply(viewMatrix, projectionMatrix);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		DrawGrid(viewProjectionMatrix, viewportMatrix);

		// 円運動の軌道を描画
		Vector3 circleCenter{
			conicalPendulum.anchor.x,
			conicalPendulum.anchor.y - height,
			conicalPendulum.anchor.z
		};
		DrawCircleXZ(circleCenter, radius, viewProjectionMatrix, viewportMatrix, kGray);

		// アンカーとボブを結ぶ線を描画
		DrawLine3D(conicalPendulum.anchor, bob.center, viewProjectionMatrix, viewportMatrix, kWhite);

		// アンカーとボブを描画
		DrawSphere(anchorSphere, viewProjectionMatrix, viewportMatrix, kRed);
		DrawSphere(bob, viewProjectionMatrix, viewportMatrix, kWhite);

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