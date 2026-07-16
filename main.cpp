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

struct Plane {
	Vector3 normal; //!< 法線
	float distance; //!< 距離
};

struct Segment {
	Vector3 origin; //!< 始点
	Vector3 diff;   //!< 終点への差分ベクトル
};

struct Capsule {
	Segment segment;
	float radius;
};

struct Ball {
	Vector3 position;     // ボールの位置
	Vector3 velocity;     // ボールの速度
	Vector3 acceleration; // ボールの加速度
	float mass;           // ボールの質量
	float radius;         // ボールの半径
	uint32_t color;       // ボールの色
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

float Dot(const Vector3& v1, const Vector3& v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	return {
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x
	};
}

float Length(const Vector3& v) {
	return std::sqrt(Dot(v, v));
}

Vector3 Normalize(const Vector3& v) {
	float length = Length(v);

	if (length == 0.0f) {
		return { 0.0f, 0.0f, 0.0f };
	}

	return { v.x / length, v.y / length, v.z / length };
}

Vector3 Project(const Vector3& v1, const Vector3& v2) {
	Vector3 normal = Normalize(v2);
	return Multiply(Dot(v1, normal), normal);
}

Vector3 Reflect(const Vector3& input, const Vector3& normal) {
	Vector3 n = Normalize(normal);

	return Subtract(input, Multiply(2.0f * Dot(input, n), n));
}

Vector3 Perpendicular(const Vector3& vector) {
	if (vector.x != 0.0f || vector.y != 0.0f) {
		return Normalize({ -vector.y, vector.x, 0.0f });
	}

	return Normalize({ 0.0f, -vector.z, vector.y });
}

Vector3 operator+(const Vector3& v1, const Vector3& v2) {
	return Add(v1, v2);
}

Vector3 operator-(const Vector3& v1, const Vector3& v2) {
	return Subtract(v1, v2);
}

Vector3 operator-(const Vector3& v) {
	return { -v.x, -v.y, -v.z };
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
			for (int index = 0; index < 4; ++index) {
				result.m[row][column] += m1.m[row][index] * m2.m[index][column];
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

float SignedDistance(const Plane& plane, const Vector3& point) {
	return Dot(point, plane.normal) - plane.distance;
}

bool IsCollision(const Sphere& sphere, const Plane& plane) {
	float distance = SignedDistance(plane, sphere.center);

	return distance <= sphere.radius;
}

bool IsCollision(const Capsule& capsule, const Plane& plane) {
	Vector3 start = capsule.segment.origin;
	Vector3 end = capsule.segment.origin + capsule.segment.diff;

	float startDistance = SignedDistance(plane, start);
	float endDistance = SignedDistance(plane, end);

	if (startDistance <= capsule.radius) {
		return true;
	}

	if (endDistance <= capsule.radius) {
		return true;
	}

	return false;
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

void DrawPlane(
	const Plane& plane,
	const Matrix4x4& viewProjectionMatrix,
	const Matrix4x4& viewportMatrix,
	uint32_t color) {

	const float kPlaneSize = 2.0f;

	Vector3 center = plane.normal * plane.distance;

	Vector3 axis1 = Perpendicular(plane.normal);
	Vector3 axis2 = Normalize(Cross(plane.normal, axis1));

	Vector3 p0 = center + axis1 * kPlaneSize + axis2 * kPlaneSize;
	Vector3 p1 = center - axis1 * kPlaneSize + axis2 * kPlaneSize;
	Vector3 p2 = center - axis1 * kPlaneSize - axis2 * kPlaneSize;
	Vector3 p3 = center + axis1 * kPlaneSize - axis2 * kPlaneSize;

	DrawLine3D(p0, p1, viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(p1, p2, viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(p2, p3, viewProjectionMatrix, viewportMatrix, color);
	DrawLine3D(p3, p0, viewProjectionMatrix, viewportMatrix, color);
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

	Plane plane{};
	plane.normal = Normalize({ -0.2f, 0.9f, -0.3f });
	plane.distance = 0.0f;

	Ball ball{};
	ball.position = { 0.8f, 1.2f, 0.3f };
	ball.velocity = { 0.0f, 0.0f, 0.0f };
	ball.acceleration = { 0.0f, -9.8f, 0.0f };
	ball.mass = 2.0f;
	ball.radius = 0.05f;
	ball.color = kWhite;

	bool isStart = false;
	bool isHit = false;

	float restitution = 0.8f;
	const float kDeltaTime = 1.0f / 60.0f;

	Vector3 initialPosition = ball.position;
	Vector3 initialVelocity = ball.velocity;

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

		if (ImGui::Button("Reset")) {
			isStart = false;
			ball.position = initialPosition;
			ball.velocity = initialVelocity;
		}

		ImGui::DragFloat3("Camera.Translate", &camera.translate.x, 0.01f);
		ImGui::DragFloat3("Camera.Rotate", &camera.rotate.x, 0.01f);

		ImGui::DragFloat3("Plane.Normal", &plane.normal.x, 0.01f);
		ImGui::DragFloat("Plane.Distance", &plane.distance, 0.01f);

		ImGui::DragFloat3("Ball.Position", &ball.position.x, 0.01f);
		ImGui::DragFloat3("Ball.Velocity", &ball.velocity.x, 0.01f);
		ImGui::DragFloat("Ball.Radius", &ball.radius, 0.01f);
		ImGui::DragFloat("Restitution", &restitution, 0.01f);

		ImGui::Text("Hit : %s", isHit ? "true" : "false");

		ImGui::End();

		// 法線は単位ベクトルにしておく
		if (Length(plane.normal) == 0.0f) {
			plane.normal = { 0.0f, 1.0f, 0.0f };
		} else {
			plane.normal = Normalize(plane.normal);
		}

		if (ball.radius < 0.01f) {
			ball.radius = 0.01f;
		}

		if (restitution < 0.0f) {
			restitution = 0.0f;
		}

		if (restitution > 1.0f) {
			restitution = 1.0f;
		}

		isHit = false;

		if (isStart) {
			Vector3 previousPosition = ball.position;

			// 重力加速度を設定
			ball.acceleration = { 0.0f, -9.8f, 0.0f };

			// 速度と位置を更新
			ball.velocity = ball.velocity + ball.acceleration * kDeltaTime;
			ball.position = ball.position + ball.velocity * kDeltaTime;

			// すり抜け対策用のカプセルを作る
			Capsule moveCapsule{};
			moveCapsule.segment.origin = previousPosition;
			moveCapsule.segment.diff = ball.position - previousPosition;
			moveCapsule.radius = ball.radius;

			if (IsCollision(moveCapsule, plane)) {
				Sphere ballSphere{ ball.position, ball.radius };

				if (IsCollision(ballSphere, plane)) {
					isHit = true;

					// めり込んだ分を平面の外に戻す
					float distance = SignedDistance(plane, ball.position);
					float penetration = ball.radius - distance;
					ball.position = ball.position + plane.normal * penetration;

					// 平面に向かっている場合だけ反射する
					if (Dot(ball.velocity, plane.normal) < 0.0f) {
						Vector3 reflected = Reflect(ball.velocity, plane.normal);
						Vector3 projectToNormal = Project(reflected, plane.normal);
						Vector3 movingDirection = reflected - projectToNormal;

						// 法線方向だけ反発係数で弱める
						ball.velocity = projectToNormal * restitution + movingDirection;
					}
				}
			}
		}

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

		DrawPlane(plane, viewProjectionMatrix, viewportMatrix, kWhite);

		Sphere drawSphere{};
		drawSphere.center = ball.position;
		drawSphere.radius = ball.radius;

		DrawSphere(drawSphere, viewProjectionMatrix, viewportMatrix, isHit ? kRed : ball.color);

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