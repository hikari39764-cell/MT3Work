#include <Novice.h>
#include <imgui.h>
#include <cmath>
#include <cstring>
#include <cstdint>

struct Vector3 {
	float x;
	float y;
	float z;

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

struct Matrix4x4 {
	float m[4][4];
};

struct Sphere {
	Vector3 center; // 中心点
	float radius;  // 半径
};

struct Ball {
	Vector3 position;     // ボールの位置
	Vector3 velocity;     // ボールの速度
	Vector3 acceleration; // ボールの加速度
	float radius;         // ボールの半径
	uint32_t color;       // ボールの色
};

Vector3 Add(const Vector3& v1, const Vector3& v2) {
	Vector3 result{};

	result.x = v1.x + v2.x;
	result.y = v1.y + v2.y;
	result.z = v1.z + v2.z;

	return result;
}

Vector3 Subtract(const Vector3& v1, const Vector3& v2) {
	Vector3 result{};

	result.x = v1.x - v2.x;
	result.y = v1.y - v2.y;
	result.z = v1.z - v2.z;

	return result;
}

Vector3 Multiply(float scalar, const Vector3& v) {
	Vector3 result{};

	result.x = scalar * v.x;
	result.y = scalar * v.y;
	result.z = scalar * v.z;

	return result;
}

float Dot(const Vector3& v1, const Vector3& v2) {
	float result = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

	return result;
}

float Length(const Vector3& v) {
	float result = std::sqrt(Dot(v, v));

	return result;
}

Vector3 Normalize(const Vector3& v) {
	Vector3 result{};

	float length = Length(v);
	if (length != 0.0f) {
		result.x = v.x / length;
		result.y = v.y / length;
		result.z = v.z / length;
	}

	return result;
}

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

Vector3 operator-(const Vector3& v) {
	return { -v.x, -v.y, -v.z };
}

Vector3 operator+(const Vector3& v) {
	return v;
}

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

Matrix4x4 operator*(const Matrix4x4& m1, const Matrix4x4& m2) {
	return Multiply(m1, m2);
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

Matrix4x4 MakeScaleMatrix(const Vector3& scale) {
	Matrix4x4 result{};

	result.m[0][0] = scale.x;
	result.m[1][1] = scale.y;
	result.m[2][2] = scale.z;
	result.m[3][3] = 1.0f;

	return result;
}

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

Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate) {
	Matrix4x4 scaleMatrix = MakeScaleMatrix(scale);
	Matrix4x4 rotateXMatrix = MakeRotateXMatrix(rotate.x);
	Matrix4x4 rotateYMatrix = MakeRotateYMatrix(rotate.y);
	Matrix4x4 rotateZMatrix = MakeRotateZMatrix(rotate.z);
	Matrix4x4 translateMatrix = MakeTranslateMatrix(translate);
	Matrix4x4 rotateMatrix = rotateXMatrix * rotateYMatrix * rotateZMatrix;

	return scaleMatrix * rotateMatrix * translateMatrix;
}

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

void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f;                               // Gridの半分の幅
	const uint32_t kSubdivision = 10;                                // 分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / kSubdivision; // 1つ分の長さ

	// 奥から手前への線を順々に引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		float x = -kGridHalfWidth + kGridEvery * xIndex;
		Vector3 start{ x, 0.0f, -kGridHalfWidth };
		Vector3 end{ x, 0.0f, kGridHalfWidth };

		uint32_t color = 0xAAAAAAFF;
		if (xIndex == kSubdivision / 2) {
			color = 0x000000FF;
		}

		DrawLine3D(start, end, viewProjectionMatrix, viewportMatrix, color);
	}

	// 左から右も同じように順々に引いていく
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		float z = -kGridHalfWidth + kGridEvery * zIndex;
		Vector3 start{ -kGridHalfWidth, 0.0f, z };
		Vector3 end{ kGridHalfWidth, 0.0f, z };

		uint32_t color = 0xAAAAAAFF;
		if (zIndex == kSubdivision / 2) {
			color = 0x000000FF;
		}

		DrawLine3D(start, end, viewProjectionMatrix, viewportMatrix, color);
	}
}

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

const char kWindowTitle[] = "MT3";

static const int kWindowWidth = 1280;
static const int kWindowHeight = 720;

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	Vector3 cameraTranslate{ 0.0f, 1.9f, -6.49f };
	Vector3 cameraRotate{ 0.26f, 0.0f, 0.0f };

	Vector3 center{ 0.0f, 0.0f, 0.0f };
	float radius = 0.8f;
	float angularVelocity = 3.14159265358979323846f;
	float angle = 0.0f;

	Ball ball{};
	ball.position = { center.x + radius, center.y, center.z };
	ball.velocity = {};
	ball.acceleration = {};
	ball.radius = 0.05f;
	ball.color = 0xFFFFFFFF;

	bool isMove = false;

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

		const float pi = 3.14159265358979323846f;
		const float deltaTime = 1.0f / 60.0f;

		ImGui::Begin("Window");

		// Startボタンで動かし始める
		if (ImGui::Button("Start")) {
			isMove = true;
		}

		ImGui::SameLine();

		// Resetボタンで初期状態に戻す
		if (ImGui::Button("Reset")) {
			isMove = false;
			angle = 0.0f;
			ball.velocity = {};
			ball.acceleration = {};
		}

		ImGui::DragFloat3("CameraTranslate", &cameraTranslate.x, 0.01f);
		ImGui::DragFloat3("CameraRotate", &cameraRotate.x, 0.01f);
		ImGui::DragFloat3("Center", &center.x, 0.01f);
		ImGui::DragFloat("Radius", &radius, 0.01f);
		ImGui::DragFloat("AngularVelocity", &angularVelocity, 0.01f);
		ImGui::DragFloat("Angle", &angle, 0.01f);
		ImGui::DragFloat("Ball.Radius", &ball.radius, 0.01f);

		ImGui::Text("position : %.3f, %.3f, %.3f", ball.position.x, ball.position.y, ball.position.z);
		ImGui::Text("velocity : %.3f, %.3f, %.3f", ball.velocity.x, ball.velocity.y, ball.velocity.z);
		ImGui::Text("acceleration : %.3f, %.3f, %.3f", ball.acceleration.x, ball.acceleration.y, ball.acceleration.z);

		ImGui::End();

		// Spaceキーで動かし始める
		if (preKeys[DIK_SPACE] == 0 && keys[DIK_SPACE] != 0) {
			isMove = true;
		}

		// Rキーで初期状態に戻す
		if (preKeys[DIK_R] == 0 && keys[DIK_R] != 0) {
			isMove = false;
			angle = 0.0f;
			ball.velocity = {};
			ball.acceleration = {};
		}

		// 半径が負にならないようにする
		if (radius < 0.0f) {
			radius = 0.0f;
		}

		if (ball.radius <= 0.0f) {
			ball.radius = 0.01f;
		}

		// 角度を更新する
		if (isMove) {
			angle += angularVelocity * deltaTime;

			if (angle >= 2.0f * pi) {
				angle -= 2.0f * pi;
			}
		}

		// XY平面上で等速円運動させる
		ball.position.x = center.x + std::cos(angle) * radius;
		ball.position.y = center.y + std::sin(angle) * radius;
		ball.position.z = center.z;

		// 等速円運動の速度を求める
		ball.velocity.x = -radius * angularVelocity * std::sin(angle);
		ball.velocity.y = radius * angularVelocity * std::cos(angle);
		ball.velocity.z = 0.0f;

		// 等速円運動の加速度を求める
		ball.acceleration.x = -radius * angularVelocity * angularVelocity * std::cos(angle);
		ball.acceleration.y = -radius * angularVelocity * angularVelocity * std::sin(angle);
		ball.acceleration.z = 0.0f;

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///


		Matrix4x4 cameraMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		Matrix4x4 viewProjectionMatrix = viewMatrix * projectionMatrix;
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		// Gridを描画
		DrawGrid(viewProjectionMatrix, viewportMatrix);

		// 中心点を黒い球で描画
		Sphere centerSphere{ center, 0.025f };
		DrawSphere(centerSphere, viewProjectionMatrix, viewportMatrix, 0x000000FF);

		// 円運動する球を描画
		Sphere ballSphere{ ball.position, ball.radius };
		DrawSphere(ballSphere, viewProjectionMatrix, viewportMatrix, ball.color);

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