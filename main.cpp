#include <Novice.h>
#include <algorithm>
#include <cmath>
#include <cstring>

// 4x4行列の構造体
struct Matrix4x4 {
	float m[4][4];
};

// 1. 行列の加法
Matrix4x4 Add(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result{};

	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			result.m[row][column] = m1.m[row][column] + m2.m[row][column];
		}
	}

	return result;
}

// 2. 行列の減法
Matrix4x4 Subtract(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result{};

	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			result.m[row][column] = m1.m[row][column] - m2.m[row][column];
		}
	}

	return result;
}

// 3. 行列の積
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

// 4. 逆行列
Matrix4x4 Inverse(const Matrix4x4& m) {
	Matrix4x4 source = m;
	Matrix4x4 result = {};

	for (int row = 0; row < 4; ++row) {
		result.m[row][row] = 1.0f;
	}

	for (int pivotIndex = 0; pivotIndex < 4; ++pivotIndex) {
		int pivotRow = pivotIndex;
		float maxAbs = std::fabs(source.m[pivotIndex][pivotIndex]);

		for (int row = pivotIndex + 1; row < 4; ++row) {
			float absValue = std::fabs(source.m[row][pivotIndex]);

			if (absValue > maxAbs) {
				maxAbs = absValue;
				pivotRow = row;
			}
		}

		if (maxAbs == 0.0f) {
			return result;
		}

		if (pivotRow != pivotIndex) {
			for (int column = 0; column < 4; ++column) {
				std::swap(source.m[pivotIndex][column], source.m[pivotRow][column]);
				std::swap(result.m[pivotIndex][column], result.m[pivotRow][column]);
			}
		}

		float pivot = source.m[pivotIndex][pivotIndex];

		for (int column = 0; column < 4; ++column) {
			source.m[pivotIndex][column] /= pivot;
			result.m[pivotIndex][column] /= pivot;
		}

		for (int row = 0; row < 4; ++row) {
			if (row != pivotIndex) {
				float factor = source.m[row][pivotIndex];

				for (int column = 0; column < 4; ++column) {
					source.m[row][column] -= factor * source.m[pivotIndex][column];
					result.m[row][column] -= factor * result.m[pivotIndex][column];
				}
			}
		}
	}

	return result;
}

// 5. 転置行列
Matrix4x4 Transpose(const Matrix4x4& m) {
	Matrix4x4 result{};

	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			result.m[row][column] = m.m[column][row];
		}
	}

	return result;
}

// 6. 単位行列の作成
Matrix4x4 MakeIdentity4x4() {
	Matrix4x4 result{};

	for (int row = 0; row < 4; ++row) {
		result.m[row][row] = 1.0f;
	}

	return result;
}

// 数値表示用の設定
static const int kRowHeight = 20;
static const int kColumnWidth = 60;

// 4x4行列の数値表示関数
void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* label) {
	Novice::ScreenPrintf(x, y, "%s", label);

	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(
				x + column * kColumnWidth,
				y + (row + 1) * kRowHeight,
				"%6.02f",
				matrix.m[row][column]);
		}
	}
}

const char kWindowTitle[] = "LC1C_14_コウケンリュウ";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	Matrix4x4 m1 = { 3.2f, 0.7f, 9.6f, 4.4f,
				   5.5f, 1.3f, 7.8f, 2.1f,
				   6.9f, 8.0f, 2.6f, 1.0f,
				   0.5f, 7.2f, 5.1f, 3.3f };

	Matrix4x4 m2 = { 4.1f, 6.5f, 3.3f, 2.2f,
				   8.8f, 0.6f, 9.9f, 7.7f,
				   1.1f, 5.5f, 6.6f, 0.0f,
				   3.3f, 9.9f, 8.8f, 2.2f };

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

		Matrix4x4 resultAdd = Add(m1, m2);
		Matrix4x4 resultMultiply = Multiply(m1, m2);
		Matrix4x4 resultSubtract = Subtract(m1, m2);
		Matrix4x4 inverseM1 = Inverse(m1);
		Matrix4x4 inverseM2 = Inverse(m2);
		Matrix4x4 transposeM1 = Transpose(m1);
		Matrix4x4 transposeM2 = Transpose(m2);
		Matrix4x4 identity = MakeIdentity4x4();

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		MatrixScreenPrintf(0, 0, resultAdd, "Add");
		MatrixScreenPrintf(0, kRowHeight * 5, resultSubtract, "Subtract");
		MatrixScreenPrintf(0, kRowHeight * 5 * 2, resultMultiply, "Multiply");
		MatrixScreenPrintf(0, kRowHeight * 5 * 3, inverseM1, "inverseM1");
		MatrixScreenPrintf(0, kRowHeight * 5 * 4, inverseM2, "inverseM2");
		MatrixScreenPrintf(kColumnWidth * 5, 0, transposeM1, "transposeM1");
		MatrixScreenPrintf(kColumnWidth * 5, kRowHeight * 5, transposeM2, "transposeM2");
		MatrixScreenPrintf(kColumnWidth * 5, kRowHeight * 5 * 2, identity, "identity");

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
