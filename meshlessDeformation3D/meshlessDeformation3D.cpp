//  main.cpp
//  meshlessDeformation3D
//
//  Created by zhuravlik on 2016/10/25.
//  Copyright © 2016年 zhuravlik. All rights reserved.
//
//Windows:glut.h MacOS:GLUT/GLUT.h

//Windows Server 2008r2 + visual c++　動作確認済
//OSX Sierra バグ有動作せず

//Win32用
#ifdef _WIN32
#include <Windows.h>  //Windows APIを使う準備
#include <glut.h>
#endif

//Mac OS用
#ifdef __APPLE__
#include <unistd.h>
#include <stdlib.h>
#include <GLUT/GLUT.h>  // OpenGLを使う準備
#endif

#include <stdio.h>
#include <iostream>
#include <numeric>
#include <vector>  // vector（便利な配列）を使う準備
#include <fstream>  // ファイルを扱う準備

#include <Eigen/Dense>  // Eigen（線形代数ライブラリ）を使う準備
#include <Eigen/LU>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

#pragma comment(lib, "glut32.lib")

Vector3d camEye(0, 0, 20), camCenter(0, 0, 0),
camUp(0, 1, 0);  // カメラの姿勢（位置、注視点、上方向）
Vector3d litEye(0, 0, 4), litCenter(0, 0, 0),
litUp(0, 1, 0);  // 照明の姿勢（位置、注視点、上方向）

int screenWidth = 1024, screenHeight = 768;  // 画面サイズ
double distanceToScreen = 2;                 // スクリーンまでの距離
double zNear = 1,
zFar = 256;  // カメラからの距離がzNearとzFarの間のものが描かれる
#ifdef _WIN32
DWORD prevTime = 0;  // 前回の時刻
#endif

int oldX = -1, oldY,  // 前回のマウスの位置の保存
litMode = 0;      // 照明を操作中かどうかのフラグ

				  // ------------------------------------------------------------
				  // メッシュクラスの定義
				  //
class mesh {
public:
	vector<Vector3d> vertices;    // メッシュの全頂点の集合
	vector<Vector3d> velocities;  // 対応する速度
	vector<Vector3d> normals;     // 対応する法線ベクトル
	vector<Vector2d> texCoords;   // 対応するテクスチャ座標
	int fixed;  // 0:動く輪郭 1:固定の輪郭（外周の壁となる立方体に使用）

	vector<vector<Vector3i>> faces;  // メッシュの全三角形の集合
									 // faces[face index][vertex index][type]
									 //   type -
									 //     0: 頂点
									 //     1: 法線ベクトル
									 //     2: テクスチャ座標

									 // テキストファイルの全行を読み込む
	vector<string> readAllLines(const string& fileName) {
		ifstream i(fileName);
		string t;
		vector<string> output;
		while (i && getline(i, t)) output.push_back(t);

		return output;
	}

	// 文字列（input）を文字（splitter）で分割する
	vector<string> split(const string& input, char splitter,
		bool bRemoveEmptyEntries) {
		stringstream s(input);
		string t;
		vector<string> output;
		while (getline(s, t, splitter))
			if (!bRemoveEmptyEntries || t != "") output.push_back(t);

		return output;
	}

	// メッシュをファイルから読み込む
	void loadObj(const string& fileName) {
		vertices.clear();
		velocities.clear();
		normals.clear();
		texCoords.clear();
		faces.clear();

		vector<string> lines =
			readAllLines(fileName);  // テキストファイルの全行を読み込む

		for (int i = 0; i < lines.size(); i++) {
			vector<string> t = split(lines[i], ' ', true);
			if (!t.size()) continue;

			if (t[0] == "v")  // 頂点
				vertices.push_back(Vector3d(atof(t[1].c_str()),
					atof(t[2].c_str()),
					atof(t[3].c_str())));

			if (t[0] == "vn")  // 法線ベクトル
				normals.push_back(Vector3d(atof(t[1].c_str()),
					atof(t[2].c_str()),
					atof(t[3].c_str())));

			if (t[0] == "vt")  // テクスチャ座標
				texCoords.push_back(
					Vector2d(atof(t[1].c_str()), atof(t[2].c_str())));

			if (t[0] == "f") {
				vector<Vector3i> f(t.size() - 1);
				for (int j = 0; j < f.size(); j++) {
					vector<string> u = split(t[j + 1], '/', false);
					for (int k = 0; k < u.size(); k++)
						f[j][k] = atoi(u[k].c_str()) - 1;
				}
				faces.push_back(f);
			}
		}
		cout << "Loaded " << (int)faces.size() << " faces" << endl;
	}
};

// ------------------------------------------------------------
// メッシュ集合
//
vector<mesh> meshes;  // 動くメッシュ（頂点と速度を毎回更新）
vector<mesh> rests;   // 初期形状を保持（その後は一切更新されない）

					  // ------------------------------------------------------------
					  // vectorに関する演算の定義 (meshlessDeformation2Dに同じ)
					  //
template <class T>
vector<T>& operator+=(vector<T>& v,
	const vector<T>& w) {  // vector（配列）同士の足し算
	for (int i = 0; i < v.size(); i++) v[i] += w[i];
	return v;
}

template <class T, class U>
vector<T>& operator+=(vector<T>& v, const U& w) {  // vector（配列）の各要素と何かの足し算
	for (int i = 0; i < v.size(); i++) v[i] += w;
	return v;
}

template <class T>
vector<T> operator-(const vector<T>& v, const vector<T>& w) {  // vector（配列）同士の引き算
	vector<T> o = v;
	for (int i = 0; i < o.size(); i++) o[i] -= w[i];
	return o;
}

template <class T, class U>
vector<T> operator-(const vector<T>& v, const U& w) {  // vector（配列）の各要素と何かの引き算
	vector<T> o = v;
	for (int i = 0; i < o.size(); i++) o[i] -= w;
	return o;
}

template <class T, class U>
vector<T> operator*(const vector<T>& v, const U& w) {  // vector（配列）の各要素と何かの掛け算
	vector<T> o = v;
	for (int i = 0; i < o.size(); i++) o[i] *= w;
	return o;
}

template <class T>
T sum(vector<T>& v) {  // vector（配列）の合計値を求める
	return accumulate(v.begin() + 1, v.end(), v[0]);
}

template <class T>
T mean(vector<T>& v) {  // vector（配列）の平均値を求める
	return sum(v) / v.size();
}

template <class T>
T min_element(vector<T>& v) {  // vector（配列）中の最小値を見つける
	return *min_element(v.begin(), v.end());
}

template <class T>
T max_element(vector<T>& v) {  // vector（配列）中の最大値を見つける
	return *max_element(v.begin(), v.end());
}

// ------------------------------------------------------------
// 衝突判定と衝撃力の作用
//
// ------------------------------------------------------------
// 直線の軌跡と頂点の軌跡（線分）の衝突判定のための関数の定義
//

class intersection {
public:
	//intersection内、vecter及びmatrixの次元は統一する必要有
	//vertA, vertB, vertCで平面を作成
	Vector3d vertA0, vertA1;
	Vector3d vertB0, vertB1;
	Vector3d vertC0, vertC1;


	//平面にぶつかる点
	Vector3d p0, p1;    // 頂点の軌跡の端点

	Matrix3d rotation;  // 回転行列

	intersection(Vector3d& vertA0_, Vector3d& vertA1_, Vector3d& vertB0_, Vector3d& vertB1_, Vector3d& vertC0_, Vector3d& vertC1_, Vector3d& p0_, Vector3d& p1_)
		: vertA0(vertA0_), vertA1(vertA1_), vertB0(vertB0_), vertB1(vertB1_), vertC0(vertC0_), vertC1(vertC1_), p0(p0_), p1(p1_) {
		rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
		// z軸を中心に90度回転
		//rotation << 1, 0, 0, 0, 0, -1, 0, 1, 0;
		// x軸を中心に90度回転
		//rotation << 0, 0, 1, 0, 1, 0, -1, 0, 0;
		// y軸を中心に90度回転
	}

	double f(double t) {
		Vector3d vertA, vertB, vertC, p;
		Vector3d ab, ac, ap, n;

		//vertA, vertB, vertCにおけるt時点での位置
		vertA = vertA0 + (vertA1 - vertA0) * t;
		vertB = vertB0 + (vertB1 - vertB0) * t;
		vertC = vertC0 + (vertC1 - vertC0) * t;

		p = p0 + (p1 - p0) * t;  // 頂点のt時点での位置

		ab = vertB - vertA;
		ac = vertC - vertA;

		//法線ベクトル
		n = ab.cross(ac);

		//pが平面上にきたときの座標
		ap = p - vertA;

		//gaとspの内積が0のときのtがpが平面上にきたとき(衝突したとき)
		return n[0] * ap[0] + n[1] * ap[1] + n[2] * ap[2];
	}

	// 関数fの微分を中心差分で求める
	double gradient(double x, double h) {
		return (f(x + h) - f(x - h)) / (h * 2);
	}
};

// ------------------------------------------------------------
// 衝突判定と衝撃力の作用
//

int collision(mesh& A, mesh& B) {
	for (int a = 0; a < A.vertices.size(); a++) {
		Vector3d& velocity = A.velocities[a];   //物体の各頂点の速度
		Vector3d p0 = A.vertices[a], p1 = p0 + velocity;  // 頂点の軌跡の端点

														  // 頂点の軌跡を囲うbounding box
														  // bbA = {minX, minY, minZ, maxX, maxY, maxZ}
		vector<double> bbA{ min(p0[0], p1[0]), min(p0[1], p1[1]), min(p0[2], p1[2]),
			max(p0[0], p1[0]), max(p0[1], p1[1]), max(p0[2], p1[2]) };

		for (int b = 0; b < B.vertices.size(); b++) {
			Vector3d& v0 = B.velocities[b];
			Vector3d& v1 = B.velocities[(b + 1) % B.velocities.size()];
			Vector3d& v2 = B.velocities[(b + 2) % B.velocities.size()];

			// 頂点Aの軌跡の端点
			Vector3d vertA0 = B.vertices[b], vertA1 = vertA0 + v0;
			// 頂点Bの軌跡の端点
			Vector3d vertB0 = B.vertices[(b + 1) % B.vertices.size()], vertB1 = vertB0 + v1;
			// 頂点Cの軌跡の端点
			Vector3d vertC0 = B.vertices[(b + 2) % B.vertices.size()], vertC1 = vertC0 + v2;

			// 線分の軌跡を囲うbounding box
			vector<double> X(6), Y(6), Z(6);
			X[0] = vertA0[0];
			X[1] = vertB1[0];
			X[2] = vertB0[0];
			X[3] = vertB1[0];
			X[4] = vertC0[0];
			X[5] = vertC1[0];

			Y[0] = vertA0[1];
			Y[1] = vertB1[1];
			Y[2] = vertB0[1];
			Y[3] = vertB1[1];
			Y[4] = vertC0[1];
			Y[5] = vertC1[1];

			Z[0] = vertA0[2];
			Z[1] = vertB1[2];
			Z[2] = vertB0[2];
			Z[3] = vertB1[2];
			Z[4] = vertC0[2];
			Z[5] = vertC1[2];

			vector<double> bbB{ min_element(X), min_element(Y), min_element(Z),
				max_element(X), max_element(Y), max_element(Z) };

#if 1
			// bounding box同士の衝突判定

			if (bbA[0] <= bbB[3] && bbB[0] <= bbA[3] && bbA[1] <= bbB[4] &&
				bbB[1] <= bbA[4] && bbA[2] <= bbB[5] && bbB[2] <= bbA[5])
#endif
			{


				intersection detector(vertA0, vertA1, vertB0, vertB1, vertC0, vertC1, p0, p1);

				// ニュートン法を用いて、f(t)=0、となるtを求める
				double t = 0.5;

				// ニュートン法での繰り返しは20回
				for (int l = 0; l < 20; l++) {
					double y = detector.f(t);
					t -= y / detector.gradient(t, 0.01);


					// yがほぼ0になったらtが求まったとする
					if (y * y < 1e-20) {

						// 0 <= t <= 1なら衝突
						// （数値計算の誤差を考慮し、後ろのみに1%の余裕を与える）
						if (0 <= t && t <= 1.01) {
							Vector3d vertA, vertB, vertC, p;

							//vertA, vertB, vertCにおけるt時点での位置
							vertA = vertA0 + (vertA1 - vertA0) * t;
							vertB = vertB0 + (vertB1 - vertB0) * t;
							vertC = vertC0 + (vertC1 - vertC0) * t;

							p = p0 + (p1 - p0) * t;

							Vector3d ab = vertB - vertA, ac = vertC - vertA, bc = vertC - vertA, ca = vertA - vertC;
							Vector3d ap = p - vertA, bp = p - vertB, cp = p - vertC;
							Vector3d cross;

							cross[0] = ab[1] * bp[2] - ab[2] * bp[1]; //ABとBpの外積
							cross[1] = bc[2] * cp[0] - bc[0] * cp[2]; //BCとCpの外積
							cross[2] = ca[0] * ap[1] - ca[1] * ap[0]; //CAとApの外積

																	  //外積の向きが全て同方向の場合
							if ((cross[0] <= 0 && cross[1] <= 0 && cross[2] <= 0) || (cross[0] > 0 && cross[1] > 0 && cross[2] > 0)) {
								Vector3d n;
								ab.normalize(), ac.normalize();;

								//法線ベクトル
								n = ab.cross(ac);
								//平面側固定時の判定
								if (B.fixed) {
									velocity -= n * velocity.dot(n) * 5;  // 法線方向に跳ね返す
								}
								else {
									Vector3d v = velocity;
									velocity += n * n.dot((v0 + v1 + v2) / 3) - n * v.dot(n);

									v0 += n * n.dot(v) - n * v0.dot(n);
									v1 += n * n.dot(v) - n * v1.dot(n);
									v2 += n * n.dot(v) - n * v2.dot(n);
								}
								return 1;
							}
						}
					}
				}
			}
		}
	}
	return 0;
}

// ------------------------------------------------------------
// Meshless deformation (論文の3.2〜3.3章を参照)
//
// currentに最もフィットするようにrestを回転＆平行移動したものをgoalに求める
//
vector<Vector3d> computeGoalPositions(mesh& current, mesh& rest) {
	//最初に全部2dを3dに変えた

	Vector3d xcm = mean(current.vertices), x0cm = mean(rest.vertices);

	vector<Vector3d> p = current.vertices - xcm, q = rest.vertices - x0cm;

	Matrix3d A = Matrix3d::Zero();
	for (int i = 0; i < p.size(); i++) A += p[i] * q[i].transpose();

	JacobiSVD<Matrix3d> svd(A.transpose() * A,
		Eigen::ComputeFullU | Eigen::ComputeFullV);

	Matrix3d D = svd.singularValues().asDiagonal();
	for (int i = 0; i < D.rows(); i++) D(i, i) = sqrt(D(i, i));
	Matrix3d S = svd.matrixU() * D * svd.matrixV().transpose();

	Matrix3d R = A * S.inverse();

	vector<Vector3d> goal;
	for (int i = 0; i < q.size(); i++) goal.push_back(R * q[i] + xcm);

	return goal;
}

// ------------------------------------------------------------
// 描画処理を行う関数（繰り返し呼ばれ続ける）
//
void display() {
#if 1
	// ------------------------------------------------------------
	// 重力の作用
	//
	for (int i = 0; i < meshes.size(); i++)
		if (!meshes[i].fixed)
			meshes[i].velocities +=
			Vector3d(0, -0.001, 0);  // 重力加速度を-0.001とする
#endif


#if 1
									 // ------------------------------------------------------------
									 // Meshless deformation
									 //
									 // 現時点の形状(temporary)に合わせて初期形状(rests[i])を
									 // フィットさせたものをgoalとし、mesh[i]の形状が
									 // goalに戻るように各頂点に力を掛ける
									 //
	for (int i = 0; i < meshes.size(); i++) {
		mesh temporary = meshes[i];

		temporary.vertices += temporary.velocities;  // 現時点の速度で頂点を更新

													 // temporaryに最もフィットするようにrests[i]を回転＆平行移動したものをgoalに求める
		vector<Vector3d> goal = computeGoalPositions(temporary, rests[i]);

		meshes[i].velocities += (goal - temporary.vertices) * 1.0;
		// 1.0は論文中のalpha（物体の柔らかさのパラメータ）に相当
	}
#endif

#if 1
	// ------------------------------------------------------------
	// 衝突判定と衝撃力の作用
	//
	// 全ての衝突が解消されるまで、衝突している頂点の速度の修正を繰り返す
	//
	int collided = 1;
	while (collided) {
		collided = 0;
		for (int i = 0; i < meshes.size(); i++)
			if (!meshes[i].fixed) {
				for (int j = 0; j < meshes.size(); j++) {
					if (i != j) {
						// meshes[i]とmeshes[j]の衝突を検査
						collided = collision(meshes[i], meshes[j]);

						if (collided)
							break;  // 衝突があったので、初めから再検査
					}
				}

				if (collided) break;  // 衝突があったので、初めから再検査
			}
	}
#endif

#if 1
	// ------------------------------------------------------------
	// 現時点の速度で頂点を更新
	//
	for (int i = 0; i < meshes.size(); i++)
		meshes[i].vertices += meshes[i].velocities;
#endif

	// ------------------------------------------------------------
	// 描画処理
	//
	glShadeModel(GL_SMOOTH);  // スムースシューティングを指定

	glClearColor(0, 0, 0, 0);  // 画面クリアに黒色を指定
	glClearDepth(1);           // デプスバッファのクリアに1（最遠値）を指定
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);  // 隠面消去を有効にする

							  // 遠近法の設定
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double a = (double)screenWidth / screenHeight;  // 画面のアスペクト比
	gluPerspective(atan(1 / a / distanceToScreen) / M_PI * 360, a, zNear,
		zFar);  // 視界の設定
	glViewport(0, 0, screenWidth, screenHeight);

	// 照明の姿勢の初期化
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	Vector4f diffuse(1, 1, 1, 1);  // 0番目の照明色に白色を指定
	glLightfv(GL_LIGHT0, GL_DIFFUSE, &diffuse[0]);
	Vector4f position(litEye[0], litEye[1], litEye[2], 1);  // 0番目の照明の位置
	glLightfv(GL_LIGHT0, GL_POSITION, &position[0]);

	glEnable(GL_LIGHTING);  //照明機能を有効にする
	glEnable(GL_LIGHT0);    // 0番目の照明を有効にする

							// カメラの姿勢を初期化
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(camEye[0], camEye[1], camEye[2], camCenter[0], camCenter[1],
		camCenter[2], camUp[0], camUp[1],
		camUp[2]);  // カメラの位置、注視点、上方向を設定

					// 三角形の表面のみ描く（裏面は描かない）
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	// メッシュの三角形を描画
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < meshes.size(); i++)
		for (int j = 0; j < meshes[i].faces.size(); j++)
		{
			vector<Vector3i>& f = meshes[i].faces[j];

			for (int k = 0; k < f.size(); k++)
			{
				glNormal3dv(&meshes[i].normals[f[k][2]][0]);
				glVertex3dv(&meshes[i].vertices[f[k][0]][0]);
			}
		}
	glEnd();

	// 照明を表す黄色いボールを描く
	glDisable(GL_LIGHTING);  //照明機能を無効にする
	glColor4d(1, 1, 0, 1);   // 描画色に黄色を指定
	glPushMatrix();
	glTranslated(litEye[0], litEye[1], litEye[2]);
	glutSolidSphere(0.1, 10, 10);
	glPopMatrix();

	glFinish();  // OpenGL関係の全ての処理の完了を待つ

				 // 20ミリ秒間のウェイト
#ifdef _WIN32
	while (timeGetTime() < prevTime + 20) Sleep(1);
	prevTime = timeGetTime();
#endif

	glutPostRedisplay();  // 再描画
}

// ------------------------------------------------------------
// マウスクリックの処理を行う関数（マウスクリックの度に呼ばれる）
//
void mouse(int b, int s, int x, int y) {
	if (s == GLUT_DOWN) {
		oldX = x;
		oldY = y;

		litMode = (b == GLUT_LEFT_BUTTON);
	}

	if (s == GLUT_UP) oldX = -1;
}

// ------------------------------------------------------------
// 頂点（vertex）をベクトル（axis）の周りにangle（単位はラジアン）だけ回転させる
//
void rotateAroundVector(Vector3d& vertex, Vector3d axis, double angle) {
	axis.normalize();

	double s = sin(angle), c = cos(angle);

	vertex = Vector3d(
		// x
		(axis[0] * axis[0] * (1 - c) + c) * vertex[0] +
		(axis[0] * axis[1] * (1 - c) - axis[2] * s) * vertex[1] +
		(axis[2] * axis[0] * (1 - c) + axis[1] * s) * vertex[2],
		// y
		(axis[0] * axis[1] * (1 - c) + axis[2] * s) * vertex[0] +
		(axis[1] * axis[1] * (1 - c) + c) * vertex[1] +
		(axis[1] * axis[2] * (1 - c) - axis[0] * s) * vertex[2],
		// z
		(axis[2] * axis[0] * (1 - c) - axis[1] * s) * vertex[0] +
		(axis[1] * axis[2] * (1 - c) + axis[0] * s) * vertex[1] +
		(axis[2] * axis[2] * (1 - c) + c) * vertex[2]);
}

// ------------------------------------------------------------
// 姿勢の更新
//
void updatePose(double dx, double dy, Vector3d& eye, Vector3d& center, Vector3d& up) {
	// 視線ベクトルと上方向ベクトルの外積を計算
	Vector3d inverseEyeshot = eye - center;
	Vector3d cross = inverseEyeshot.cross(up);

	// 上方向ベクトルの更新
	rotateAroundVector(up, cross, -dy);

	// 視点の更新
	rotateAroundVector(inverseEyeshot, cross, -dy);
	rotateAroundVector(inverseEyeshot, up, -dx);
	eye = center + inverseEyeshot;
}

// ------------------------------------------------------------
// マウスの動きの処理を行う関数（マウスが動く度に呼ばれる）
//
void motion(int x, int y) {
	if (0 <= oldX) {
		double dx = double(x - oldX) / 100, dy = double(oldY - y) / 100;

		if (litMode)
			// 照明の姿勢を更新
			updatePose(dx, dy, litEye, litCenter, litUp);
		else
			// カメラの姿勢を更新
			updatePose(dx, dy, camEye, camCenter, camUp);

		oldX = x;
		oldY = y;

		glutPostRedisplay();  // 再描画
	}
}

// ------------------------------------------------------------
// メイン関数（ここからプログラムの実行が始まる）
//
int main(int argc, char* argv[]) {
	meshes.resize(5);

	// 立方体
	meshes[0].vertices.resize(8);  // 頂点の指定
	meshes[0].vertices[0] = Vector3d(-1, -1, 1);
	meshes[0].vertices[1] = Vector3d(1, -1, 1);
	meshes[0].vertices[2] = Vector3d(-1, 1, 1);
	meshes[0].vertices[3] = Vector3d(1, 1, 1);
	meshes[0].vertices[4] = Vector3d(-1, 1, -1);
	meshes[0].vertices[5] = Vector3d(1, 1, -1);
	meshes[0].vertices[6] = Vector3d(-1, -1, -1);
	meshes[0].vertices[7] = Vector3d(1, -1, -1);
	meshes[0].normals.resize(6);  // 法線ベクトルの指定
	meshes[0].normals[0] = Vector3d(0, 0, 1);
	meshes[0].normals[1] = Vector3d(0, 1, 0);
	meshes[0].normals[2] = Vector3d(0, 0, -1);
	meshes[0].normals[3] = Vector3d(0, -1, 0);
	meshes[0].normals[4] = Vector3d(1, 0, 0);
	meshes[0].normals[5] = Vector3d(-1, 0, 0);
	meshes[0].faces.resize(12, vector<Vector3i>(3));  // 三角形の指定
	meshes[0].faces[0][0] = Vector3i(0, 0, 0);
	meshes[0].faces[0][1] = Vector3i(1, 0, 0);
	meshes[0].faces[0][2] = Vector3i(2, 0, 0);
	meshes[0].faces[1][0] = Vector3i(2, 0, 0);
	meshes[0].faces[1][1] = Vector3i(1, 0, 0);
	meshes[0].faces[1][2] = Vector3i(3, 0, 0);
	meshes[0].faces[2][0] = Vector3i(2, 0, 1);
	meshes[0].faces[2][1] = Vector3i(3, 0, 1);
	meshes[0].faces[2][2] = Vector3i(4, 0, 1);
	meshes[0].faces[3][0] = Vector3i(4, 0, 1);
	meshes[0].faces[3][1] = Vector3i(3, 0, 1);
	meshes[0].faces[3][2] = Vector3i(5, 0, 1);
	meshes[0].faces[4][0] = Vector3i(4, 0, 2);
	meshes[0].faces[4][1] = Vector3i(5, 0, 2);
	meshes[0].faces[4][2] = Vector3i(6, 0, 2);
	meshes[0].faces[5][0] = Vector3i(6, 0, 2);
	meshes[0].faces[5][1] = Vector3i(5, 0, 2);
	meshes[0].faces[5][2] = Vector3i(7, 0, 2);
	meshes[0].faces[6][0] = Vector3i(6, 0, 3);
	meshes[0].faces[6][1] = Vector3i(7, 0, 3);
	meshes[0].faces[6][2] = Vector3i(0, 0, 3);
	meshes[0].faces[7][0] = Vector3i(0, 0, 3);
	meshes[0].faces[7][1] = Vector3i(7, 0, 3);
	meshes[0].faces[7][2] = Vector3i(1, 0, 3);
	meshes[0].faces[8][0] = Vector3i(1, 0, 4);
	meshes[0].faces[8][1] = Vector3i(7, 0, 4);
	meshes[0].faces[8][2] = Vector3i(3, 0, 4);
	meshes[0].faces[9][0] = Vector3i(3, 0, 4);
	meshes[0].faces[9][1] = Vector3i(7, 0, 4);
	meshes[0].faces[9][2] = Vector3i(5, 0, 4);
	meshes[0].faces[10][0] = Vector3i(6, 0, 5);
	meshes[0].faces[10][1] = Vector3i(0, 0, 5);
	meshes[0].faces[10][2] = Vector3i(4, 0, 5);
	meshes[0].faces[11][0] = Vector3i(4, 0, 5);
	meshes[0].faces[11][1] = Vector3i(0, 0, 5);
	meshes[0].faces[11][2] = Vector3i(2, 0, 5);

	//球の読み込み
#ifdef _WIN32
	meshes[1].loadObj("sphere.obj");
	meshes[2].loadObj("sphere.obj");
	meshes[3].loadObj("sphere.obj");

#endif

#ifdef __APPLE__
	meshes[1].loadObj("~/sphere.obj");
	meshes[2].loadObj("~/sphere.obj");
#endif


	// 外周の壁となる立方体 (meshes[0]を拡大し、全三角形を裏返す)
	meshes[4].vertices = meshes[0].vertices * 8;
	meshes[4].normals = meshes[0].normals * -1;
	meshes[4].faces = meshes[0].faces;
	for (int i = 0; i < meshes[4].faces.size(); i++)
		for (int j = 0; j < meshes[3].faces[i].size(); j++)
			// 三角形を裏返す（頂点の並びを逆順にする）
			meshes[4].faces[i][j] = meshes[0].faces[i][2 - j];
	meshes[4].fixed = 1;

	rests = meshes;

	// 全メッシュの速度の初期化
	for (int i = 0; i < meshes.size(); i++)
		meshes[i].velocities.resize(meshes[i].vertices.size(),
			Vector3d(0, 0, 0));

	// サイズの調整
	meshes[1].vertices = meshes[1].vertices * 2.0;
	meshes[2].vertices = meshes[2].vertices * 0.5;
	meshes[3].vertices = meshes[2].vertices * 1.0;

	// 位置の調整
	meshes[0].vertices += Vector3d(0, 5, 0);
	meshes[1].vertices += Vector3d(3, 5, 1);
	meshes[2].vertices += Vector3d(-4, 5, -3);
	meshes[3].vertices += Vector3d(4, 5, 3);
	

	glutInit(&argc, argv);

#ifdef _WIN32
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
#endif

#ifdef __APPLE__
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
#endif

	glutInitWindowPosition(100, 0);
	glutInitWindowSize(screenWidth, screenHeight);
	glutCreateWindow("Meshless deformation");

	glutDisplayFunc(display);  // 描画処理を行う関数の指定
	glutMouseFunc(mouse);      // マウスクリックの処理を行う関数の指定
	glutMotionFunc(motion);    // マウスの動きの処理を行う関数の指定

	glutMainLoop();  // メインループの開始

	return 0;
}
