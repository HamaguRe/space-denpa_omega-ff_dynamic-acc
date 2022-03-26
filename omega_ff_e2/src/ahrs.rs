//! 姿勢推定フィルタ

use super::DT;
use super::quat;
use super::quat::{Vector3, Quaternion};

/// 標準重力
pub const STANDARD_GRAVITY: f64 = 9.80665;

/// 基準座標系上における加速度計測値
pub const ACC_R: [f64; 3] = [0.0, 0.0, STANDARD_GRAVITY];

/// 基準座標系上における地磁気計測値
pub const MAG_R: [f64; 3] = [0.0, 1.0, 0.0];

/// 外乱検知判定のヒステリシス
const HYSTERESIS: f64 = 0.2;

pub struct AttitudeFilter {
    pub q: Quaternion<f64>,      // 姿勢推定値
    gyr_correct: Vector3<f64>,   // 補正角速度（角速度バイアスの推定値を含む）
    coef_gyr_c: f64,             // 補正角速度を計算するときのパラメータ
    coef_integ: f64,             // 補正角速度の積分係数
    pub gyr_integ: Vector3<f64>, // 補正角速度の積分項
    thr_weak: f64,               // 弱い外乱判定の閾値
    thr_strong: f64,             // 強い外乱判定の閾値
    flag_acc_weak: bool,    // ヒステリシス処理に使う変数
    flag_acc_strong: bool,  // ヒステリシス処理に使う変数
}

impl AttitudeFilter {
    /// * alpha : 基準姿勢に収束するまでの時間[s]
    /// * beta  : 補正角速度の積分係数
    /// * thr_weak  : 弱い外乱判定の閾値（< thr_strong）
    /// * thr_strong: 強い外乱判定の閾値（> thr_weak）
    pub fn new(alpha: f64, beta: f64, thr_weak: f64, thr_strong: f64) -> Self {
        Self {
            q: (1.0, [0.0; 3]),
            gyr_correct: [0.0; 3],
            coef_gyr_c: 2.0 / alpha,
            coef_integ: beta,
            gyr_integ: [0.0; 3],
            thr_weak: thr_weak,
            thr_strong: thr_strong,
            flag_acc_weak: false,
            flag_acc_strong: false,
        }
    }

    /// 予測ステップ
    /// 
    /// * gyr: 機体上で計測した角速度[rad/s]
    pub fn predict(&mut self, gyr: Vector3<f64>) {
        let omega = quat::add_vec(gyr, self.gyr_correct);

        // 積分（q[n+1] = q[n] + Δt/2 *q[n]*ω[n]）
        let tmp0 = quat::scale_vec(self.q.0, omega);
        let dot = quat::dot_vec(self.q.1, omega);
        let cross = quat::cross_vec(self.q.1, omega);
        let tmp1 = (-dot, quat::add_vec(tmp0, cross));
        self.q = quat::scale_add(0.5 * DT, tmp1, self.q);
        // 正規化
        self.q = quat::normalize(self.q);
    }

    /// 補正ステップ（外乱検知も行う）
    /// 
    /// * acc: 機体上のセンサで計測した加速度[m/s^2]
    /// * mag: 機体上のセンサで計測した地磁気（方向だけわかれば良いので単位不問）
    pub fn correct(&mut self, mut acc: Vector3<f64>, mag: Vector3<f64>) {
        let mut coef = self.coef_gyr_c;

        // 加速度外乱検知
        let acc_q = quat::frame_rotation(self.q, ACC_R);
        let e = quat::norm_vec( quat::sub_vec(acc, acc_q) ) / STANDARD_GRAVITY;  // E2
        if e > self.thr_strong {
            // 強い外乱なので，加速度による補正をストップする．
            self.flag_acc_strong = true;
            acc = acc_q;
        } else if e > self.thr_weak {
            // ヒステリシス処理：強い外乱 -> 弱い外乱
            if self.flag_acc_strong && e > (self.thr_strong - self.thr_strong * HYSTERESIS) {
                acc = acc_q;
            } else {
                // 弱い外乱なので，補正角速度の重みを変更．
                self.flag_acc_strong = false;
                self.flag_acc_weak = true;
                coef *= 0.5;
            }
        } else {
            // ヒステリシス処理：弱い外乱 -> 外乱無し
            if self.flag_acc_weak && e > (self.thr_weak - self.thr_weak * HYSTERESIS) {
                coef *= 0.5;
            } else {
                self.flag_acc_weak = false;
                self.flag_acc_strong = false;
            }
        }

        // accとmagから姿勢q_gmを計算
        let q_gm = get_q_gm(acc, mag);

        // qからq_gmに到達するための角速度を計算
        let term1 = quat::scale_vec(self.q.0, q_gm.1);
        let term2 = quat::scale_vec(q_gm.0, self.q.1);
        let term3 = quat::cross_vec(q_gm.1, self.q.1);
        self.gyr_correct = quat::scale_vec(coef, quat::add_vec(quat::sub_vec(term1, term2), term3));
        // 符号をqに合わせる
        if quat::dot(self.q, q_gm).is_sign_negative() {
            self.gyr_correct = quat::negate_vec(self.gyr_correct);
        }

        // 積分項を更新
        self.gyr_integ = quat::scale_add_vec(DT, self.gyr_correct, self.gyr_integ);

        // 積分項の値を補正角速度に反映
        self.gyr_correct = quat::scale_add_vec(self.coef_integ, self.gyr_integ, self.gyr_correct);
    }
}

// 加速度に外乱が入っていなければ良いが、外乱がある場合地磁気の伏角除去に影響が出る。
/// 機体座標系上で計測した加速度と地磁気ベクトルから，基準座標系に対する姿勢を計算する．
pub fn get_q_gm(acc: Vector3<f64>, mag: Vector3<f64>) -> Quaternion<f64> {
    let q_g = quat::rotate_a_to_b(acc, ACC_R);
    let mag_b2r = quat::hadamard_vec(quat::vector_rotation(q_g, mag), [1.0, 1.0, 0.0]);
    let q_e = quat::rotate_a_to_b(mag_b2r, MAG_R);
    quat::mul(q_e, q_g)
}