//! 角速度をフィードバックする形で補正を行う姿勢推定フィルタ
//! 
//! 外乱検知式にE1を使用

use std::fs;
use std::io::{Write, BufWriter};
use std::mem::MaybeUninit;

use rand::distributions::{Distribution, Normal};
use quaternion as quat;

mod ahrs;

const DT: f64 = 0.02;
const SIM_TIME: f64 = 30.0;
const N: usize = (SIM_TIME / DT) as usize + 1;

/// 角速度センサのノイズ分散
const GYR_VAR: f64 = 0.0001;

/// 加速度センサのノイズ分散
const ACC_VAR: f64 = 0.01;

/// 地磁気センサのノイズ分散
const MAG_VAR: f64 = 0.01;

fn main() {
    // CSVファイルにデータ保存（同一ファイルが存在したら上書き）
    let mut file = BufWriter::new( fs::File::create("result.csv").unwrap() );

    // 標準正規分布の乱数を生成
    let randn = Normal::new(0.0, 1.0);  // 平均値:0，標準偏差:1

    // 姿勢推定フィルタ
    let beta = 0.2;
    let mut filter = ahrs::AttitudeFilter::new(1.0, beta, 0.04, 0.08);

    let mut q = (1.0, [0.0; 3]);
    //q = quat::normalize((0.0, [1.0, -0.5, 1.5]));  // 初期値をずらす
    let gyr_bias = [-0.02, 0.01, 0.05];
    let mut a_dr = [0.0; 3];  // センサに直接加わる加速度外乱

    // ---- Loop start ---- //
    let gyr = [0.1; 3];
    for t in 0..N {
        let time = t as f64 * DT;

        // 加速度外乱印加
        if time >= 10.0 && time <= 20.0 {
            //a_dr[0] = 0.5 * (time * 5.0).sin() + 1.0;
            a_dr[0] = 3.0;
        } else {
            a_dr[0] = 0.0;
        }

        // 積分（q = q + 0.5*Δt*q*ω）
        q = {
            let tmp0 = quat::scale_vec(q.0, gyr);
            let dot = quat::dot_vec(q.1, gyr);
            let cross = quat::cross_vec(q.1, gyr);
            let tmp1 = (-dot, quat::add_vec(tmp0, cross));
            quat::scale_add(0.5 * DT, tmp1, q)
        };
        q = quat::normalize(q);

        // 計測値生成
        let mut acc_b = quat::frame_rotation(q, ahrs::ACC_R);
        let mut mag_b = quat::frame_rotation(q, ahrs::MAG_R);
        acc_b = add_noise(&randn, ACC_VAR, acc_b);
        mag_b = add_noise(&randn, MAG_VAR, mag_b);

        // 外乱を加える
        acc_b = quat::add_vec(acc_b, a_dr);

        // 推定
        let gyr_noisy = add_noise(&randn, GYR_VAR, gyr);
        filter.predict( quat::add_vec(gyr_noisy, gyr_bias) );
        filter.correct(acc_b, mag_b);

        // ---------- データ書き込み ---------- //
        // 時刻
        file.write( format!("{:.3},", t as f64 * DT ).as_bytes() ).unwrap();
        // オイラー角の真値
        let ypr_true = quat::to_euler_angles( q );
        for i in 0..3 {
            file.write( format!("{:.7},", ypr_true[i] ).as_bytes() ).unwrap();
        }
        // オイラー角の推定値
        let ypr_hat = quat::to_euler_angles( filter.q );
        for i in 0..3 {
            file.write( format!("{:.7},", ypr_hat[i] ).as_bytes() ).unwrap();
        }
        // 角速度バイアスの真値
        for i in 0..3 {
            file.write( format!("{:.7},", gyr_bias[i] ).as_bytes() ).unwrap();
        }
        // 角速度バイアスの推定値（補正の仕方の問題で符号が反転している）
        for i in 0..3 {
            file.write( format!("{:.7},", -beta * filter.gyr_integ[i] ).as_bytes() ).unwrap();
        }
        // 四元数の真値
        file.write( format!("{:.7},", q.0 ).as_bytes() ).unwrap();
        for i in 0..3 {
            file.write( format!("{:.7},", q.1[i] ).as_bytes() ).unwrap();
        }
        // 四元数の推定値
        file.write( format!("{:.7},", filter.q.0 ).as_bytes() ).unwrap();
        for i in 0..3 {
            file.write( format!("{:.7},", filter.q.1[i] ).as_bytes() ).unwrap();
        }
        // 加速度外乱の真値
        for i in 0..3 {
            file.write( format!("{:.7},", a_dr[i] ).as_bytes() ).unwrap();
        }
        // 外乱検出の誤差関数
        let e = ( quat::norm_vec(acc_b) - ahrs::STANDARD_GRAVITY ).abs() / ahrs::STANDARD_GRAVITY;  // E1
        file.write( format!("{:.7}\n", e).as_bytes() ).unwrap();
        // ------------------------------------ //
    }
}

/// ベクトルxにノイズを加える．
fn add_noise(randn: &rand::distributions::Normal, variance: f64, x: quat::Vector3<f64>) -> quat::Vector3<f64> {
    let mut noisy: quat::Vector3<f64> = unsafe {MaybeUninit::uninit().assume_init()};

    let tmp = variance.sqrt();
    for i in 0..3 {
        noisy[i] = x[i] + randn.sample(&mut rand::thread_rng()) * tmp;
    }
    noisy
}