# 姿勢推定フィルタの比較シミュレーション

宇宙電波実験室の記事内で掲載したシミュレーションを行うプログラムです。
実装する際の参考にしてください。

該当記事：[動加速度外乱を考慮した姿勢推定（宇宙電波実験室）](https://space-denpa.jp/2022/03/26/omega-ff_dynamic-acc/)

## 内容

各ディレクトリにRustで記述したシミュレーション用プログラムが入っています。

* /omega_ff_normal：[素のOmegaFF](https://space-denpa.jp/2022/03/01/omega-feedback-filter/)
* /omega_ff_e1　　：外乱判定式E1を用いて外乱検知を行うように拡張したOmegaFF
* /omega_ff_e2　　：外乱判定式E2　　　　　　　　　〃

## 使い方

シミュレーションを実行する際は、各ディレクトリに移動して以下のコマンドを実行してください。

```
cargo run && python3 data_plot.py
```
