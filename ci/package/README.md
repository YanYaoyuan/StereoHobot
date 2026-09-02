# StereoInfer S100 部署包

## 目录结构

- `bin/`：ARM64 可执行程序
- `lib/`：OpenCV 4.5.4 与 D-Robotics UCP 运行库
- `config/`：S100 HBM 模型
- `samples/img/`：双目图片和相机内参样例
- `scripts/`：运行入口
- `result/`：默认结果目录

## 使用

```bash
cd StereoInfer_S100
source setup_env.sh

scripts/run_infer.sh \
  config/dstereo_s100_320_640_352_v2.4.hbm \
  samples/img \
  0.10

scripts/run_test_perf.sh \
  config/dstereo_s100_320_640_352_v2.4.hbm \
  1 30 0.10
```

程序也带有相对 RPATH，可直接执行 `bin/infer` 或 `bin/test_perf`。
