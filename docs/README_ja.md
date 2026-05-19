![Banner](banner_1.png)

[简体中文](README_zh.md) | [English](../README.md) | [Русский](README_ru.md) | [한국어](README_ko.md) | [日本語](README_ja.md)

## 【謹んで感謝】元のORB-SLAM2コア:

https://github.com/raulmur/ORB_SLAM2

## 【謹んで感謝】元のAndroid ORB-SLAM2プロジェクト:

https://github.com/Martin20150405/SLAM_AR_Android.git

# ORB-SLAM2s: Android Spatial Computing & Lightweight AR Demo

_ORB-SLAM2sの小文字の's'は**Smart · Swift · Small**（スマート・迅速・小型）を意味し、元のORB-SLAM2と比較して強化された知能性、より高速なパフォーマンス、より軽量なサイズを強調しています。_

このプロジェクトでは小文字の's'を使用しています。大文字の'S'を持つ別の論文ORB-SLAM2S：

```
（Y. Diao, R. Cen, F. Xue and X. Su, "ORB-SLAM2S: A Fast ORB-SLAM2 System with Sparse Optical Flow Tracking," 2021 13th International Conference on Advanced Computational Intelligence (ICACI), Wanzhou, China, 2021, pp. 160-165, doi: 10.1109/ICACI52617.2021.9435915.
keywords: {Visualization;Simultaneous localization and mapping;Cameras;Real-time systems;Aircraft navigation;Central Processing Unit;Trajectory;visual SLAM;real-time performance;trajectory accuracy},）
```

は、このプロジェクトと同様の性能最適化のコンセプトを持っていますが、その手法はまだこのプロジェクトに統合されていません。しかし、この論文は今後の最適化方向性に対する貴重な参考資料となるでしょう。研究者の皆様の貢献と共有に心より感謝いたします。

## プロジェクト概要

**ORB-SLAM2s** は、Android向けにORB-SLAM2をベースにした強化型空間計算ツールです。リアルタイムのスパースポイントクラウドマッピング、マップの保存/読み込み、および再配置マッチングをサポートしています。コンピュータビジョン、慣性航法（IMU）、および拡張現実（AR）技術を組み合わせることで、このプロジェクトはモバイルデバイス向けに安定した**6DoF**（6自由度）空間ポジショニングを提供します。

### なぜORB-SLAM3ではなくORB-SLAM2なのか？

- **軽量設計**: 依存関係が少なく、コードが簡素化されているため、モバイルプラットフォームでのコンパイルと展開が容易です。
- **IMUとVIOのサポートについて**:
  - **ORB-SLAM3の高いハードル**: 公式実装のVIOは**密結合**であり、高品質なIMUと厳密な**カメラ-IMU間のタイムスタンプ同期**が必要です。これはロボット工学では一般的ですが、Android携帯電話の標準APIでは「すぐに使える」状態にするのは困難です。
  - **Androidの現状**: ほとんどの既存のAndroidデバイスはカメラとIMUの取得が非同期であり、統一されたハードウェア同期フレームワークが不足しており、民生グレードのIMUはノイズやドリフトが大きいです。厳密なキャリブレーションと同期がない場合、VIOは純粋なVisual SLAMよりも発散しやすくなります。Android 11以降のバージョンでのみSENSOR_TIMESTAMPメソッドが利用可能です。
- **低リソース消費**: ORB-SLAM3と比較してCPUおよびRAMの要求が低いです。
- **実用的な効率**: 単眼カメラのみのシナリオでは、標準的なモバイル用途においてORB-SLAM3と同等のパフォーマンスを発揮します。

---

## 主要機能

- **ポイントクラウドSLAMマッピング**: 単眼カメラ入力に基づくリアルタイムスパースマッピング。
- **マップ永続化**: マップをローカルストレージに保存し、将来のセッションで再読み込みする機能をサポート。
- **再ローカライゼーションとマッチング**: 既存マップ読み込み時のポーズ推定と特徴マッチング。
- **信頼度の可視化**: キーポイントの視覚的追跡と、現在のフレームと読み込まれたマップ間のマッチング統計。
- **平面検出**: 現在のポーズとポイントクラウドデータに基づいたインテリジェントな床/表面検出。
- **ネイティブARレンダリング**: OpenGL ESを使用した基本的なAR実装。
- **暗所フレーム検出**: SLAMスレッドのブロックを防ぐために、暗いまたは低品質のフレームを自動的にスキップ。
- **ARオブジェクト管理**: 検出された平面上に3Dオブジェクトを配置および操作する機能。

---

## パフォーマンス指標

現在主にQualcomm SnapdragonプラットフォームCPUでテストしています:

| SoC                  | デバイス      | パフォーマンス |
| -------------------- | ------------- | -------------- |
| Snapdragon 8 Elite   | Xiaomi 15     | 30 FPS         |
| Snapdragon 8+ Gen1   | Redmi K60     | 30 FPS         |
| Snapdragon 870       | Xiaomi 10S    | 30 FPS         |
| Snapdragon 835       | Xiaomi 6      | 15-20 FPS      |
| Snapdragon AR1 Gen 1 | Rokid Glasses | 10-15 FPS      |

### 暗所フレーム検出

スムーズなユーザーエクスペリエンスを確保するために、システムは露出レベルを監視します。環境が暗すぎる場合、SLAM追跡は一時停止され、計算遅延や「ロスト」状態を回避します。

---

## 進捗トラッカー

- [x] スパースポイントクラウドSLAMマッピング
- [x] マップ保存/読み込み機能
- [x] 再ローカライゼーションマッチング
- [x] 基本ARレンダリングエンジン
- [x] 暗所フレームスキップロジック
- [x] 3D ARオブジェクト管理
- [x] 複数マップファイルの同時読み込みとマッチング
- [x] **Unity3D**との統合。

## 今後のロードマップ

- [ ] マッピング速度と初期化の改善。
- [ ] ARの安定性と6DoFの堅牢性の強化。
- [ ] より高いフレームレートのためのレンダリングパイプラインの最適化。
- [ ] センサーフュージョンの深化（VIO - Visual Inertial Odometry）。
- [ ] SLAM周波数のダウンサンプリングと適応ノイズ処理。
- [ ] 精密なセンサーチェッキングロジック。

## 謝辞

このプロジェクトは、以下の優れたオープンソースライブラリに基づいて構築されています:

- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
- [DBoW2](https://github.com/dorian3d/DBoW2)
- [g2o](https://github.com/RainerKuemmerle/g2o)
- [Eigen](http://eigen.tuxfamily.org/)
- [OpenCV](https://opencv.org/)

---

# ORB-SLAM2

**著者:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) および [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

ORB-SLAM2は、**単眼**、**ステレオ**、および**RGB-D**カメラ用のリアルタイムSLAMライブラリであり、カメラの軌跡とスパース3D再構成（ステレオおよびRGB-Dでは真のスケールで）を計算します。ループを検出し、リアルタイムでカメラを再ローカライズできます。[KITTIデータセット](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)でステレオまたは単眼として、[TUMデータセット](http://vision.in.tum.de/data/datasets/rgbd-dataset)でRGB-Dまたは単眼として、[EuRoCデータセット](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)でステレオまたは単眼としてSLAMシステムを実行する例を提供しています。

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>

### 関連出版物:

[単眼] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. _IEEE Transactions on Robotics,_ vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[ステレオおよびRGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. _IEEE Transactions on Robotics,_ vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. _IEEE Transactions on Robotics,_ vol. 28, no. 5, pp. 1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. ライセンス

## ORB-SLAM2コアライブラリ

ORB-SLAM2コアライブラリは[GPLv3ライセンス](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt)の下でリリースされています。すべてのコード/ライブラリ依存関係（および関連ライセンス）のリストについては、[Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md)を参照してください。

商用目的での非オープンソース版ORB-SLAM2については、著者に連絡してください: orbslam (at) unizar (dot) es。

## このプロジェクト (ORB-SLAM2s)

このAndroidアダプテーションおよび強化プロジェクト（ORB-SLAM2s）も**GPL-3.0ライセンス**の下でライセンスされています。詳細については、[LICENSE.txt](../LICENSE.txt)および[License-gpl.txt](../License-gpl.txt)ファイルを参照してください。
<br><br>プロジェクトコラボレーションまたはその他の分野の協力に関するお問い合わせは、OlscStudio@outlook.comまでご連絡ください。

## 学術的引用

学術的研究でORB-SLAM2（単眼）を使用する場合は、以下のように引用してください:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

学術的研究でORB-SLAM2（ステレオまたはRGB-D）を使用する場合は、以下のように引用してください:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }

このAndroidアダプテーション（ORB-SLAM2s）を学術的研究で使用する場合は、適切にこのオープンソースURLを記載してください。

# 2. 事前条件

## OpenCV

画像と特徴の操作には[OpenCV](http://opencv.org)を使用しています。ダウンロードおよびインストール手順は以下にあります: http://opencv.org **最低4.5.0以上が必要です**。

## Eigen3

g2oに必要です（下記参照）。ダウンロードおよびインストール手順は以下にあります: http://eigen.tuxfamily.org 。

## DBoW2およびg2o（Thirdpartyフォルダに含まれています）

[DBoW2](https://github.com/dorian3d/DBoW2)および[g2o](https://github.com/RainerKuemmerle/g2o)ライブラリをアルゴリズム参照として使用しています。両方のライブラリ（ライセンスを含む）は*Thirdparty*フォルダに含まれています。

多言語翻訳はQwen3によって提供されています。誤りがあってもご容赦ください。誤りが見つかった場合は問題を提出してください。

# 慣性航法（IMU）について

参考：https://github.com/Olsc/Android_3dof <br>
参考：https://github.com/ZUXTUO/Android_6dof <br>
本プロジェクトはまだ研究統合が完了していません。

<p align="center">
  <img src="./aesthetic_visual_causal_flow_en.svg" alt="visual_causal_flow">
</p>

---

<br>
<br>

# ♥ 貢献者

[![Contributors](https://contrib.rocks/image?repo=Olsc/Android_ORB-SLAM2s)](https://github.com/Olsc/Android_ORB-SLAM2s/graphs/contributors)
