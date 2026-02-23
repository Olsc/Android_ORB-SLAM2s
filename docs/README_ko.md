![Banner](banner_1.png)

[简体中文](README_zh.md) | [English](../README.md) | [Русский](README_ru.md) | [한국어](README_ko.md) | [日本語](README_ja.md)

## [삼가 고개 숙여 감사] 원본 ORB-SLAM2 코어:

https://github.com/raulmur/ORB_SLAM2

## [삼가 고개 숙여 감사] 원본 Android ORB-SLAM2 프로젝트:

https://github.com/Martin20150405/SLAM_AR_Android.git

# ORB-SLAM2s: Android Spatial Computing & Lightweight AR Demo

*ORB-SLAM2s의 소문자 's'는 **Smart · Swift · Small**(스마트 · 스위프트 · 스몰)을 의미하며, 기존 ORB-SLAM2에 비해 향상된 지능성, 더 빠른 성능, 더 가벼운 무게를 강조합니다.*

이 프로젝트는 소문자 's'를 사용합니다. 대문자 'S' 논문인 ORB-SLAM2S：
```
(Y. Diao, R. Cen, F. Xue and X. Su, "ORB-SLAM2S: A Fast ORB-SLAM2 System with Sparse Optical Flow Tracking," 2021 13th International Conference on Advanced Computational Intelligence (ICACI), Wanzhou, China, 2021, pp. 160-165, doi: 10.1109/ICACI52617.2021.9435915.
keywords: {Visualization;Simultaneous localization and mapping;Cameras;Real-time systems;Aircraft navigation;Central Processing Unit;Trajectory;visual SLAM;real-time performance;trajectory accuracy},)
```
는 본 프로젝트와 유사한 성능 최적화 개념을 공유하지만, 그 방법들은 아직 본 프로젝트에 통합되지 않았습니다. 하지만 이 논문은 향후 최적화 방향에 대한 귀중한 참고 자료가 될 수 있습니다. 연구자들의 기여와 공유에 진심으로 감사드립니다.

## 프로젝트 개요

**ORB-SLAM2s**는 Android용으로 기반한 ORB-SLAM2의 향상된 공간 컴퓨팅 도구입니다. 실시간 희소 포인트 클라우드 매핑, 지도 저장/로드 및 재배치 매칭을 지원합니다. 컴퓨터 비전, 관성 항법(IMU), 증강 현실(AR) 기술을 결합함으로써 이 프로젝트는 모바일 기기용으로 안정적인 **6DoF**(6자유도) 공간 포지셔닝을 제공합니다.

### 왜 ORB-SLAM3 대신 ORB-SLAM2인가요?

* **경량 설계**: 더 적은 의존성과 간소화된 코드로, 모바일 플랫폼에서의 컴파일 및 배포가 더 용이합니다.
* **IMU 및 VIO 지원에 관하여**:
    * **ORB-SLAM3의 높은 장벽**: 공식 구현의 VIO는 **타이트하게 결합(tightly coupled)**되어 있어 고품질 IMU와 엄격한 **카메라-IMU 타임스탬프 동기화**가 필요합니다. 이는 로봇 공학에서는 일반적이지만, Android 휴대폰의 표준 API로는 "즉시 사용(out of the box)"하기 어렵습니다.
    * **Android 현황**: 대부분의 기존 Android 기기는 카메라와 IMU 수집이 비동기식이며, 통합된 하드웨어 동기화 프레임워크가 부족하고, 소비자 등급 IMU는 노이즈/드리프트가 큽니다. 엄격한 보정과 동기화가 없으면 VIO는 순수 시각 SLAM보다 발산하기 쉽습니다. Android 11 이상의 버전에서만 SENSOR_TIMESTAMP 메서드를 사용할 수 있습니다.
* **낮은 리소스 소비**: ORB-SLAM3에 비해 CPU 및 RAM 요구 사항이 낮습니다.
* **실용적인 효율성**: 단안 카메라 전용 시나리오에서 표준 모바일 사용 사례에 대해 ORB-SLAM3와 비교할 만한 성능을 보입니다.

---

## 핵심 기능

* **포인트 클라우드 SLAM 매핑**: 단안 카메라 입력 기반의 실시간 희소 매핑.
* **지도 지속성**: 로컬 저장소에 지도를 저장하고 향후 세션을 위해 다시 로드하는 기능 지원.
* **재로컬라이제이션 및 매칭**: 기존 지도 로드 시 자세 추정 및 특징 매칭.
* **신뢰도 시각화**: 현재 프레임과 로드된 지도 간의 키포인트 시각적 추적 및 매칭 통계.
* **평면 감지**: 현재 자세와 포인트 클라우드 데이터 기반의 지능형 바닥/표면 감지.
* **네이티브 AR 렌더링**: OpenGL ES를 사용한 기본 AR 구현.
* **어두운 프레임 감지**: SLAM 스레드 블로킹을 방지하기 위해 자동으로 어둡거나 저품질 프레임을 건너뜁니다.
* **AR 오브젝트 관리**: 감지된 평면에 3D 오브젝트를 배치하고 상호 작용하는 기능.

---

## 성능 지표

현재 주로 퀄컴 스냅드래곤 플랫폼 CPU에서 테스트 중입니다:

| SoC | 장치 | 성능 |
|-----|--------|-------------|
| Snapdragon 8 Elite | Xiaomi 15 | 30 FPS |
| Snapdragon 8+ Gen1 | Redmi K60 | 30 FPS |
| Snapdragon 870 | Xiaomi 10S | 30 FPS |
| Snapdragon 835 | Xiaomi 6 | 15-20 FPS |

### 어두운 프레임 감지

원활한 사용자 경험을 보장하기 위해 시스템은 노출 수준을 모니터링합니다. 환경이 너무 어두운 경우, SLAM 추적은 계산 지연과 "로스트" 상태를 피하기 위해 일시 중지됩니다.

---

## 진행 상황 추적기

* [x] 희소 포인트 클라우드 SLAM 매핑
* [x] 지도 저장/로드 기능
* [x] 재로컬라이제이션 매칭
* [x] 기본 AR 렌더링 엔진
* [x] 어두운 프레임 건너뛰기 로직
* [x] 3D AR 오브젝트 관리
* [x] 다중 지도 파일의 동시 로드 및 매칭

## 향후 로드맵

* [ ] 매핑 속도 및 초기화 개선.
* [ ] AR 안정성 및 6DoF 강건성 향상.
* [ ] 더 높은 프레임 속도를 위한 렌더링 파이프라인 최적화.
* [ ] 센서 퓨전 심화(VIO - 시각 관성 주행 거리 측정).
* [ ] **Unity3D** 통합.
* [ ] SLAM 주파수 다운샘플링 및 적응형 노이즈 처리.
* [ ] 정밀 센서 게이팅 로직.

---

## 감사의 말

이 프로젝트는 다음과 같은 훌륭한 오픈 소스 라이브러리들을 기반으로 구축되었습니다:

* [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
* [DBoW2](https://github.com/dorian3d/DBoW2)
* [g2o](https://github.com/RainerKuemmerle/g2o)
* [Eigen](http://eigen.tuxfamily.org/)
* [OpenCV](https://opencv.org/)

---

# ORB-SLAM2
**저자:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) 및 [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

ORB-SLAM2는 **단안**, **스테레오** 및 **RGB-D** 카메라용 실시간 SLAM 라이브러리로, 카메라 궤적과 희소 3D 재구성을 계산합니다(스테레오 및 RGB-D의 경우 실제 스케일로). 루프를 감지하고 카메라를 실시간으로 재로컬라이즈할 수 있습니다. [KITTI 데이터셋](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)에서 스테레오 또는 단안으로, [TUM 데이터셋](http://vision.in.tum.de/data/datasets/rgbd-dataset)에서 RGB-D 또는 단안으로, [EuRoC 데이터셋](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)에서 스테레오 또는 단안으로 SLAM 시스템을 실행하는 예제를 제공합니다.

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>


### 관련 출판물:

[단안] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[스테레오 및 RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. 라이선스

## ORB-SLAM2 코어 라이브러리

ORB-SLAM2 코어 라이브러리는 [GPLv3 라이선스](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt)에 따라 배포됩니다. 모든 코드/라이브러리 종속성(및 관련 라이선스) 목록은 [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md)를 참조하십시오.

상업적 목적으로 사용하는 폐쇄형 버전의 ORB-SLAM2가 필요한 경우, 저자에게 문의하십시오: orbslam (at) unizar (dot) es.

## 이 프로젝트 (ORB-SLAM2s)

이 Android 어댑테이션 및 강화 프로젝트(ORB-SLAM2s)는 **GPL-3.0 라이선스**에 따라 라이선스가 부여됩니다. 자세한 내용은 [LICENSE.txt](../LICENSE.txt) 및 [License-gpl.txt](../License-gpl.txt) 파일을 참조하십시오.
<br><br>프로젝트 협업 또는 기타 분야 협력 문의는 OlscStudio@outlook.com으로 연락 주십시오.

## 학술적 인용

학술 작업에서 ORB-SLAM2(단안)를 사용하는 경우, 다음을 인용하십시오:

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

학술 작업에서 ORB-SLAM2(스테레오 또는 RGB-D)를 사용하는 경우, 다음을 인용하십시오:

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

이 Android 어댑테이션(ORB-SLAM2s)을 학술 작업에서 사용하는 경우, 적절히 이 오픈소스 URL을 기재하십시오.

# 2. 필수 조건

## OpenCV
이미지 및 특징을 조작하기 위해 [OpenCV](http://opencv.org)를 사용합니다. 다운로드 및 설치 지침은 다음에서 찾을 수 있습니다: http://opencv.org. **최소 4.5.0 이상 필요**.

## Eigen3
g2o에 필요합니다(아래 참조). 다운로드 및 설치 지침은 다음에서 찾을 수 있습니다: http://eigen.tuxfamily.org.

## DBoW2 및 g2o (Thirdparty 폴더에 포함됨)
[DBoW2](https://github.com/dorian3d/DBoW2) 및 [g2o](https://github.com/RainerKuemmerle/g2o) 라이브러리를 알고리즘 참조용으로 사용합니다. 두 라이브러리(라이선스 포함)는 *Thirdparty* 폴더에 포함되어 있습니다.

다국어 번역은 Qwen3에 의해 제공되었습니다. 오류가 있을 수 있으므로 양해 부탁드리며, 오류가 발견되면 문제를 제출해 주시기 바랍니다.

# 관성 항법(IMU) 정보
참고: https://github.com/Olsc/Android_3dof <br>
참고: https://github.com/ZUXTUO/Android_6dof <br>
이 프로젝트는 아직 연구 통합이 완료되지 않았습니다.

<p align="center">
  <img src="./aesthetic_visual_causal_flow_en.svg" alt="visual_causal_flow">
</p>

---
<br>
<br>

# ♥ 기여자

[![Contributors](https://contrib.rocks/image?repo=Olsc/Android_ORB-SLAM2s)](https://github.com/Olsc/Android_ORB-SLAM2s/graphs/contributors)
