# ORB-SLAM2s Privacy Policy and Terms of Use

**Last Updated: August 17, 2026**

---

## 1. Project Overview

**ORB-SLAM2s** is an Android-based enhanced spatial computing and lightweight Augmented Reality (AR) demo software built upon ORB-SLAM2. The lowercase 's' in ORB-SLAM2s stands for **Smart · Swift · Small**, emphasizing enhanced intelligence, faster performance, and lighter weight compared to the original ORB-SLAM2.

This project combines computer vision-based Simultaneous Localization and Mapping (SLAM) technology with Augmented Reality (AR) rendering capabilities to provide **6-Degrees-of-Freedom (6DoF) spatial positioning** on mobile devices. Core features include:

1. **Sparse Point Cloud SLAM Mapping**: Real-time extraction of ORB features from monocular camera input to construct sparse 3D point cloud maps of the environment;
2. **Map Persistence**: Saving constructed maps as binary files (.bin) with associated metadata (.json) to local device storage, and reloading them for future sessions;
3. **Relocalization & Matching**: Recovering camera pose through feature matching when loading existing maps;
4. **Confidence Visualization**: Visual tracking of keypoints and matching statistics between current frames and the loaded map;
5. **Plane Detection**: Intelligent detection of horizontal floors/surfaces based on current camera pose and point cloud data;
6. **Native AR Rendering**: 3D rendering via **Google Filament** (GLB/glTF format models) and OpenGL ES 2.0, supporting placement and interaction of virtual objects on detected planes;
7. **Dark Frame Detection**: Automatic detection of overly dark environments, pausing SLAM tracking to prevent wasted computational resources and tracking loss;
8. **3D Object Interaction Management**: Support for placing, scaling (pinch gesture), and interacting with 3D AR objects;
9. **3DOF Orientation Tracking**: Three-degrees-of-freedom orientation tracking using onboard device sensors (Rotation Vector Sensor / Accelerometer + Magnetometer);
10. **Multi-Map Support**: Simultaneous loading, matching, and management of multiple map files;
11. **Fully Decoupled IPC Architecture & Open Secondary Development**: The core SLAM computation engine is completely decoupled into an isolated background process service (`:slam_process`), communicating with UI and rendering pipelines via zero-copy shared memory (SharedMemory/Ashmem double-buffering) and asynchronous AIDL IPC. This architecture fully separates computation from rendering, exposes a clean, standardized IPC client interface, and enables secondary editing, custom algorithm backend replacements, and third-party host engine integrations.

This project **only** runs on the Android mobile platform. Its underlying SLAM engine is implemented in C/C++ (running in the isolated `:slam_process` service), while the upper layer uses Java/Kotlin for user interface and AR rendering pipelines, coordinating via high-performance local Inter-Process Communication (IPC).

---

## 2. User Privacy Consent Mechanism

The application includes a built-in privacy consent screen (`PrivacyConsentActivity`) that is displayed when the app is launched for the first time. The consent process works as follows:

1. **First Launch**: On initial startup, the app loads this Privacy Policy (selecting the appropriate language — Chinese or English — based on the device's system language setting).
2. **Scroll-to-Read Requirement**: The user must scroll through the full policy document. The "Agree" button remains disabled (grayed out) until the bottom of the document has been reached, ensuring the user has the opportunity to read the complete terms before giving consent.
3. **Consent Persistence**: Once the user taps "Agree", their consent is saved locally on the device via `SharedPreferences`. On subsequent launches, the privacy screen is skipped automatically.
4. **User Control**: If the user does not agree to the terms, they can exit the application by pressing the system back button.
5. **No Data Transmission**: This consent mechanism runs entirely on the local device. No consent status or any other data is transmitted to any remote server.

---

## 3. Open Source License

### 3.1 Project License

ORB-SLAM2s (this Android adaptation and enhancement project) is released under the **GNU General Public License v3.0 (GPL-3.0)**.

- License file copies: [LICENSE.txt](https://github.com/Olsc/Android_ORB-SLAM2s/blob/main/LICENSE.txt) and [License-gpl.txt](https://github.com/Olsc/Android_ORB-SLAM2s/blob/main/License-gpl.txt)
- Official GPL-3.0 full text: https://www.gnu.org/licenses/gpl-3.0.html

Under the terms of the GPL-3.0 License:

- **You are free to**: copy, distribute, and modify this software.
- **You must comply with**:
  - Any modified version or derivative work must also be distributed under GPL-3.0;
  - You must provide a notice of modifications;
  - You must provide complete source code with distribution;
  - You must retain copyright notices and license statements in the software;
  - **This software is provided "AS IS" without any warranty, express or implied** (see Section 15 Disclaimer and Section 16 Limitation of Liability in the GPL-3.0).

### 3.2 Upstream Dependency Licenses

This project is built upon the following open-source libraries, each with its own independent license:

| Component                       | License                                 | Description                                                                                 |
| ------------------------------- | --------------------------------------- | ------------------------------------------------------------------------------------------- |
| ORB-SLAM2 Core                  | GPL-3.0                                 | Original SLAM library by Raul Mur-Artal et al.                                              |
| DBoW2 (modified)                | Modified BSD (with notification clause) | Bag-of-Words library for place recognition                                                  |
| g2o (modified)                  | BSD 2-Clause (core)                     | Graph optimization library for non-linear optimization (some components GPL-3.0/LGPL-3.0)   |
| Eigen3                          | MPL-2.0 (mostly)                        | Linear algebra library (3.4+ portions also Apache-2.0/BSD-3-Clause)                         |
| OpenCV                          | Apache 2.0                              | Computer vision library (some files BSD-3-Clause)                                           |
| AndroidX / CameraX              | Apache 2.0                              | Official Android camera and UI components                                                   |
| Google Material Design          | Apache 2.0                              | UI design library                                                                           |
| **srrg_hbst (HBST)**            | **BSD 3-Clause**                        | **Hierarchical Bag of Scalable Trees — fast incremental image matching for relocalization** |
| **Google Filament**             | **Apache 2.0**                          | **Physically-based 3D rendering engine for AR object display (GLB/glTF model support)**     |
| Google Guava                    | Apache 2.0                              | Java core library extensions                                                                |
| **Markwon (io.noties.markwon)** | **Apache 2.0**                          | **Markdown rendering library — used to display this Privacy Policy within the app**         |

For closed-source commercial licensing inquiries of ORB-SLAM2, please contact the original authors (orbslam@unizar.es).

For project collaboration or other field cooperation inquiries, please contact: OlscStudio@outlook.com

---

## 4. Permissions Requested and Usage Description

The following permissions are declared in AndroidManifest.xml. Each permission serves a specific and necessary purpose:

### 4.1 `android.permission.CAMERA`

- **Purpose**: Access the device rear camera to capture real-time video frames
- **Usage Location**: `CameraGLView.java` → `CameraX` framework → `ImageAnalysis.Analyzer`
- **Necessity**: **Core required permission**. SLAM system operation depends on real-time camera image acquisition for ORB feature extraction, camera trajectory estimation, and 3D point cloud map construction. Without this permission, SLAM and AR functionality cannot function.
- **Data Scope**: Only real-time video frames are acquired for image processing (grayscale and RGBA). No recording, uploading, or persistent storage of video streams occurs. Frame data exists temporarily in memory and is released immediately after processing.
- **Usage Scenario**: Throughout SLAM initialization, real-time tracking, plane detection, and AR rendering.
- **Rear Camera Only**: Uses only `CameraSelector.LENS_FACING_BACK` (rear camera); does not access the front camera.

### 4.2 `android.permission.READ_EXTERNAL_STORAGE`

- **Purpose**: Read saved SLAM map files, vocabulary files, and camera configuration files
- **Usage Location**: `NativeHelper.java` — `initSLAM()`, `loadMap()`, `loadMapWithId()`; `ZipHelper.java`
- **Necessity**: **Functionally required**. The app needs to read the ORB vocabulary file (`ORBvoc.txt.arm.bin`) and camera parameter configuration (`CameraSettings.yaml`) at startup; loading previously saved maps requires reading `.bin` and `.json` files.
- **Limitation**: `android:maxSdkVersion="32"` — Android 13+ (API 33+) will no longer grant this permission; the app will use scoped storage (`getExternalFilesDir()`) instead.
- **Storage Path**: Only accesses files under `getExternalFilesDir("SLAM")`; does not read other user private files.

### 4.3 `android.permission.WRITE_EXTERNAL_STORAGE`

- **Purpose**: Save SLAM map files (.bin) and metadata files (.json) to external storage
- **Usage Location**: `NativeHelper.MapManager.saveMap()`
- **Necessity**: **Functionally required**. When the user triggers "Save Map", the currently constructed sparse point cloud map is serialized and written to storage along with metadata (keyframe count, map point count, creation time, etc.).
- **Limitation**: `android:maxSdkVersion="32"` — Same as read permission; higher Android versions use scoped storage.
- **Written Content**: Only `.bin` (serialized map data) and `.json` (metadata description) file types.
- **Storage Path**: Only writes to the `getExternalFilesDir("SLAM/maps")` directory; does not modify other user files.

---

## 5. Hardware Feature Usage

### 5.1 Camera Hardware

| Item              | Description                                                                                                                                        |
| ----------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Declaration**   | `<uses-feature android:name="android.hardware.camera" />`                                                                                          |
| **Purpose**       | Primary input source for SLAM real-time tracking and AR rendering. Resolution is dynamically computed (base 1280x720, maintain 16:9 aspect ratio). |
| **Data Pipeline** | CameraX `ImageAnalysis` outputs RGBA_8888 format frames → converted to OpenCV Mat objects (RGBA + Gray) → passed to JNI layer for SLAM processing. |

### 5.2 Autofocus

| Item            | Description                                                                                               |
| --------------- | --------------------------------------------------------------------------------------------------------- |
| **Declaration** | `<uses-feature android:name="android.hardware.camera.autofocus" android:required="false" />`              |
| **Purpose**     | User tap-to-focus triggers center autofocus and metering, improving image quality and tracking stability. |
| **Optional**    | Marked as `required="false"` — devices without autofocus support can still use the camera normally.       |

### 5.3 Sensors (Runtime Usage, Not Declared in Manifest)

| Sensor Type                                       | Purpose                                             | Priority                                |
| ------------------------------------------------- | --------------------------------------------------- | --------------------------------------- |
| **TYPE_ROTATION_VECTOR** (Rotation Vector Sensor) | 3DOF orientation tracking, computes device 3D pose  | Primary                                 |
| **TYPE_ACCELEROMETER** (Accelerometer)            | Fallback when rotation vector sensor is unavailable | Secondary (combined with magnetometer)  |
| **TYPE_MAGNETIC_FIELD** (Magnetometer)            | Fallback when rotation vector sensor is unavailable | Secondary (combined with accelerometer) |

**Notes**:

- Sensor data sample rate is set to `SensorManager.SENSOR_DELAY_GAME` (game level, ~20ms interval);
- All sensor data is processed **locally on the device** and **is not uploaded, stored, or transmitted to any external location**;
- Sensor coordinate systems are remapped for landscape orientation to accommodate the app's fixed landscape display.

---

## 6. Data Collection and Privacy Protection

### 6.1 Data We Do NOT Collect

This project strictly adheres to the principle of data minimization. **We do NOT collect, record, transmit, or share any of the following information**:

- ❌ No personally identifiable information (name, ID number, phone number, email address, etc.);
- ❌ No device identifiers (IMEI, IMSI, MAC address, Android ID, advertising ID, etc.);
- ❌ No location information (GPS coordinates, WiFi positioning, cellular base station positioning, etc.);
- ❌ No biometric information (fingerprints, face, iris, etc.);
- ❌ No contacts, SMS, call logs, or calendar data;
- ❌ No app usage statistics or analytics data;
- ❌ No crash reports or telemetry data;
- ❌ No third-party advertising SDKs or analytics SDKs;
- ❌ No active connections to any remote servers.

### 6.2 Local Data Processing and Inter-Process Communication (IPC)

| Data Type                                | Processing Method                                                                                                                                                                                                                            | Storage                                      |
| ---------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------- |
| **Camera frames**                        | Real-time processing: written to anonymous shared memory (Ashmem/SharedMemory double buffer), notifying the `:slam_process` process via oneway AIDL for processing. Frame data is overwritten/released by subsequent frames.                 | Not stored                                   |
| **Inter-Process Data Flow**              | Point cloud coordinates, MVP transformation matrices, and tracking states are exchanged between the main UI process and SLAM process via shared memory headers and memory mapping, entirely within local RAM with zero network transmission. | Not stored                                   |
| **SLAM maps (when user actively saves)** | Serialized as binary `.bin` files + `.json` metadata files                                                                                                                                                                                   | Saved to `getExternalFilesDir("SLAM/maps/")` |
| **Map metadata**                         | Includes: map name, keyframe count, map point count, creation timestamp, whether plane detection exists                                                                                                                                      | Same as above                                |
| **ORB vocabulary**                       | Pre-packaged file, used for feature matching (read-only)                                                                                                                                                                                     | `getExternalFilesDir("SLAM/")`               |
| **Camera calibration config**            | Pre-packaged file (read-only)                                                                                                                                                                                                                | `getExternalFilesDir("SLAM/")`               |
| **Log information**                      | Outputs debug logs via Android Logcat, visible only in developer mode                                                                                                                                                                        | System log buffer (circular overwrite)       |

### 6.3 Crash Handling

This application **does not include any custom crash collection mechanism**. When the application crashes, the Android system default behavior applies (displaying the "App has stopped" dialog).

- Benefiting from the **IPC process decoupling architecture**, the native SLAM algorithm runs in an independent `:slam_process`. If the underlying algorithm encounters an extreme out-of-memory or native crash, the host UI process can detect the disconnection via `ServiceConnection` and degrade gracefully, preventing a full application crash;
- The application itself **does NOT collect, store, or upload any crash information** to any remote server;
- The Android system may, with the user's explicit consent, collect basic crash stack information for system diagnostic purposes (this behavior is controlled by the Android system and is unrelated to this application).

### 6.4 Third-Party Library Data Handling

All third-party libraries used by this project run locally on the device and do not involve data transmission:

- **OpenCV**: Image processing (feature extraction, matrix operations), all executed locally on the CPU;
- **ORB-SLAM2 Core (C++)**: SLAM algorithm engine, encapsulated in `:slam_process` and executed entirely locally;
- **srrg_hbst (HBST)**: Hierarchical binary search tree for image matching and relocalization, all executed locally;
- **Google Filament + gltfio**: Physically-based 3D rendering engine for AR object display (GLB/glTF), rendering executed locally on GPU;
- **Google Guava / AndroidX**: System utility classes, not involved with user data;
- **Markwon**: Markdown rendering library used solely for displaying the Privacy Policy within the app, all executed locally.

---

## 7. Security Considerations

1. **App Signing**: APK/AAB distributions of this application should be signed with the developer's private key to ensure integrity and trustworthy source.
2. **Data Isolation**: All application data is stored in the app's `getExternalFilesDir()` sandbox directory, with access restricted by the Android system.
3. **Process Isolation & Security Boundary**: The SLAM core service runs in an isolated private process (`:slam_process`) declared with `android:exported="false"`. External unauthorized apps cannot bind to or invoke this service. The separation also delivers process-level fault isolation.
4. **Minimum Permissions**: Only the minimum permissions required for SLAM and AR functionality are requested; no extraneous permissions are declared.
5. **No Persistent Background Services**: The application has no persistent background services. All resources (camera, sensors, IPC bindings, GL context) are released upon exit.
6. **App Backup & Data Leakage Prevention**: The application's `AndroidManifest.xml` declares `android:allowBackup="true"` for convenience. On Android 12+ (API 31+), users can disable backup in device settings to prevent map files and app data from being included in system backups. Users handling sensitive mapping data should consider disabling app backup.
7. **Native Code Crash Handling**: The application's C++ native layer includes a signal handler framework that intercepts native crashes (e.g., SIGSEGV, SIGABRT) to produce diagnostic logs. These logs are written only to the Android Logcat buffer (accessible solely in developer/debug mode) and are never collected, stored, or transmitted by the application itself. This mechanism is provided solely for debugging purposes during development.

---

## 8. Respect and Inclusivity

The development, use, and community interaction of this project follow these principles:

### 8.1 Inclusive Community

- This project welcomes all contributors and users, **regardless of race, color, ethnicity, gender identity, sexual orientation, age, disability, religion, nationality, or any other characteristic protected by law**;
- We are committed to fostering a **friendly, safe, and inclusive** development environment and user community;
- When submitting Issues, Pull Requests, or participating in any form of project discussion, please use **respectful, professional, and constructive** language;

### 8.2 Prohibited Conduct

The following behaviors are **strictly prohibited** in all communication channels of this project (including but not limited to GitHub Issues, Pull Requests, discussion forums, mailing lists, etc.):

- ❌ Any form of discriminatory speech (including but not limited to racism, sexism, ageism, geographic discrimination, etc.);
- ❌ Personal attacks, insults, derogation, or harassment of others;
- ❌ Malicious provocation, stalking, intimidation, or unwanted attention;
- ❌ Posting pornographic, violent, or otherwise inappropriate content;
- ❌ Any form of bullying behavior;

### 8.3 Reporting and Resolution

If you encounter any behavior that violates the above principles in the project community, please contact the project maintainer (OlscStudio@outlook.com). All reports will be taken seriously and addressed promptly.

### 8.4 Respectful Use

- Users should respect others' privacy and image rights when using this software. **Do not use the AR and camera capabilities of this software for unauthorized recording or surveillance**;
- Do not use this software to create or distribute AR content containing discrimination, hate, violence, or illegal material;
- When using AR features in public or private spaces, respect the rules of that venue and the comfort of those around you.

---

## 9. Prohibited Uses

When downloading, using, or distributing this software (ORB-SLAM2s) or any derivative versions, you are **strictly prohibited** from using it for the following purposes:

### 9.1 Illegal and Malicious Purposes

- ❌ Any violation of the laws of the People's Republic of China;
- ❌ Any violation of the laws of the user's country or region;
- ❌ Infringement of others' privacy rights (e.g., covert recording, illegal surveillance);
- ❌ Any non-defensive military use, or use by internationally recognized terrorist organizations for manufacturing, operating, or guiding weapon systems, drone attack systems, or any lethal equipment;
- ❌ Unauthorized surveillance, tracking, or monitoring;
- ❌ Use of the SLAM/graphics components in un-certified control systems for autonomous vehicles;
- ❌ Use in un-certified medical diagnosis or surgical assistance;
- ❌ Use in un-certified aircraft, spacecraft navigation, or flight control systems;

### 9.2 High-Risk Activities

- ❌ Use in nuclear facilities, chemical plants, life-support systems, or other environments with extremely high safety requirements;
- ❌ Use in scenarios that could result in personal injury or property damage;
- ❌ Integration into critical infrastructure (aerospace, rail transportation, autonomous driving, etc.) without adequate safety testing and regulatory certification;

### 9.3 Intellectual Property and Compliance

- ❌ Removing or obscuring copyright notices and license information of this software and its upstream open-source components (ORB-SLAM2, DBoW2, g2o, etc.);
- ❌ Distributing modified versions or derivative works of this software in violation of GPL-3.0 license terms (must also be distributed under GPL-3.0 with source code provided);
- ❌ Using this software or its components for patent infringement or to assist in patent infringement.

---

## 10. Disclaimers

### 10.1 Software Disclaimer

Pursuant to Sections 15 and 16 of the GPL-3.0 License, and to the maximum extent permitted by applicable law:

**THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED**, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.

IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MODIFIES AND/OR CONVEYS THE SOFTWARE AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR INABILITY TO USE THE SOFTWARE (INCLUDING BUT NOT LIMITED TO LOSS OF DATA, DATA BEING RENDERED INACCURATE, LOSSES SUSTAINED BY YOU OR THIRD PARTIES, OR A FAILURE OF THE SOFTWARE TO OPERATE WITH ANY OTHER SOFTWARE), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

### 10.2 Functional Accuracy Disclaimer

- **SLAM Positioning Accuracy**: This software implements spatial positioning using monocular visual SLAM. Its accuracy is affected by various factors including ambient lighting, texture richness, camera calibration quality, and device motion speed. **Positioning results may contain errors, and no guarantee is made for centimeter-level or any specific positioning accuracy**. It should not be used in scenarios requiring high precision.
- **Plane Detection**: Plane detection is estimated based on sparse point cloud data; 100% accuracy is not guaranteed.
- **Dark Frame Detection**: The dark frame skip logic is designed to optimize performance and should not replace professional light detection equipment.

### 10.3 Force Majeure Disclaimer

The developers and copyright holders shall not be held liable for software unavailability, data loss, positioning errors, or other damages caused by the following force majeure events:

1. **Natural Disasters**: Earthquakes, fires, floods, typhoons, blizzards, and other natural disasters causing device damage or communication interruption;
2. **Public Events**: War, armed conflict, terrorist attacks, strikes, riots, etc.;
3. **Government Actions**: Changes in laws or regulations, government controls, sanctions, import/export restrictions, etc.;
4. **Public Health Events**: Major epidemics, quarantine measures, etc.;
5. **Technical Disasters**: Large-scale power outages, internet backbone failures, DNS service outages, and other technical infrastructure failures beyond the control of this software;
6. **Device Issues**: Hardware failures (e.g., camera sensor damage, IMU failure), operating system errors, storage media corruption, etc., unless directly caused by this software;
7. **Environmental Factors**: Extreme temperatures, strong magnetic field interference, severe vibration, insufficient or excessive lighting, and other environmental conditions affecting SLAM performance;
8. **Third-Party Service Interruptions**: Unavailability of code hosting platforms, dependency distribution platforms, OS update services, and other third-party services.

### 10.4 Legal Compliance Disclaimer

Users are solely responsible for ensuring their use of this software complies with the laws and regulations of their country or region. The developers assume no liability for users' violation of applicable laws.

---

## 11. Intellectual Property

This project is built upon the following open-source libraries and integrates the corresponding third-party components. The intellectual property ownership and license declarations for each library are as follows:

| Component                                    | Copyright Holder                                                                          | License                                                                      |
| -------------------------------------------- | ----------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------- |
| **ORB-SLAM2 Core Algorithm**                 | Raul Mur-Artal, Juan D. Tardos, J. M. M. Montiel, Dorian Galvez-Lopez                     | GPL-3.0                                                                      |
| **DBoW2 (modified)**                         | Dorian Galvez-Lopez                                                                       | Modified BSD (with notification clause)                                      |
| **g2o (modified)**                           | Rainer Kuemmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige, Wolfram Burgard        | BSD 2-Clause (core); some components GPL-3.0 / LGPL-3.0                      |
| **Eigen3**                                   | Benoît Jacob, Gaël Guennebaud and contributors                                            | MPL-2.0 (core; 3.4+ portions also under Apache-2.0 / BSD-3-Clause / GPL-3.0) |
| **OpenCV**                                   | Intel Corporation, Willow Garage, Itseez, NVIDIA, AMD, OpenCV Foundation and contributors | Apache 2.0 (some files BSD-3-Clause)                                         |
| **srrg_hbst (HBST)**                         | Dominik Schlegel, Giorgio Grisetti — srrg-software                                        | BSD 3-Clause                                                                 |
| **Google Filament**                          | Google LLC                                                                                | Apache 2.0                                                                   |
| **gltfio / filament-utils**                  | Google LLC                                                                                | Apache 2.0                                                                   |
| **ZXing ("Zebra Crossing")**                 | Sean Owen and ZXing project contributors                                                  | Apache 2.0                                                                   |
| **Google Guava**                             | Google LLC                                                                                | Apache 2.0                                                                   |
| **Markwon (io.noties.markwon)**              | Dimitry Ivanov (noties)                                                                   | Apache 2.0                                                                   |
| **AndroidX / CameraX**                       | Google LLC / Android Open Source Project                                                  | Apache 2.0                                                                   |
| **Android Support Library / Appcompat**      | Google LLC / Android Open Source Project                                                  | Apache 2.0                                                                   |
| **Material Components (Material Design)**    | Google LLC                                                                                | Apache 2.0                                                                   |
| **ORB-SLAM2s Adaptation & Enhancement Code** | Project contributors (see GitHub contributors list)                                       | GPL-3.0                                                                      |
| **Project Name "ORB-SLAM2s"**                | Project maintainer                                                                        | Does not constitute trademark registration                                   |

**Notes**:

- For the specific terms of each third-party library, please refer to the original license text in their official repositories.
- For closed-source commercial licensing inquiries of ORB-SLAM2, please contact the original authors: orbslam (at) unizar (dot) es.
- For project collaboration or other field cooperation inquiries, please contact: OlscStudio@outlook.com

---

## 12. Governing Law and Dispute Resolution

1. **Governing Law**: The interpretation, validity, and resolution of disputes under these terms shall be governed by the **laws of the People's Republic of China**, while also taking into account relevant international intellectual property treaties (including the Berne Convention, WIPO Copyright Treaty, etc.).
2. **Dispute Resolution**: Any disputes arising from or related to this software or these terms shall first be resolved through friendly negotiation; if negotiation fails, the dispute shall be submitted to the **People's Court with jurisdiction at the location of the project's primary maintainer**.
3. **International Users**: To the extent that PRC law conflicts with international law, PRC law shall govern, provided it does not violate mandatory legal provisions of the user's country.

---

## 13. Miscellaneous

1. **Entire Agreement**: This Privacy Policy and Terms of Use constitute the entire agreement between you and the developer regarding the use of this software.
2. **Severability**: If any provision of these terms is held to be invalid or unenforceable by a court of competent jurisdiction, the remaining provisions shall continue in full force and effect.
3. **Amendments**: This document may be revised as the project is updated. Continued use of the software after revisions constitutes acceptance of the updated terms.
4. **Language Versions**: This document is provided in both Chinese and English. In the event of any discrepancy between the two versions, the Chinese version shall prevail.
5. **Contact the Developer**: For any questions or concerns regarding this Privacy Policy and Terms of Use, please contact:
   - Email: OlscStudio@outlook.com
   - Project Repository: https://github.com/Olsc/Android_ORB-SLAM2s

---

**© 2026 ORB-SLAM2s Project Contributors. Released under the GPL-3.0 License.**
