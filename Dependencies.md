Third-Party Dependencies and Licenses
=====================================

This file lists third-party code and libraries bundled with the project but not
authored by the project authors, together with the licenses they are released
under. Refer to each upstream repository for the authoritative license text.

Engine (MenthaAR, GPLv3 module)
-------------------------------

- OpenCV                     Apache License 2.0
                             https://github.com/opencv/opencv
- Eigen3                     Mozilla Public License 2.0
                             https://eigen.tuxfamily.org
- DBoW2                      BSD 3-Clause License
                             https://github.com/dorian3d/DBoW2
- g2o                        BSD 3-Clause License (core modules)
                             https://github.com/RainerKuemmerle/g2o
- srrg_hbst                  BSD 3-Clause License
                             https://gitlab.com/srrg-software/srrg_hbst

App (Apache-2.0 module)
-----------------------

- cgltf                      MIT License
                             https://github.com/jkuhlmann/cgltf
- AndroidX / CameraX          Apache License 2.0
                             https://developer.android.com/jetpack

Notes
-----

1. Apache-2.0 and MIT components are compatible with GPLv3 and are incorporated
   into the GPLv3 engine module on that basis.
2. The `app/` module is Apache-2.0 and uses the `MenthaAR-ipc/` interface module
   (dual-licensed Apache-2.0 or GPLv3) at compile time only; the GPLv3 engine is
   packaged as a runtime-only artifact and reached through IPC.
3. Upstream ORB-SLAM2 is GPLv3; see MenthaAR/LICENSE and the project LICENSE for
   the license structure of this repository.
