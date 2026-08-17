/*
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s IPC Interface.
 *
 * Dual-licensed under the Apache License, Version 2.0 (the "Apache License")
 * or the GNU General Public License, version 3 (the "GPLv3").
 *
 * Under the Apache License, Version 2.0:
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *       http://www.apache.org/licenses/LICENSE-2.0
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * Under the GPLv3:
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   See <https://www.gnu.org/licenses/> for details.
  */
package com.orb.slam2s.ipc;

interface ISlamCallback {
    // 每帧的 MVP / 跟踪状态 / 点云已改为共享内存回传，不再走 binder 回调
    oneway void onPlaneDetected(int result);
    oneway void onSLAMInitialized(boolean success, String message);
}