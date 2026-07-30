# Profiler Tools

This directory contains tools for analyzing the performance of the ORB-SLAM2s system on Android.

## Mentha Profiler Viewer (`mentha_viewer.py`)

The Mentha Profiler is a high-performance, real-time profiling system integrated into the C++ core. It records function execution times and call sequences with minimal overhead.

### Usage

1.  **Generate Log**: Run the Android app in Develop Mode. A file named `mentha_profile.bin` will be created in the app's internal storage (typically `/data/data/com.orb.slam2s.slamar/files/`).
2.  **Pull Log**: Use ADB to pull the log to your computer:
    ```bash
    adb pull /data/data/com.orb.slam2s.slamar/files/mentha_profile.bin .
    ```
3.  **Convert**: Run the viewer script to convert the binary log to JSON:
    ```bash
    python mentha_viewer.py mentha_profile.bin
    ```
4.  **Visualize**: 
    *   Open Google Chrome.
    *   Navigate to `chrome://tracing`.
    *   Drag and drop the generated `profile_trace.json` into the window.

### Features
*   **Nanosecond Precision**: Uses high-resolution hardware timers.
*   **Asynchronous Writing**: Minimal impact on SLAM tracking performance.
*   **Crash Resistance**: Data is flushed regularly to ensure logs are preserved even if the app crashes.
