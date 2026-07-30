# Security Policy

We take the security and integrity of **ORB-SLAM2s** seriously. This document outlines our policy for reporting vulnerabilities, supported versions, and best practices for securing deployments.

## Supported Versions

As an active research and development project, security patches and stability fixes are generally backported only to the latest stable branch or the `main` branch.

| Version | Supported          | Description                                            |
| ------- | ------------------ | ------------------------------------------------------ |
| 0.0.x   | :white_check_mark: | Active development & optimization (latest main branch) |
| < 0.0.3 | :x:                | Older versions, please upgrade to the latest codebase  |

We recommend always building and running the latest commit from the `main` branch to ensure you have the latest stability fixes, memory leak resolutions, and security improvements.

## Scope of Security Concerns

Since **ORB-SLAM2s** is a C++/NDK native computer vision application running on Android/Ubuntu platforms, security researchers should pay close attention to the following areas:

1. **Map Deserialization**:
   - The map saving/loading feature serializes dynamic SLAM keyframes and point clouds into binary map files (`mentha_map.bin` & `*.arinfo`).
   * Loading untrusted or malformed binary map files may trigger **buffer overflows**, **out-of-bounds read/write**, or **arbitrary code execution (RCE)** in the native C++ serialization code.
2. **Buffer Overruns in OpenCV / Camera Feeds**:
   - The real-time image processing pipeline receives raw camera input frames. Ensure that standard NDK camera APIs and OpenCV resizing/conversions are compiled with proper stack protection.
3. **Memory Safety**
   - Underlying libraries such as `g2o`, `Eigen3`, and core C++ tracking/mapping components perform extensive custom memory allocations and matrix optimizations.
   - Improper handling may result in **memory corruption**, **arbitrary code execution (RCE)**, or **information disclosure**.

## Reporting Critical Security Issues

If you discover a security issue that could potentially allow **privilege escalation, remote control, or denial-of-service**, please **do not disclose it publicly**. Report it privately to our team:

- **Email**: [OlscStudio@outlook.com](mailto:OlscStudio@outlook.com)
- **Please include**:
  - Detailed description of the vulnerability and its potential impact
  - Steps to reproduce (proof-of-concept or malformed map file if applicable)
  - Compiler environment, device configuration, and affected versions

### Our Commitment

We will make every effort to respond to your report and coordinate a resolution:

1. **Initial Response**: Within **3 to 5 business days** of receiving your report, acknowledging receipt and beginning triage.
2. **Triage & Status Updates**: We will investigate the issue and keep you updated on our progress weekly.
3. **Remediation**: If accepted, we will work on a patch and coordinate a public release timeline. We will credit you for your responsible disclosure in the release notes or commit history (unless you prefer to remain anonymous).
