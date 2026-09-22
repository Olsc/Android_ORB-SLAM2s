/*
 * MenthaAR PSVita - ORB-SLAM2 / MenthaAR engine front-end.
 *
 * This mirrors Thirdparty_Platforms/ubuntu/main.cpp as closely as the Vita
 * allows: it instantiates the very same ORB_SLAM2::System engine (monocular),
 * feeds it the camera as grayscale frames, projects the tracked / mapped
 * points with the calibrated intrinsics from Config.h, and exposes the same
 * map persistence (SaveMap / LoadMap).
 *
 * The engine itself is compiled UNCHANGED from ../../src + ../../include, using
 * the bundled Eigen / g2o / srrg_hbst and a calib3d compatibility layer
 * (compat/) on top of the community Vita OpenCV git submodule
 * (thirdparty/libopencv4).
 *
 * Controls:
 *   O ....... save map   (ux0:data/MenthaAR/mentha_map.bin)
 *   [] ...... load map
 *   /\ ...... reset / clear map
 *   L1 ...... toggle point-cloud overlay
 *   R1 ...... toggle "loaded points only"
 *   SELECT .. switch front / rear camera
 *   START ... exit
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <vita2d.h>
#include <psp2/ctrl.h>
#include <psp2/io/stat.h>
#include <psp2/kernel/processmgr.h>

#include <opencv2/core.hpp>

#include "include/System.h"
#include "include/Config.h"

#include "camera.h"
#include "renderer.h"
#include "config.h"        /* psvita-side constants (SCR_W, CAM_W, PACK_RGBA) */

#define MAP_PATH "ux0:data/MenthaAR/mentha_map.bin"
#define MAP_DIR  "ux0:data/MenthaAR"

static vita2d_texture *g_camTex = 0;

/* Display fit: 640x360 camera on a 960x544 screen (uniform scale + centring). */
static float g_dispScale = 1.5f;
static float g_offX = 0.0f;
static float g_offY = 0.0f;

static void copyCameraToTexture(const uint32_t *src)
{
	if (!g_camTex || !src)
		return;
	uint32_t *dst = (uint32_t *)vita2d_texture_get_datap(g_camTex);
	int stride = (int)(vita2d_texture_get_stride(g_camTex) / 4);
	if (stride == CAM_W) {
		memcpy(dst, src, (size_t)CAM_W * CAM_H * 4);
	} else {
		for (int y = 0; y < CAM_H; ++y)
			memcpy(dst + (size_t)y * stride, src + (size_t)y * CAM_W,
			       (size_t)CAM_W * 4);
	}
}

/* Camera ABGR (memory order R,G,B,A) -> CV_8UC1. */
static void abgrToGray(const uint32_t *src, cv::Mat &gray)
{
	uint8_t *out = gray.ptr<uint8_t>();
	const int n = CAM_W * CAM_H;
	for (int i = 0; i < n; ++i) {
		uint32_t px = src[i];
		int r = (int)(px & 0xFFu);
		int g = (int)((px >> 8) & 0xFFu);
		int b = (int)((px >> 16) & 0xFFu);
		out[i] = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
	}
}

/* Project a world point with Tcw and return display pixel coordinates. */
static bool projectPoint(const cv::Point3f &Pw, const cv::Mat &Tcw,
                         float fx, float fy, float cx, float cy,
                         float &outX, float &outY)
{
	const float R11 = Tcw.at<float>(0, 0), R12 = Tcw.at<float>(0, 1), R13 = Tcw.at<float>(0, 2);
	const float R21 = Tcw.at<float>(1, 0), R22 = Tcw.at<float>(1, 1), R23 = Tcw.at<float>(1, 2);
	const float R31 = Tcw.at<float>(2, 0), R32 = Tcw.at<float>(2, 1), R33 = Tcw.at<float>(2, 2);
	const float tx = Tcw.at<float>(0, 3), ty = Tcw.at<float>(1, 3), tz = Tcw.at<float>(2, 3);

	const float Xc = R11 * Pw.x + R12 * Pw.y + R13 * Pw.z + tx;
	const float Yc = R21 * Pw.x + R22 * Pw.y + R23 * Pw.z + ty;
	const float Zc = R31 * Pw.x + R32 * Pw.y + R33 * Pw.z + tz;
	if (Zc <= ORB_SLAM2::PROJECT_MIN_DEPTH)
		return false;

	const float invZ = 1.0f / Zc;
	outX = (fx * Xc * invZ + cx) * g_dispScale + g_offX;
	outY = (fy * Yc * invZ + cy) * g_dispScale + g_offY;
	return true;
}

int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	if (vita2d_init() < 0)
		return 0;
	vita2d_set_clear_color(PACK_RGBA(0, 0, 0, 255));
	vita2d_set_vblank_wait(1);

	Renderer renderer;
	renderer.init();

	g_camTex = vita2d_create_empty_texture_format(CAM_W, CAM_H,
	                                              SCE_GXM_TEXTURE_FORMAT_A8B8G8R8);
	if (g_camTex) {
		uint32_t *p = (uint32_t *)vita2d_texture_get_datap(g_camTex);
		memset(p, 0, (size_t)vita2d_texture_get_stride(g_camTex) * CAM_H);
	}

	{
		float sx = (float)SCR_W / (float)CAM_W;
		float sy = (float)SCR_H / (float)CAM_H;
		g_dispScale = (sx < sy) ? sx : sy;
		g_offX = (SCR_W - CAM_W * g_dispScale) * 0.5f;
		g_offY = (SCR_H - CAM_H * g_dispScale) * 0.5f;
	}

	sceCtrlSetSamplingMode(SCE_CTRL_MODE_ANALOG);
	sceIoMkdir(MAP_DIR, 0777);

	VitaCamera camera;
	if (!camera.open(1))
		camera.open(0);

	/* ---- SLAM engine (identical to the Ubuntu/Android front-ends) -------- */
	const float fx = ORB_SLAM2::CAMERA_FX;
	const float fy = ORB_SLAM2::CAMERA_FY;
	const float cx = ORB_SLAM2::CAMERA_CX;
	const float cy = ORB_SLAM2::CAMERA_CY;

	ORB_SLAM2::System *slamSys =
		new ORB_SLAM2::System("", ORB_SLAM2::System::MONOCULAR);

	/* Camera calibration can be refreshed from Config.h at any time. */
	slamSys->UpdateCalibration(fx, fy, cx, cy);

	cv::Mat gray(CAM_H, CAM_W, CV_8UC1);

	double timeStamp = 0.0;
	cv::Mat Tcw;
	std::vector<ORB_SLAM2::MapPoint *> vMPs;
	std::vector<cv::KeyPoint> vKeys;
	int status = ORB_SLAM2::Tracking::NO_IMAGES_YET;

	char statusMsg[160] = "MenthaAR SLAM starting - move the camera";
	float statusTimer = 5.0f;

	bool running = true;
	bool showCloud = true;
	bool loadedOnly = false;

	SceCtrlData pad, prev;
	memset(&pad, 0, sizeof(pad));
	memset(&prev, 0, sizeof(prev));
	bool havePrev = false;

	uint32_t lastTime = sceKernelGetProcessTimeLow();
	int fpsFrames = 0;
	float fpsAccum = 0.0f;
	float fps = 0.0f;

	while (running) {
		uint32_t now = sceKernelGetProcessTimeLow();
		float dt = (float)(now - lastTime) / 1000000.0f;
		lastTime = now;
		if (dt > 0.5f)
			dt = 0.5f;
		fpsFrames++;
		fpsAccum += dt;
		if (fpsFrames >= 30) {
			fps = (fpsAccum > 1e-4f) ? (fpsFrames / fpsAccum) : 0.0f;
			fpsFrames = 0;
			fpsAccum = 0.0f;
		}

		sceCtrlPeekBufferPositive(0, &pad, 1);
		unsigned int pressed = 0;
		if (havePrev)
			pressed = pad.buttons & ~prev.buttons;
		prev = pad;
		havePrev = true;

		if (pressed & SCE_CTRL_START)
			running = false;

		if (pressed & SCE_CTRL_SELECT) {
			int next = (camera.device() == 1) ? 0 : 1;
			if (camera.open(next))
				snprintf(statusMsg, sizeof(statusMsg), "Camera: %s",
				         next == 1 ? "rear" : "front");
			else
				snprintf(statusMsg, sizeof(statusMsg), "Camera switch failed");
			statusTimer = 3.0f;
		}

		if (pressed & SCE_CTRL_LTRIGGER) {
			showCloud = !showCloud;
			snprintf(statusMsg, sizeof(statusMsg), "Point cloud: %s",
			         showCloud ? "ON" : "OFF");
			statusTimer = 2.0f;
		}

		if (pressed & SCE_CTRL_RTRIGGER) {
			loadedOnly = !loadedOnly;
			snprintf(statusMsg, sizeof(statusMsg), "Show: %s",
			         loadedOnly ? "loaded map points" : "all map points");
			statusTimer = 2.0f;
		}

		if (pressed & SCE_CTRL_TRIANGLE) {
			if (slamSys)
				slamSys->Reset(false);
			snprintf(statusMsg, sizeof(statusMsg), "SLAM reset");
			statusTimer = 2.0f;
		}

		if (pressed & SCE_CTRL_CIRCLE) {
			if (slamSys) {
				slamSys->SaveMap(MAP_PATH);
				snprintf(statusMsg, sizeof(statusMsg), "Map saved: %s", MAP_PATH);
			}
			statusTimer = 4.0f;
		}

		if (pressed & SCE_CTRL_SQUARE) {
			if (slamSys) {
				slamSys->LoadMap(MAP_PATH, 0, false);
				snprintf(statusMsg, sizeof(statusMsg), "Map loaded: %s", MAP_PATH);
			}
			statusTimer = 4.0f;
		}

		/* ---- grab + track ------------------------------------------------- */
		bool frameOk = camera.isOpen() && camera.readFrame();
		if (frameOk) {
			copyCameraToTexture(camera.frame());
			if (slamSys) {
				abgrToGray(camera.frame(), gray);
				timeStamp += 1.0 / ORB_SLAM2::CAMERA_FPS;
				Tcw = slamSys->TrackMonocular(gray, timeStamp);
				status = slamSys->GetTrackingState();
				vMPs = slamSys->GetTrackedMapPoints();
				vKeys = slamSys->GetTrackedKeyPointsUn();
			}
		}

		if (statusTimer > 0.0f)
			statusTimer -= dt;

		/* ---- draw --------------------------------------------------------- */
		vita2d_wait_rendering_done();
		renderer.beginFrame(0x00000000u);

		if (showCloud && slamSys && !Tcw.empty() && Tcw.rows >= 3 && Tcw.cols >= 4) {
			std::vector<ORB_SLAM2::MapPoint *> allPts = slamSys->GetAllMapPoints();
			int drawn = 0;
			for (size_t i = 0; i < allPts.size() && drawn < ORB_SLAM2::UI_MAX_DRAWN_POINTS; ++i) {
				ORB_SLAM2::MapPoint *pMP = allPts[i];
				if (!pMP || pMP->isBad())
					continue;
				if (loadedOnly && !pMP->mbFromLoadedMap)
					continue;
				cv::Point3f Pw;
				pMP->GetWorldPos(Pw);
				float px, py;
				if (!projectPoint(Pw, Tcw, fx, fy, cx, cy, px, py))
					continue;
				if (px < 0 || px >= SCR_W || py < 0 || py >= SCR_H)
					continue;
				uint32_t col = pMP->mbFromLoadedMap
					? PACK_RGBA(ORB_SLAM2::UI_COLOR_LOADED_POINT_R,
					            ORB_SLAM2::UI_COLOR_LOADED_POINT_G,
					            ORB_SLAM2::UI_COLOR_LOADED_POINT_B, 255)
					: PACK_RGBA(ORB_SLAM2::UI_COLOR_NEW_POINT_R,
					            ORB_SLAM2::UI_COLOR_NEW_POINT_G,
					            ORB_SLAM2::UI_COLOR_NEW_POINT_B, 255);
				renderer.drawMarker((int)px, (int)py, col,
				                    ORB_SLAM2::UI_CLOUD_POINT_RADIUS);
				drawn++;
			}
		}

		/* Tracked keypoints (cyan = new, green = loaded). */
		if (showCloud) {
			const size_t kn = vKeys.size();
			for (size_t i = 0; i < kn; ++i) {
				const cv::KeyPoint &kp = vKeys[i];
				float px = kp.pt.x * g_dispScale + g_offX;
				float py = kp.pt.y * g_dispScale + g_offY;
				bool matched = (i < vMPs.size()) && vMPs[i];
				int r = ORB_SLAM2::UI_POINT_RADIUS;
				uint32_t col;
				if (matched && vMPs[i]->mbFromLoadedMap)
					col = PACK_RGBA(0, 255, 0, 255);
				else if (matched)
					col = PACK_RGBA(ORB_SLAM2::UI_COLOR_NEW_POINT_R,
					                ORB_SLAM2::UI_COLOR_NEW_POINT_G,
					                ORB_SLAM2::UI_COLOR_NEW_POINT_B, 255);
				else
					col = PACK_RGBA(90, 90, 90, 255);
				renderer.drawMarker((int)px, (int)py, col, r);
			}
		}

		/* HUD */
		const char *stateStr = "NO IMAGES";
		uint32_t stateCol = PACK_RGBA(180, 180, 180, 255);
		if (status == ORB_SLAM2::Tracking::NOT_INITIALIZED) {
			stateStr = "NOT INITIALIZED";
			stateCol = PACK_RGBA(255, 200, 40, 255);
		} else if (status == ORB_SLAM2::Tracking::OK) {
			stateStr = "TRACKING OK";
			stateCol = PACK_RGBA(40, 240, 80, 255);
		} else if (status == ORB_SLAM2::Tracking::LOST) {
			stateStr = "TRACKING LOST";
			stateCol = PACK_RGBA(255, 60, 60, 255);
		}

		renderer.drawPanel(10, 10, 560, 108, 0.55f);
		renderer.drawText(20, 16, "MenthaAR PSVita  |  ORB-SLAM2 monocular",
		                  PACK_RGBA(120, 255, 200, 255), 1);
		renderer.drawTextf(20, 34, PACK_RGBA(255, 255, 255, 255), 1,
		                   "SLAM: %-16s FPS: %4.0f  Camera: %s",
		                   stateStr, fps,
		                   camera.isOpen() ? (camera.device() == 1 ? "rear" : "front")
		                                   : "off");
		int nKF = slamSys ? slamSys->GetNumKeyFrames() : 0;
		int nMP = slamSys ? slamSys->GetNumMapPoints() : 0;
		renderer.drawTextf(20, 52, PACK_RGBA(0, 255, 136, 255), 1,
		                   "KFs: %-5d  MapPoints: %-6d  Tracked: %u",
		                   nKF, nMP, (unsigned)vMPs.size());
		renderer.drawText(20, 70,
		                  "X Scan/move  O Save  [] Load  /\\ Reset   L1 Cloud  R1 Loaded",
		                  PACK_RGBA(200, 220, 255, 255), 1);
		renderer.drawTextf(20, 88, PACK_RGBA(160, 180, 200, 255), 1,
		                   "Config fx=%.0f fy=%.0f cx=%.0f cy=%.0f   START exits",
		                   fx, fy, cx, cy);

		if (statusTimer > 0.0f && statusMsg[0]) {
			renderer.drawPanel(10, SCR_H - 40, 820, 30, 0.6f);
			renderer.drawText(20, SCR_H - 32, statusMsg,
			                  PACK_RGBA(255, 255, 255, 255), 1);
		}

		vita2d_start_drawing();
		vita2d_clear_screen();
		if (g_camTex && camera.isOpen())
			vita2d_draw_texture_scale(g_camTex, g_offX, g_offY,
			                          g_dispScale, g_dispScale);
		if (renderer.texture())
			vita2d_draw_texture(renderer.texture(), 0.0f, 0.0f);
		vita2d_end_drawing();
		vita2d_swap_buffers();
	}

	/* ---- shutdown -------------------------------------------------------- */
	if (slamSys) {
		slamSys->Shutdown();
		delete slamSys;
		slamSys = 0;
	}

	camera.close();
	if (g_camTex) {
		vita2d_free_texture(g_camTex);
		g_camTex = 0;
	}
	renderer.shutdown();
	vita2d_fini();

	sceKernelExitProcess(0);
	return 0;
}
