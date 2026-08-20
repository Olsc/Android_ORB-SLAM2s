/*
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.orb.slam2s.graphics;

import android.content.Context;
import android.opengl.GLES20;
import android.opengl.Matrix;

import com.orb.slam2s.R;

import java.nio.FloatBuffer;

// 相机 2D 背景正交直通渲染器（包含全屏网格面片几何体与正交着色器程序）
public class GLPassThroughRenderer {

    // 2D 矩形几何体网格面片
    private static class PlainMesh {
        private static final float[] TRIANGLE_VERTICES = {
                -1.0f, -1.0f, 0.0f,
                 1.0f, -1.0f, 0.0f,
                -1.0f,  1.0f, 0.0f,
                 1.0f,  1.0f, 0.0f
        };

        private static final float[] TEXTURE_NO_ROTATION = {
                0.0f, 1.0f,
                1.0f, 1.0f,
                0.0f, 0.0f,
                1.0f, 0.0f
        };

        private final FloatBuffer mVerticesBuffer;
        private final FloatBuffer mTexCoordinateBuffer;

        PlainMesh() {
            mVerticesBuffer = GLUtils.createDirectFloatBuffer(TRIANGLE_VERTICES, 0);
            mTexCoordinateBuffer = GLUtils.createDirectFloatBuffer(TEXTURE_NO_ROTATION, 0);
        }

        void uploadVertices(int positionHandle) {
            if (mVerticesBuffer == null) return;
            mVerticesBuffer.position(0);
            GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false, 0, mVerticesBuffer);
            GLUtils.checkGLError("glVertexAttribPointer position");
            GLES20.glEnableVertexAttribArray(positionHandle);
            GLUtils.checkGLError("glEnableVertexAttribArray position");
        }

        void uploadTextureCoords(int textureCoordinateHandle) {
            if (mTexCoordinateBuffer == null) return;
            mTexCoordinateBuffer.position(0);
            GLES20.glVertexAttribPointer(textureCoordinateHandle, 2, GLES20.GL_FLOAT, false, 0, mTexCoordinateBuffer);
            GLUtils.checkGLError("glVertexAttribPointer texCoord");
            GLES20.glEnableVertexAttribArray(textureCoordinateHandle);
            GLUtils.checkGLError("glEnableVertexAttribArray texCoord");
        }

        void draw() {
            GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);
        }
    }

    // 直通着色器程序
    private static class PassThroughProgram {
        private int mProgramId;
        private final String mVertexShaderSource;
        private final String mFragmentShaderSource;

        private int mPositionHandle;
        private int mTextureHandle;
        private int mMVPMatrixHandle;
        private int mTextureSamplerHandle;

        PassThroughProgram(Context context) {
            mVertexShaderSource = GLUtils.readRawTextFile(context, R.raw.vertex_shader_pass_through);
            mFragmentShaderSource = GLUtils.readRawTextFile(context, R.raw.fragment_shader_pass_through);
        }

        void create() {
            mProgramId = GLUtils.createProgram(mVertexShaderSource, mFragmentShaderSource);
            if (mProgramId == 0) return;

            mPositionHandle = GLES20.glGetAttribLocation(mProgramId, "aPosition");
            GLUtils.checkGLError("glGetAttribLocation aPosition");
            if (mPositionHandle == -1) {
                throw new RuntimeException("Could not get attrib location for aPosition");
            }

            mTextureHandle = GLES20.glGetAttribLocation(mProgramId, "aTextureCoord");
            GLUtils.checkGLError("glGetAttribLocation aTextureCoord");
            if (mTextureHandle == -1) {
                throw new RuntimeException("Could not get attrib location for aTextureCoord");
            }

            mTextureSamplerHandle = GLES20.glGetUniformLocation(mProgramId, "sTexture");
            GLUtils.checkGLError("glGetUniformLocation sTexture");

            mMVPMatrixHandle = GLES20.glGetUniformLocation(mProgramId, "uMVPMatrix");
            GLUtils.checkGLError("glGetUniformLocation uMVPMatrix");
        }

        void use() {
            GLES20.glUseProgram(mProgramId);
            GLUtils.checkGLError("glUseProgram");
        }

        void destroy() {
            if (mProgramId != 0) {
                GLES20.glDeleteProgram(mProgramId);
                mProgramId = 0;
            }
        }

        int getPositionHandle() { return mPositionHandle; }
        int getTextureHandle() { return mTextureHandle; }
        int getMVPMatrixHandle() { return mMVPMatrixHandle; }
        int getTextureSamplerHandle() { return mTextureSamplerHandle; }
    }

    private final PassThroughProgram mProgram;
    private final PlainMesh mMesh;
    private final float[] mProjectionMatrix = new float[16];
    private int mSurfaceWidth;
    private int mSurfaceHeight;

    public GLPassThroughRenderer(Context context) {
        mProgram = new PassThroughProgram(context);
        mMesh = new PlainMesh();
        Matrix.setIdentityM(mProjectionMatrix, 0);
    }

    public void init() {
        mProgram.create();
    }

    public void destroy() {
        mProgram.destroy();
    }

    public void onSurfaceChanged(int width, int height) {
        mSurfaceWidth = width;
        mSurfaceHeight = height;
    }

    public void onDrawFrame(int textureId) {
        mProgram.use();
        mMesh.uploadTextureCoords(mProgram.getTextureHandle());
        mMesh.uploadVertices(mProgram.getPositionHandle());
        GLES20.glUniformMatrix4fv(mProgram.getMVPMatrixHandle(), 1, false, mProjectionMatrix, 0);

        GLUtils.bindTexture2D(textureId, GLES20.GL_TEXTURE0, mProgram.getTextureSamplerHandle(), 0);
        GLES20.glViewport(0, 0, mSurfaceWidth, mSurfaceHeight);
        mMesh.draw();
    }
}
