package com.orb.slam2s.rendering.gles;

import android.content.Context;
import android.opengl.GLES20;
import android.opengl.Matrix;

import com.orb.slam2s.rendering.geometry.Plain;
import com.orb.slam2s.R;
import com.orb.slam2s.utils.TextureUtils;

public class OrthoFilter{

    private final GLPassThroughProgram glPassThroughProgram;
    private final Plain plain;

    private final float[] projectionMatrix = new float[16];
    private int surfaceWidth,surfaceHeight;
    public OrthoFilter(Context context) {
        glPassThroughProgram=new GLPassThroughProgram(context, R.raw.vertex_shader_pass_through,R.raw.fragment_shader_pass_through);
        plain=new Plain();
        Matrix.setIdentityM(projectionMatrix,0);
    }

    public void init() {
        glPassThroughProgram.create();
    }

    public void destroy() {
        glPassThroughProgram.onDestroy();
    }

    public void onSurfaceChanged(int width, int height){
        this.surfaceWidth=width;
        this.surfaceHeight=height;
    }

    public void onDrawFrame(int textureId) {
        glPassThroughProgram.use();
        plain.uploadTexCoordinateBuffer(glPassThroughProgram.getMaTextureHandle());
        plain.uploadVerticesBuffer(glPassThroughProgram.getMaPositionHandle());
        GLES20.glUniformMatrix4fv(glPassThroughProgram.getMVPMatrixHandle(), 1, false, projectionMatrix, 0);

        TextureUtils.bindTexture2D(textureId, GLES20.GL_TEXTURE0,glPassThroughProgram.getTextureSamplerHandle(),0);
        GLES20.glViewport(0,0,surfaceWidth,surfaceHeight);
        plain.draw();
    }

}
