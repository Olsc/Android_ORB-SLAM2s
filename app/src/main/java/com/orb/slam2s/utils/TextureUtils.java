package com.orb.slam2s.utils;

import android.graphics.Bitmap;
import android.opengl.GLES20;
import android.opengl.GLUtils;

/**
 * Created by Ads on 2016/11/19.
 * 由Olsc于2025/8/25开始进行修改
 */
public class TextureUtils{

    public static int loadTexture(final Bitmap img, final int usedTexId) {
        int textures[] = new int[1];
        if (usedTexId == 0) {
            GLES20.glGenTextures(1, textures, 0);
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textures[0]);
            GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D,
                    GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
            GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D,
                    GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
            GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D,
                    GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
            GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D,
                    GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);

            GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, img, 0);
        } else {
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, usedTexId);

            GLUtils.texSubImage2D(GLES20.GL_TEXTURE_2D, 0, 0, 0, img);
            textures[0] = usedTexId;
        }
        return textures[0];
    }

    public static int getTextureFromBitmap(final Bitmap img, int[] outSize) {
        if (img == null || img.isRecycled()) {
            return 0;
        }
        if (outSize != null && outSize.length >= 2) {
            outSize[0] = img.getWidth();
            outSize[1] = img.getHeight();
        }
        return loadTexture(img, 0);
    }

    public static void bindTexture2D(int textureId,int activeTextureID,int handle,int idx){
        if (textureId !=0) {
            GLES20.glActiveTexture(activeTextureID);
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textureId);
            GLES20.glUniform1i(handle, idx);
        }
    }
}