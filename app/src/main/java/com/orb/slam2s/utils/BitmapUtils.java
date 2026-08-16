package com.orb.slam2s.utils;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import java.io.IOException;
import java.io.InputStream;

/**
 * Created by Ads on 2016/11/8.
 * 由Olsc于2025/8/25开始进行修改
 */
public class BitmapUtils {

    public static Bitmap loadBitmapFromAssets(Context context,String filePath){
        // J-14：finally 中关闭流——原先 InputStream 泄漏
        InputStream inputStream = null;
        try {
            inputStream = context.getResources().getAssets().open(filePath);
            BitmapFactory.Options options=new BitmapFactory.Options();
            options.inScaled=false;
            return BitmapFactory.decodeStream(inputStream);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        } finally {
            if (inputStream != null) {
                try { inputStream.close(); } catch (IOException ignored) {}
            }
        }
    }

    public static Bitmap loadBitmapFromRaw(Context context, int resourceId){
        BitmapFactory.Options options=new BitmapFactory.Options();
        options.inScaled=false;
        Bitmap bitmap= BitmapFactory.decodeResource(context.getResources(),resourceId,options);
        return bitmap;
    }
}