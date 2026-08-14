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
        InputStream inputStream = null;
        try {
            inputStream = context.getResources().getAssets().open(filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }
        if(inputStream==null) return null;
        BitmapFactory.Options options=new BitmapFactory.Options();
        options.inScaled=false;
        Bitmap bitmap= BitmapFactory.decodeStream(inputStream);
        return bitmap;
    }

    public static Bitmap loadBitmapFromRaw(Context context, int resourceId){
        BitmapFactory.Options options=new BitmapFactory.Options();
        options.inScaled=false;
        Bitmap bitmap= BitmapFactory.decodeResource(context.getResources(),resourceId,options);
        return bitmap;
    }
}