-keep class cn.pedant.SweetAlert.** { *;}

-keepclasseswithmembernames,includedescriptorclasses class * {
    native <methods>;
}

-keepclassmembers class com.orb.slam2s.graphics.GlbModelRenderer {
    public int onLoadTextureFromBytes(byte[]);
}
-keep class com.orb.slam2s.graphics.GlbModelRenderer { *; }

-keep class com.orb.slam2s.ipc.SlamService { *; }
-keep interface com.menthaar.sdk.internal.ISlamService { *; }
-keep class com.menthaar.sdk.internal.ISlamService$Stub { *; }

-keep class com.orb.slam2s.app.SplashActivity { *; }
-keep class com.orb.slam2s.ui.MainActivity { *; }
-keep class com.orb.slam2s.ui.MapManageActivity { *; }
-keep class com.orb.slam2s.ui.IconSelectActivity { *; }
-keep class com.orb.slam2s.util.MapManager** { *; }

-keepclasseswithmembers class * {
    public <init>(android.content.Context, android.util.AttributeSet);
}
-keepclasseswithmembers class * {
    public <init>(android.content.Context, android.util.AttributeSet, int);
}

-keepattributes Signature,*Annotation*,InnerClasses,EnclosingMethod

-keepattributes SourceFile,LineNumberTable
-renamesourcefileattribute SourceFile