
-keep class cn.pedant.SweetAlert.** { *;}

-keepclasseswithmembernames,includedescriptorclasses class * {
    native <methods>;
}

-keepclassmembers class com.orb.slam2s.graphics.GlbModelRenderer {
    public int onLoadTextureFromBytes(byte[]);
}
-keep class com.orb.slam2s.graphics.GlbModelRenderer { *; }

-keep class com.orb.slam2s.slamar.NativeHelper { *; }

-keep interface com.orb.slam2s.ipc.ISlamService { *; }
-keep class com.orb.slam2s.ipc.ISlamService { *; }
-keep class com.orb.slam2s.ipc.ISlamService$Stub { *; }

-keep class com.orb.slam2s.app.SplashActivity { *; }
-keep class com.orb.slam2s.ui.MainActivity { *; }
-keep class com.orb.slam2s.ui.MapManageActivity { *; }
-keep class com.orb.slam2s.ui.IconSelectActivity { *; }
-keep class com.orb.slam2s.ipc.SlamService { *; }

-keepclasseswithmembers class * {
    public <init>(android.content.Context, android.util.AttributeSet);
}
-keepclasseswithmembers class * {
    public <init>(android.content.Context, android.util.AttributeSet, int);
}

-keepattributes Signature,*Annotation*,InnerClasses,EnclosingMethod

-keepattributes SourceFile,LineNumberTable
-renamesourcefileattribute SourceFile