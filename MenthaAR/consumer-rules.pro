# JNI 方法按名称绑定，必须保留 native 方法名与声明它们的类
-keepclasseswithmembernames,includedescriptorclasses class * {
    native <methods>;
}
-keep class com.orb.slam2s.slamar.NativeHelper { *; }

# SLAM 服务由 Manifest 按类名启动，且通过 Binder 跨进程通信
-keep class com.orb.slam2s.ipc.SlamService { *; }
-keep interface com.menthaar.sdk.internal.ISlamService { *; }
-keep class com.menthaar.sdk.internal.ISlamService$Stub { *; }