# MenthaAR SDK 公开 API（其余为内部实现，可被压缩/混淆）
-keep class com.menthaar.sdk.MenthaAR { public *; }
-keep interface com.menthaar.sdk.MenthaAR$Listener { *; }
-keep class com.menthaar.sdk.MenthaAR$Frame { public *; }
-keep class com.menthaar.sdk.MenthaAR$Result { public *; }
-keep class com.menthaar.sdk.MenthaAR$MapStats { public *; }

# AIDL 协议：客户端与服务端通过 Binder 跨进程调用，类名与方法名不可混淆
-keep interface com.menthaar.sdk.internal.ISlamService { *; }