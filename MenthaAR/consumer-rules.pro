-keepclasseswithmembernames,includedescriptorclasses class * {
    native <methods>;
}
-keep class com.orb.slam2s.slamar.NativeHelper { *; }

-keep interface com.orb.slam2s.ipc.ISlamService { *; }
-keep class com.orb.slam2s.ipc.ISlamService { *; }
-keep class com.orb.slam2s.ipc.ISlamService$Stub { *; }
-keep class com.orb.slam2s.ipc.SlamService { *; }

-keep class com.orb.slam2s.constant.GlobalConstant { *; }
