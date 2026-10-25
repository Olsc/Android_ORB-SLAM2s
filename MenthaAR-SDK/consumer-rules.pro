// SDK 消费者混淆规则：保持 AIDL 接口与共享内存客户端不被裁剪
-keep interface com.orb.slam2s.ipc.ISlamService { *; }
-keep class com.orb.slam2s.ipc.SlamIPCClient { public *; }
-keep class com.orb.slam2s.ipc.SharedMemoryBuffer { *; }