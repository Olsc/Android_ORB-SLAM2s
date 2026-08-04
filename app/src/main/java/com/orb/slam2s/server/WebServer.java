package com.orb.slam2s.server;

import android.util.Log;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.security.KeyFactory;
import java.security.KeyStore;
import java.security.PrivateKey;
import java.security.cert.Certificate;
import java.security.cert.CertificateFactory;
import java.security.spec.PKCS8EncodedKeySpec;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import javax.net.ssl.KeyManagerFactory;
import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLServerSocket;
import javax.net.ssl.SSLServerSocketFactory;

public class WebServer {
    private static final String TAG = "WebServer";
    private final int port;
    private ServerSocket serverSocket;
    private volatile boolean isRunning;
    private com.orb.slam2s.slamar.NativeHelper nativeHelper;
    private final android.content.Context context;

    // 活跃的MJPEG流
    private final List<OutputStream> streamClients = Collections.synchronizedList(new ArrayList<>());

    // 从浏览器接收的图像帧回调接口
    public interface OnFrameReceivedListener {
        void onFrameReceived(byte[] frameData);
    }

    private OnFrameReceivedListener frameReceivedListener;

    public WebServer(int port, com.orb.slam2s.slamar.NativeHelper nativeHelper, android.content.Context context) {
        this.port = port;
        this.nativeHelper = nativeHelper;
        this.context = context;
    }

    public void setOnFrameReceivedListener(OnFrameReceivedListener listener) {
        this.frameReceivedListener = listener;
    }

    public void start() {
        if (isRunning)
            return;
        isRunning = true;
        new Thread(() -> {
            try {
                // 创建SSL服务器Socket
                SSLServerSocket sslServerSocket = createSSLServerSocket(port);
                serverSocket = sslServerSocket;
                
                Log.d(TAG, "服务器在 " + port + " 上启动");
                while (isRunning) {
                    try {
                        Socket client = serverSocket.accept();
                        new Thread(new ClientHandler(client)).start();
                    } catch (IOException e) {
                        if (isRunning)
                            Log.e(TAG, "接受连接错误", e);
                    }
                }
            } catch (Exception e) {
                Log.e(TAG, "服务器启动错误", e);
            }
        }).start();
    }

    /**
     * 创建SSL服务器Socket
     */
    private SSLServerSocket createSSLServerSocket(int port) throws Exception {
        // 1. 从assets读取证书和密钥
        InputStream certStream = context.getAssets().open("cert/cert.pem");
        InputStream keyStream = context.getAssets().open("cert/key.pem");

        // 2. 加载证书
        CertificateFactory cf = CertificateFactory.getInstance("X.509");
        Certificate cert = cf.generateCertificate(certStream);
        certStream.close();

        // 3. 加载私钥 (PEM格式)
        String keyPEM = readPEMFile(keyStream);
        keyStream.close();
        byte[] keyBytes = parsePEMPrivateKey(keyPEM);
        PKCS8EncodedKeySpec keySpec = new PKCS8EncodedKeySpec(keyBytes);
        KeyFactory kf = KeyFactory.getInstance("RSA");
        PrivateKey privateKey = kf.generatePrivate(keySpec);

        // 4. 创建KeyStore并加载证书和私钥
        KeyStore keyStore = KeyStore.getInstance(KeyStore.getDefaultType());
        keyStore.load(null, null);
        keyStore.setCertificateEntry("cert", cert);
        keyStore.setKeyEntry("key", privateKey, "".toCharArray(), new Certificate[]{cert});

        // 5. 初始化KeyManagerFactory
        KeyManagerFactory kmf = KeyManagerFactory.getInstance(KeyManagerFactory.getDefaultAlgorithm());
        kmf.init(keyStore, "".toCharArray());

        // 6. 创建SSLContext
        SSLContext sslContext = SSLContext.getInstance("TLS");
        sslContext.init(kmf.getKeyManagers(), null, null);

        // 7. 创建SSLServerSocketFactory
        SSLServerSocketFactory ssf = sslContext.getServerSocketFactory();
        
        // 8. 创建并返回SSLServerSocket
        SSLServerSocket sslServerSocket = (SSLServerSocket) ssf.createServerSocket(port);
        sslServerSocket.setNeedClientAuth(false);
        sslServerSocket.setWantClientAuth(false);
        return sslServerSocket;
    }

    /**
     * 读取PEM文件内容
     */
    private String readPEMFile(InputStream is) throws IOException {
        BufferedReader reader = new BufferedReader(new InputStreamReader(is));
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            sb.append(line).append("\n");
        }
        reader.close();
        return sb.toString();
    }

    /**
     * 解析PEM格式的私钥，提取Base64编码的密钥数据
     */
    private byte[] parsePEMPrivateKey(String pem) {
        // 移除PEM头尾和换行符
        String privateKeyPEM = pem
                .replace("-----BEGIN PRIVATE KEY-----", "")
                .replace("-----END PRIVATE KEY-----", "")
                .replace("-----BEGIN RSA PRIVATE KEY-----", "")
                .replace("-----END RSA PRIVATE KEY-----", "")
                .replaceAll("\\s", "");
        
        // Base64解码 (使用Android的Base64)
        return android.util.Base64.decode(privateKeyPEM, android.util.Base64.DEFAULT);
    }

    public void stop() {
        isRunning = false;
        try {
            if (serverSocket != null)
                serverSocket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        synchronized (streamClients) {
            for (OutputStream os : streamClients) {
                try {
                    os.close();
                } catch (IOException e) {
                    // 忽略
                }
            }
            streamClients.clear();
        }
    }

    private class ClientHandler implements Runnable {
        private final Socket socket;

        public ClientHandler(Socket socket) {
            this.socket = socket;
        }

        @Override
        public void run() {
            try (InputStream is = socket.getInputStream();
                    OutputStream os = socket.getOutputStream()) {

                // 使用手动readLine避免BufferedReader缓冲导致的数据丢失
                String line = readLine(is);
                if (line == null)
                    return;

                String[] parts = line.split(" ");
                if (parts.length < 2)
                    return;

                String method = parts[0];
                String path = parts[1];

                if (path.equals("/upload_frame") && method.equals("POST")) {
                    // upload_frame 会自己读取头部
                    handleUploadFrame(is, os);
                } else {
                    // 对于其他请求，先消耗掉剩余的头部，防止 TCP RST
                    consumeHeaders(is);

                    if (path.equals("/")) {
                        sendAssetFile(os);
                    } else if (path.equals("/stream")) {
                        handleStream(os);
                    } else if (path.equals("/data")) {
                        handleData(os);
                    } else {
                        send404(os);
                    }
                }

            } catch (javax.net.ssl.SSLException e) {
                // SSL 握手或协议错误（通常是客户端/浏览器未信任自签名证书而拒绝 TLS 握手）
                Log.w(TAG, "客户端 SSL 握手失败 (证书未信任或客户端断开): " + e.getMessage());
            } catch (java.net.SocketException e) {
                // 客户端连接中断或重置
                Log.w(TAG, "客户端 Socket 异常: " + e.getMessage());
            } catch (IOException e) {
                Log.e(TAG, "客户端处理器错误", e);
            }
        }

        // 消耗剩余的HTTP头部
        private void consumeHeaders(InputStream is) throws IOException {
            int b;
            int prev1 = 0, prev2 = 0, prev3 = 0;
            while ((b = is.read()) != -1) {
                if (prev3 == '\r' && prev2 == '\n' && prev1 == '\r' && b == '\n') {
                    break;
                }
                prev3 = prev2;
                prev2 = prev1;
                prev1 = b;
            }
        }

        // 手动读取一行，直到遇到换行符，避免缓冲过多数据
        private String readLine(InputStream is) throws IOException {
            ByteArrayOutputStream buffer = new ByteArrayOutputStream();
            int b;
            while ((b = is.read()) != -1) {
                if (b == '\n') {
                    break;
                }
                buffer.write(b);
            }
            if (b == -1 && buffer.size() == 0)
                return null;
            return buffer.toString("UTF-8").trim();
        }

        private void sendAssetFile(OutputStream os) throws IOException {
            try (InputStream assetIs = context.getAssets().open("index.html")) {
                ByteArrayOutputStream buffer = new ByteArrayOutputStream();
                int nRead;
                byte[] data = new byte[4096];
                while ((nRead = assetIs.read(data, 0, data.length)) != -1) {
                    buffer.write(data, 0, nRead);
                }
                buffer.flush();
                byte[] content = buffer.toByteArray();

                String response = "HTTP/1.1 200 OK\r\n" +
                        "Content-Type: text/html\r\n" +
                        "Content-Length: " + content.length + "\r\n\r\n";
                os.write(response.getBytes());
                os.write(content);
            } catch (IOException e) {
                send404(os);
            }
        }

        private void handleStream(OutputStream os) throws IOException {
            os.write(("HTTP/1.1 200 OK\r\n" +
                    "Content-Type: multipart/x-mixed-replace; boundary=boundary\r\n\r\n").getBytes());
            os.flush();
            synchronized (streamClients) {
                streamClients.add(os);
            }
            // 保持线程存活以维持流
            try {
                while (isRunning && socket.isConnected()) {
                    Thread.sleep(1000);
                }
            } catch (InterruptedException e) {
                // 忽略
            }
            synchronized (streamClients) {
                streamClients.remove(os);
            }
        }

        private void handleData(OutputStream os) throws IOException {
            if (nativeHelper == null) {
                send404(os);
                return;
            }

            // 1. 从本地获取数据
            // 当前跟踪点（蓝色）
            float[] trackedPoints = nativeHelper.getTrackedPoints(5000);
            if (trackedPoints == null)
                trackedPoints = new float[0];

            // 地图点（绿色）
            float[] mapPoints = nativeHelper.getMiniMapPoints(20000);
            if (mapPoints == null)
                mapPoints = new float[0];

            // AR对象
            float[] arObjectsRaw = nativeHelper.getAllArObjectsData();
            // 格式：[count, m0...m15, scale, m0...m15, scale...]

            int arObjCount = 0;
            List<Float> arObjList = new ArrayList<>();
            if (arObjectsRaw != null && arObjectsRaw.length > 0) {
                arObjCount = (int) arObjectsRaw[0];
                int idx = 1;
                for (int i = 0; i < arObjCount; i++) {
                    // 矩阵16个浮点数
                    for (int j = 0; j < 16; j++) {
                        if (idx < arObjectsRaw.length) {
                            arObjList.add(arObjectsRaw[idx++]);
                        } else {
                            arObjList.add(0f);
                        }
                    }
                    // 跳过缩放
                    idx++;
                }
            }

            // 计算总大小
            // 跟踪点：4 (计数) + len * 4
            // 地图点：4 (计数) + len * 4
            // AR：4 (计数) + (count * 16 * 4)
            int totalSize = 4 + (trackedPoints.length * 4) +
                    4 + (mapPoints.length * 4) +
                    4 + (arObjList.size() * 4) +
                    4 + (16 * 4); // 跟踪状态(4) + ViewMatrix(16*4)

            ByteBuffer buffer = ByteBuffer.allocate(totalSize);
            buffer.order(ByteOrder.LITTLE_ENDIAN);

            // 写入跟踪点
            buffer.putInt(trackedPoints.length / 3);
            FloatBuffer fb = buffer.asFloatBuffer();
            fb.put(trackedPoints);
            buffer.position(buffer.position() + trackedPoints.length * 4);

            // 写入地图点
            buffer.putInt(mapPoints.length / 3);
            fb = buffer.asFloatBuffer();
            fb.put(mapPoints);
            buffer.position(buffer.position() + mapPoints.length * 4);

            // 写入AR对象
            buffer.putInt(arObjCount);
            for (float f : arObjList) {
                buffer.putFloat(f);
            }

            // 4. 写入相机姿态 (View Matrix) 和 跟踪状态
            float[] viewMatrix = new float[16];
            nativeHelper.getV(viewMatrix);
            int trackingStatus = nativeHelper.getLastTrackingResult();

            buffer.putInt(trackingStatus);
            // 注意: buffer.putFloat会推进位置，混合使用putInt和FloatBuffer需小心 position
            // 这里直接用 putFloat 循环写入比较安全，或者重新切片
            for (float f : viewMatrix) {
                buffer.putFloat(f);
            }

            byte[] bytes = buffer.array();

            String header = "HTTP/1.1 200 OK\r\n" +
                    "Content-Type: application/octet-stream\r\n" +
                    "Content-Length: " + bytes.length + "\r\n" +
                    "Access-Control-Allow-Origin: *\r\n\r\n";
            os.write(header.getBytes());
            os.write(bytes);
        }


        private void handleUploadFrame(InputStream is, OutputStream os) throws IOException {
            try {
                // 手动读取HTTP头部（不使用BufferedReader避免消耗body数据）
                ByteArrayOutputStream headerBuffer = new ByteArrayOutputStream();
                int contentLength = -1;

                // 使用4字节缓冲区检测 \r\n\r\n
                byte[] last4 = new byte[4];
                int b;
                int count = 0;

                // 读取请求头，直到遇到\r\n\r\n
                while ((b = is.read()) != -1) {
                    headerBuffer.write(b);

                    // 更新最后4个字节的缓冲区
                    if (count < 4) {
                        last4[count] = (byte) b;
                        count++;
                    } else {
                        // 滑动窗口：左移一位
                        last4[0] = last4[1];
                        last4[1] = last4[2];
                        last4[2] = last4[3];
                        last4[3] = (byte) b;
                    }

                    // 检查是否为 \r\n\r\n (13, 10, 13, 10)
                    if (count >= 4 &&
                            last4[0] == 13 && last4[1] == 10 &&
                            last4[2] == 13 && last4[3] == 10) {
                        break;
                    }

                    // 防止头部过大
                    if (headerBuffer.size() > 8192) {
                        Log.e(TAG, "handleUploadFrame: 请求头过大");
                        send404(os);
                        return;
                    }
                }

                // 解析请求头，查找Content-Length
                String headers = headerBuffer.toString("UTF-8");

                for (String line : headers.split("\r\n")) {
                    if (line.toLowerCase().startsWith("content-length:")) {
                        contentLength = Integer.parseInt(line.substring("content-length:".length()).trim());
                        break;
                    }
                }

                if (contentLength <= 0) {
                    Log.e(TAG, "handleUploadFrame: Content-Length无效或未找到");
                    send404(os);
                    return;
                }

                // 读取图像数据（POST body）
                byte[] imageData = new byte[contentLength];
                int totalRead = 0;
                while (totalRead < contentLength) {
                    int read = is.read(imageData, totalRead, contentLength - totalRead);
                    if (read == -1)
                        break;
                    totalRead += read;
                }

                // 触发回调
                if (frameReceivedListener != null && totalRead == contentLength) {
                    frameReceivedListener.onFrameReceived(imageData);
                } else {
                    if (frameReceivedListener == null) {
                        Log.w(TAG, "handleUploadFrame: frameReceivedListener为null");
                    } else {
                        Log.w(TAG, "handleUploadFrame: 读取不完整 " + totalRead + " != " + contentLength);
                    }
                }

                // 返回成功响应
                String response = "HTTP/1.1 200 OK\r\n" +
                        "Access-Control-Allow-Origin: *\r\n" +
                        "Content-Type: text/plain\r\n" +
                        "Content-Length: 2\r\n\r\nOK";
                os.write(response.getBytes());
                os.flush();

            } catch (Exception e) {
                Log.e(TAG, "上传帧处理错误", e);
                send404(os);
            }
        }

        private void send404(OutputStream os) throws IOException {
            String response = "HTTP/1.1 404 Not Found\r\n\r\n";
            os.write(response.getBytes());
        }
    }
}