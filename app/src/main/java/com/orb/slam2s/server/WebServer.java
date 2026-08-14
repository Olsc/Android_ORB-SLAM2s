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
import java.security.KeyStore;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import javax.net.ssl.KeyManagerFactory;
import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLServerSocket;
import javax.net.ssl.SSLServerSocketFactory;

import com.orb.slam2s.ipc.SlamIPCClient;

public class WebServer {
    private static final String TAG = "WebServer";
    private final int port;
    private ServerSocket serverSocket;
    private volatile boolean isRunning;
    private com.orb.slam2s.slamar.NativeHelper nativeHelper;
    private SlamIPCClient slamIPCClient;
    private final android.content.Context context;

    private final List<OutputStream> streamClients = Collections.synchronizedList(new ArrayList<>());

    public interface OnFrameReceivedListener {
        void onFrameReceived(byte[] frameData);
    }

    private OnFrameReceivedListener frameReceivedListener;

    public WebServer(int port, com.orb.slam2s.slamar.NativeHelper nativeHelper, android.content.Context context) {
        this.port = port;
        this.nativeHelper = nativeHelper;
        this.context = context;
    }

    public WebServer(int port, com.orb.slam2s.slamar.NativeHelper nativeHelper, SlamIPCClient slamIPCClient, android.content.Context context) {
        this.port = port;
        this.nativeHelper = nativeHelper;
        this.slamIPCClient = slamIPCClient;
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

    private SSLServerSocket createSSLServerSocket(int port) throws Exception {
        KeyStore keyStore = KeyStore.getInstance("AndroidKeyStore");
        keyStore.load(null);
        String alias = "webserver_dynamic_key";
        if (keyStore.containsAlias(alias)) {
            keyStore.deleteEntry(alias);
        }

        java.security.KeyPairGenerator kpg = java.security.KeyPairGenerator.getInstance(
                android.security.keystore.KeyProperties.KEY_ALGORITHM_RSA, "AndroidKeyStore");
        
        kpg.initialize(new android.security.keystore.KeyGenParameterSpec.Builder(
                alias,
                android.security.keystore.KeyProperties.PURPOSE_SIGN | android.security.keystore.KeyProperties.PURPOSE_VERIFY | android.security.keystore.KeyProperties.PURPOSE_ENCRYPT | android.security.keystore.KeyProperties.PURPOSE_DECRYPT)
                .setDigests(android.security.keystore.KeyProperties.DIGEST_NONE, android.security.keystore.KeyProperties.DIGEST_SHA256, android.security.keystore.KeyProperties.DIGEST_SHA512)
                .setEncryptionPaddings(android.security.keystore.KeyProperties.ENCRYPTION_PADDING_NONE, android.security.keystore.KeyProperties.ENCRYPTION_PADDING_RSA_PKCS1)
                .setSignaturePaddings(android.security.keystore.KeyProperties.SIGNATURE_PADDING_RSA_PKCS1)
                .setRandomizedEncryptionRequired(false)
                .setCertificateSubject(new javax.security.auth.x500.X500Principal("CN=localhost"))
                .setCertificateSerialNumber(java.math.BigInteger.valueOf(System.currentTimeMillis()))
                .setCertificateNotBefore(new java.util.Date())
                .setCertificateNotAfter(new java.util.Date(System.currentTimeMillis() + 365L * 24 * 60 * 60 * 1000))
                .build());

        kpg.generateKeyPair();

        KeyManagerFactory kmf = KeyManagerFactory.getInstance(KeyManagerFactory.getDefaultAlgorithm());
        kmf.init(keyStore, null);

        SSLContext sslContext = SSLContext.getInstance("TLS");
        sslContext.init(kmf.getKeyManagers(), null, null);

        SSLServerSocketFactory ssf = sslContext.getServerSocketFactory();

        SSLServerSocket sslServerSocket = (SSLServerSocket) ssf.createServerSocket(port);
        sslServerSocket.setNeedClientAuth(false);
        sslServerSocket.setWantClientAuth(false);
        return sslServerSocket;
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
                } catch (IOException ignored) {
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

                String line = readLine(is);
                if (line == null)
                    return;

                String[] parts = line.split(" ");
                if (parts.length < 2)
                    return;

                String method = parts[0];
                String path = parts[1];

                if (path.equals("/upload_frame") && method.equals("POST")) {
                    handleUploadFrame(is, os);
                } else {
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
                Log.w(TAG, "客户端 SSL 握手失败: " + e.getMessage());
            } catch (java.net.SocketException e) {
                Log.w(TAG, "客户端 Socket 异常: " + e.getMessage());
            } catch (IOException e) {
                Log.e(TAG, "客户端处理器错误", e);
            }
        }

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
            try {
                while (isRunning && socket.isConnected()) {
                    Thread.sleep(1000);
                }
            } catch (InterruptedException ignored) {
            }
            synchronized (streamClients) {
                streamClients.remove(os);
            }
        }

        private void handleData(OutputStream os) throws IOException {
            float[] trackedPoints = null;
            float[] mapPoints = null;
            float[] arObjectsRaw = null;
            int trackingStatus = 0;

            if (slamIPCClient != null && slamIPCClient.isConnected()) {
                trackedPoints = slamIPCClient.getTrackedPoints(5000);
                mapPoints = slamIPCClient.getMiniMapPoints(20000);
                arObjectsRaw = slamIPCClient.getAllArObjectsData();
                trackingStatus = slamIPCClient.getLastTrackingResult();
            } else if (nativeHelper != null) {
                trackedPoints = nativeHelper.getTrackedPoints(5000);
                mapPoints = nativeHelper.getMiniMapPoints(20000);
                arObjectsRaw = nativeHelper.getAllArObjectsData();
                trackingStatus = nativeHelper.getLastTrackingResult();
            } else {
                send404(os);
                return;
            }

            if (trackedPoints == null) trackedPoints = new float[0];
            if (mapPoints == null) mapPoints = new float[0];

            int arObjCount = 0;
            List<Float> arObjList = new ArrayList<>();
            if (arObjectsRaw != null && arObjectsRaw.length > 0) {
                arObjCount = (int) arObjectsRaw[0];
                int idx = 1;
                for (int i = 0; i < arObjCount; i++) {
                    for (int j = 0; j < 16; j++) {
                        if (idx < arObjectsRaw.length) {
                            arObjList.add(arObjectsRaw[idx++]);
                        } else {
                            arObjList.add(0f);
                        }
                    }
                    idx++;
                }
            }

            int totalSize = 4 + (trackedPoints.length * 4) +
                    4 + (mapPoints.length * 4) +
                    4 + (arObjList.size() * 4) +
                    4 + (16 * 4);

            ByteBuffer buffer = ByteBuffer.allocate(totalSize);
            buffer.order(ByteOrder.LITTLE_ENDIAN);

            buffer.putInt(trackedPoints.length / 3);
            FloatBuffer fb = buffer.asFloatBuffer();
            fb.put(trackedPoints);
            buffer.position(buffer.position() + trackedPoints.length * 4);

            buffer.putInt(mapPoints.length / 3);
            fb = buffer.asFloatBuffer();
            fb.put(mapPoints);
            buffer.position(buffer.position() + mapPoints.length * 4);

            buffer.putInt(arObjCount);
            for (float f : arObjList) {
                buffer.putFloat(f);
            }

            float[] viewMatrix = new float[16];
            if (slamIPCClient != null && slamIPCClient.isConnected()) {
                slamIPCClient.getV(viewMatrix);
            } else if (nativeHelper != null) {
                nativeHelper.getV(viewMatrix);
            }

            buffer.putInt(trackingStatus);
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
                ByteArrayOutputStream headerBuffer = new ByteArrayOutputStream();
                int contentLength = -1;

                byte[] last4 = new byte[4];
                int b;
                int count = 0;

                while ((b = is.read()) != -1) {
                    headerBuffer.write(b);

                    if (count < 4) {
                        last4[count] = (byte) b;
                        count++;
                    } else {
                        last4[0] = last4[1];
                        last4[1] = last4[2];
                        last4[2] = last4[3];
                        last4[3] = (byte) b;
                    }

                    if (count >= 4 &&
                            last4[0] == 13 && last4[1] == 10 &&
                            last4[2] == 13 && last4[3] == 10) {
                        break;
                    }

                    if (headerBuffer.size() > 8192) {
                        Log.e(TAG, "handleUploadFrame: 请求头过大");
                        send404(os);
                        return;
                    }
                }

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

                byte[] imageData = new byte[contentLength];
                int totalRead = 0;
                while (totalRead < contentLength) {
                    int read = is.read(imageData, totalRead, contentLength - totalRead);
                    if (read == -1)
                        break;
                    totalRead += read;
                }

                if (frameReceivedListener != null && totalRead == contentLength) {
                    frameReceivedListener.onFrameReceived(imageData);
                }

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