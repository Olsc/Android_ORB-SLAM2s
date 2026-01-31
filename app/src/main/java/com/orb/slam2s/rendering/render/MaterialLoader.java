package com.orb.slam2s.rendering.render;

import android.content.Context;
import android.util.Log;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Map;

/**
 * MaterialLoader - 用于解析和管理MTL材质文件
 * 支持现代OBJ格式中的材质定义
 */
public class MaterialLoader {
    private static final String TAG = "MaterialLoader";
    
    // 材质属性类
    public static class Material {
        public String name;                    // 材质名称
        public float[] ambientColor = {0.2f, 0.2f, 0.2f};     // 环境光颜色 Ka
        public float[] diffuseColor = {0.8f, 0.8f, 0.8f};     // 漫射光颜色 Kd
        public float[] specularColor = {1.0f, 1.0f, 1.0f};    // 镜面光颜色 Ks
        public float shininess = 32.0f;        // 光泽度 Ns
        public float transparency = 1.0f;      // 透明度 d/Tr
        
        // 纹理贴图路径
        public String diffuseTexture = null;   // map_Kd 漫射贴图
        public String specularTexture = null;  // map_Ks 镜面贴图
        public String normalTexture = null;    // map_bump 法线贴图
        public String ambientTexture = null;   // map_Ka 环境贴图
        
        public Material(String name) {
            this.name = name;
        }
        
        @Override
        public String toString() {
            return String.format("Material{name='%s', diffuse=[%.2f,%.2f,%.2f], shininess=%.1f, diffuseTexture='%s'}", 
                    name, diffuseColor[0], diffuseColor[1], diffuseColor[2], shininess, diffuseTexture);
        }
    }
    
    private Map<String, Material> materials = new HashMap<>();
    private String currentMaterialName = null;
    
    /**
     * 从assets中加载MTL文件
     * @param context Android上下文
     * @param mtlFileName MTL文件名
     * @return 是否成功加载
     */
    public boolean loadMTL(Context context, String mtlFileName) {
        try {
            InputStream inputStream = context.getAssets().open(mtlFileName);
            BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
            
            String line;
            Material currentMaterial = null;
            
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                
                // 跳过空行和注释
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }
                
                String[] parts = line.split("\\s+");
                if (parts.length == 0) continue;
                
                String command = parts[0].toLowerCase();
                
                switch (command) {
                    case "newmtl":
                        // 新材质定义
                        if (parts.length >= 2) {
                            currentMaterialName = parts[1];
                            currentMaterial = new Material(currentMaterialName);
                            materials.put(currentMaterialName, currentMaterial);
                            Log.d(TAG, "创建新材质: " + currentMaterialName);
                        }
                        break;
                        
                    case "ka":
                        // 环境光颜色
                        if (currentMaterial != null && parts.length >= 4) {
                            currentMaterial.ambientColor[0] = Float.parseFloat(parts[1]);
                            currentMaterial.ambientColor[1] = Float.parseFloat(parts[2]);
                            currentMaterial.ambientColor[2] = Float.parseFloat(parts[3]);
                        }
                        break;
                        
                    case "kd":
                        // 漫射光颜色
                        if (currentMaterial != null && parts.length >= 4) {
                            currentMaterial.diffuseColor[0] = Float.parseFloat(parts[1]);
                            currentMaterial.diffuseColor[1] = Float.parseFloat(parts[2]);
                            currentMaterial.diffuseColor[2] = Float.parseFloat(parts[3]);
                        }
                        break;
                        
                    case "ks":
                        // 镜面光颜色
                        if (currentMaterial != null && parts.length >= 4) {
                            currentMaterial.specularColor[0] = Float.parseFloat(parts[1]);
                            currentMaterial.specularColor[1] = Float.parseFloat(parts[2]);
                            currentMaterial.specularColor[2] = Float.parseFloat(parts[3]);
                        }
                        break;
                        
                    case "ns":
                        // 光泽度
                        if (currentMaterial != null && parts.length >= 2) {
                            currentMaterial.shininess = Float.parseFloat(parts[1]);
                        }
                        break;
                        
                    case "d":
                    case "tr":
                        // 透明度
                        if (currentMaterial != null && parts.length >= 2) {
                            float transparency = Float.parseFloat(parts[1]);
                            // Tr和d的定义相反：Tr是不透明度，d是透明度
                            currentMaterial.transparency = command.equals("tr") ? (1.0f - transparency) : transparency;
                        }
                        break;
                        
                    case "map_kd":
                        // 漫射贴图
                        if (currentMaterial != null && parts.length >= 2) {
                            currentMaterial.diffuseTexture = parts[parts.length - 1]; // 取最后一个参数作为文件名
                        }
                        break;
                        
                    case "map_ks":
                        // 镜面贴图
                        if (currentMaterial != null && parts.length >= 2) {
                            currentMaterial.specularTexture = parts[parts.length - 1];
                        }
                        break;
                        
                    case "map_ka":
                        // 环境贴图
                        if (currentMaterial != null && parts.length >= 2) {
                            currentMaterial.ambientTexture = parts[parts.length - 1];
                        }
                        break;
                        
                    case "map_bump":
                    case "bump":
                        // 法线/凹凸贴图
                        if (currentMaterial != null && parts.length >= 2) {
                            currentMaterial.normalTexture = parts[parts.length - 1];
                        }
                        break;
                        
                    default:
                        // 忽略未知命令
                        Log.v(TAG, "忽略未知MTL命令: " + command);
                        break;
                }
            }
            
            reader.close();
            
            Log.i(TAG, String.format("成功加载MTL文件 '%s'，包含 %d 个材质", mtlFileName, materials.size()));
            for (Material material : materials.values()) {
                Log.d(TAG, "  " + material.toString());
            }
            
            return true;
            
        } catch (IOException e) {
            Log.e(TAG, "加载MTL文件失败: " + mtlFileName, e);
            return false;
        } catch (NumberFormatException e) {
            Log.e(TAG, "解析MTL文件数值失败: " + mtlFileName, e);
            return false;
        }
    }
    
    /**
     * 获取指定名称的材质
     * @param materialName 材质名称
     * @return 材质对象，如果不存在返回null
     */
    public Material getMaterial(String materialName) {
        return materials.get(materialName);
    }
    
    /**
     * 获取所有材质
     * @return 材质映射表
     */
    public Map<String, Material> getAllMaterials() {
        return materials;
    }
    
    /**
     * 获取默认材质（如果没有找到指定材质时使用）
     * @return 默认材质
     */
    public Material getDefaultMaterial() {
        Material defaultMaterial = new Material("default");
        defaultMaterial.diffuseColor = new float[]{0.8f, 0.8f, 0.8f};
        defaultMaterial.ambientColor = new float[]{0.2f, 0.2f, 0.2f};
        defaultMaterial.specularColor = new float[]{0.5f, 0.5f, 0.5f};
        defaultMaterial.shininess = 32.0f;
        defaultMaterial.transparency = 1.0f;
        return defaultMaterial;
    }
    
    /**
     * 清空所有材质
     */
    public void clear() {
        materials.clear();
        currentMaterialName = null;
    }
    
    /**
     * 检查是否有材质数据
     * @return 是否包含材质
     */
    public boolean hasMaterials() {
        return !materials.isEmpty();
    }
    
    /**
     * 获取材质数量
     * @return 材质数量
     */
    public int getMaterialCount() {
        return materials.size();
    }
}
