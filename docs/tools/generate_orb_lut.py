import re
import math
import struct
import os

def generate_lut():
    # 获取当前脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 定位到项目根目录（假设脚本在 docs/tools 目录下）
    project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
    
    # 1. 从 ORBextractor.cc 读取模式 (pattern)
    pattern_file = os.path.join(project_root, "app", "src", "main", "cpp", "src", "ORBextractor.cc")
    
    if not os.path.exists(pattern_file):
        print(f"错误：找不到文件 {pattern_file}")
        return

    with open(pattern_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # 提取 bit_pattern_31_ 的内容
    match = re.search(r'static int bit_pattern_31_\[\d+\*\d+\]\s*=\s*\{(.*?)\};', content, re.DOTALL)
    if not match:
        print("错误：在 ORBextractor.cc 中找不到 bit_pattern_31_")
        return

    pattern_str = match.group(1)
    # 移除注释
    pattern_str = re.sub(r'/\*.*?\*/', '', pattern_str)
    # 解析整数
    values = [int(x) for x in pattern_str.replace('\n', '').replace(' ', '').split(',') if x]
    
    # 检查大小：256 * 4 = 1024 个值
    if len(values) != 1024:
        print(f"错误：期望 1024 个值，但实际得到 {len(values)} 个")
        return

    # 转换为点对列表 (x, y)
    # 值为 [x0, y0, x1, y1, ...]
    point_pairs = []
    for i in range(0, len(values), 2):
        point_pairs.append((values[i], values[i+1]))
        
    print(f"从模式中提取了 {len(point_pairs)} 个点。")

    # 2. 生成查找表 (LUT)
    # [360 度][512 个点] -> {dy, dx}
    
    lut_data = bytearray()
    
    print("正在生成 LUT...")
    
    # 预计算三角函数值
    cos_table = []
    sin_table = []
    for i in range(361):
        rad = i * math.pi / 180.0
        cos_table.append(math.cos(rad))
        sin_table.append(math.sin(rad))

    for angle in range(360):
        a = cos_table[angle]
        b = sin_table[angle]
        
        for pt in point_pairs:
            x = float(pt[0])
            y = float(pt[1])
            
            # C++ 逻辑：
            # descriptorOffsetLUT[angle][i].dy = (int)(x * b + y * a + 0.5f);
            # descriptorOffsetLUT[angle][i].dx = (int)(x * a - y * b + 0.5f);
            
            val_y = x * b + y * a
            val_x = x * a - y * b
            
            # Python 的 int(float) 会向零取整。
            # (int)(val + 0.5) 对于正数结果是一致的，
            # 但对于负数：-2.9 + 0.5 = -2.4 -> int(-2.4) = -2。
            # 而将 -2.9 四舍五入到最近的整数应该是 -3。
            # 但 C++ 代码使用的是 `(int)(... + 0.5f)`。
            # 因此我们必须复制该精确行为，否则精度将不匹配。
            
            r_dy = int(val_y + 0.5)
            r_dx = int(val_x + 0.5)
            
            # 打包为 2 个 int32 (dy, dx)
            # 结构体 { int dy; int dx; }
            # 小端序
            lut_data.extend(struct.pack('<ii', r_dy, r_dx))

    # 3. 保存到文件
    output_dir = os.path.join(project_root, "app", "src", "main", "other")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    output_file = os.path.join(output_dir, "ORB_LUT.bin")
    with open(output_file, 'wb') as f:
        f.write(lut_data)
        
    print(f"LUT 已保存至 {output_file}。大小：{len(lut_data)} 字节")

if __name__ == "__main__":
    generate_lut()
