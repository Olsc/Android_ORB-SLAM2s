import struct
import json
import sys
import os

def parse_vtonax_profile(file_path, output_json):
    if not os.path.exists(file_path):
        print(f"错误：未找到文件 {file_path}")
        return

    with open(file_path, 'rb') as f:
        # 读取文件头
        magic = f.read(4)
        if magic != b'VPRO':
            print("错误：无效的魔数。预期为 'VPRO'。")
            return
        
        version = struct.unpack('I', f.read(4))[0]
        print(f"分析器版本：{version}")

        name_map = {}
        events = []
        
        # 持续读取直到文件末尾
        while True:
            marker_byte = f.read(1)
            if not marker_byte:
                break
            
            marker = struct.unpack('B', marker_byte)[0]
            
            if marker == 0xFF: # 名称映射记录
                name_id = struct.unpack('I', f.read(4))[0]
                name_len = struct.unpack('I', f.read(4))[0]
                name = f.read(name_len).decode('utf-8')
                name_map[name_id] = name
            elif marker == 0xEE: # 事件记录
                data = f.read(4 + 4 + 8 + 1)
                if len(data) < 17:
                    break
                
                name_id, tid, ts_ns, ev_type = struct.unpack('IIQB', data)
                
                name = name_map.get(name_id, f"未知函数_{name_id}")
                ph = "B" if ev_type == 0 else "E"
                
                events.append({
                    "name": name,
                    "ph": ph,
                    "ts": ts_ns / 1000.0, # 转换为微秒（Chrome Tracing 格式要求）
                    "tid": tid,
                    "pid": 1
                })

    print(f"解析完成，共处理 {len(events)} 个事件。")
    
    with open(output_json, 'w') as f:
        json.dump(events, f)
    
    print(f"成功！输出已保存至 {output_json}")
    print("查看方法：打开 Chrome 浏览器，进入 'chrome://tracing'，然后将 JSON 文件拖入即可。")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python vtonax_viewer.py <输入二进制文件> [输出JSON文件]")
    else:
        input_file = sys.argv[1]
        output_file = sys.argv[2] if len(sys.argv) > 2 else "profile_trace.json"
        parse_vtonax_profile(input_file, output_file)
