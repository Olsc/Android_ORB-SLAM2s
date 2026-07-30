import struct
import json
import sys
import os

def parse_mentha_profile(file_path, output_json):
    if not os.path.exists(file_path):
        print(f"错误：未找到文件 {file_path}")
        return

    # 获取文件总大小用于计算解析进度
    file_size = os.path.getsize(file_path)
    print(f"正在分析二进制文件: {file_path} (大小: {file_size} 字节)")

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
        corrupted_segments = 0
        event_record_size = None
        
        # 持续读取直到文件末尾
        while True:
            current_offset = f.tell()
            marker_byte = f.read(1)
            if not marker_byte:
                break
            
            marker = struct.unpack('B', marker_byte)[0]
            
            if marker == 0xFF: # 名称映射记录
                try:
                    # 尝试读取 name_id (4字节) 和 name_len (4字节)
                    header_data = f.read(8)
                    if len(header_data) < 8:
                        print(f"警告：文件在偏移量 {f.tell()} 处意外结束（名称映射头部不足）。")
                        break
                    name_id, name_len = struct.unpack('II', header_data)
                    
                    # 极其重要的安全检查：名称长度异常（例如大于 1024 字节），通常意味着解析错位
                    if name_len > 1024 or name_len == 0:
                        raise ValueError(f"异常的名称长度: {name_len} 字节")
                        
                    raw_name = f.read(name_len)
                    if len(raw_name) < name_len:
                        print(f"警告：文件在偏移量 {f.tell()} 处意外结束（读取名称字符不足）。")
                        break
                        
                    # 解码字符串
                    try:
                        name = raw_name.decode('utf-8')
                    except UnicodeDecodeError:
                        try:
                            name = raw_name.decode('gbk')
                        except UnicodeDecodeError:
                            name = raw_name.decode('utf-8', errors='replace')
                            
                    name_map[name_id] = name
                    
                except (ValueError, struct.error) as e:
                    # 发生错位，回退并触发同步搜索
                    corrupted_segments += 1
                    f.seek(current_offset + 1)
                    resynchronize_stream(f, name_map, events)
                    
            elif marker == 0xEE: # 事件记录
                if event_record_size is None:
                    # 自动检测事件记录结构体大小 (17字节 packed 或 24字节 aligned)
                    orig_pos = f.tell()
                    lookahead = f.read(32)
                    f.seek(orig_pos) # 恢复位置
                    
                    is_17 = len(lookahead) >= 18 and lookahead[17] in (0xEE, 0xFF)
                    is_24 = len(lookahead) >= 25 and lookahead[24] in (0xEE, 0xFF)
                    
                    if is_24 and not is_17:
                        event_record_size = 24
                        print("[自动诊断] 检测到 C++ 端写入启用了内存对齐，自动启用 24 字节解析模式。")
                    else:
                        remaining = file_size - orig_pos
                        if remaining == 24:
                            event_record_size = 24
                            print("[自动诊断] 检测到 C++ 端写入启用了内存对齐，自动启用 24 字节解析模式。")
                        else:
                            event_record_size = 17
                            if is_17:
                                print("[自动诊断] 检测到紧凑的 17 字节解析格式（已 pack）。")
                
                event_data = f.read(event_record_size)
                if len(event_data) < event_record_size:
                    # 如果不足字节，可能是文件末尾或者错位，尝试回退重新同步
                    if f.tell() < file_size:
                        corrupted_segments += 1
                        f.seek(current_offset + 1)
                        resynchronize_stream(f, name_map, events)
                    else:
                        break
                    continue
                
                name_id, tid, ts_ns, ev_type = struct.unpack('IIQB', event_data[:17])
                
                # C++ 结构体对齐诊断：检查 ev_type 是否为合法的 0 或 1
                if ev_type not in (0, 1):
                    # 类型不合法，说明当前位置大概率不是一个真正的 0xEE 事件记录，或者流已经对齐失效
                    corrupted_segments += 1
                    f.seek(current_offset + 1) # 向前滑动 1 字节，重新寻找同步点
                    resynchronize_stream(f, name_map, events)
                    continue
                
                name = name_map.get(name_id, f"未知函数_{name_id}")
                ph = "B" if ev_type == 0 else "E"
                
                events.append({
                    "name": name,
                    "ph": ph,
                    "ts": ts_ns / 1000.0, # 转换为微秒
                    "tid": tid,
                    "pid": 1
                })
                
            else:
                # 读到了未知标记（非 0xFF 也非 0xEE），说明已经发生流错位！
                corrupted_segments += 1
                # 向前滑动 1 字节，尝试重新同步流
                f.seek(current_offset + 1)
                resynchronize_stream(f, name_map, events)
 
    print(f"\n--- 解析报告 ---")
    print(f"成功恢复事件总数: {len(events)}")
    print(f"已知名称映射数: {len(name_map)}")
    print(f"检测并处理的流错位/损坏段数: {corrupted_segments}")
    
    # 性能与诊断性提示
    if event_record_size == 24:
        print("\n[性能与存储优化建议]:")
        print("1. 检测到您的 C++ 端启用了内存对齐（Memory Alignment/Padding），每个事件记录占用 24 字节（比紧凑模式多消耗了 41% 存储空间）。")
        print("   虽然脚本已成功自动适配并完美解析，但强烈建议在 C++ 结构体定义上加上伪指令防止对齐以节省存储空间。例如：")
        print("   #pragma pack(push, 1)")
        print("   struct EventRecord { ... };")
        print("   #pragma pack(pop)")
    elif corrupted_segments > 0:
        print("\n[重要诊断建议]:")
        print("1. 监测到较多流错位。这通常是因为 C++ 端写入二进制时启用了内存对齐（Memory Alignment/Padding）。")
        print("   请在 C++ 结构体定义上加上伪指令防止对齐。例如：")
        print("   #pragma pack(push, 1)")
        print("   struct EventRecord { ... };")
        print("   #pragma pack(pop)")
        print("2. 或者，请确认在 C++ 中写入字符串时，写入的是 `str.c_str()` 加上 `str.length()`，而不是直接 `fwrite(&str, sizeof(str))`。")

    # 保存结果
    with open(output_json, 'w') as f:
        json.dump(events, f)
    
    print(f"\n成功！输出已保存至 {output_json}")
    print("查看方法：打开 Chrome 浏览器，进入 'chrome://tracing'，然后将 JSON 文件拖入即可。")


def resynchronize_stream(f, name_map, events):
    """
    当二进制解析流错位时，通过滑动窗口在文件中寻找下一个可能是 0xEE 或 0xFF 的同步标记。
    """
    chunk_size = 1024
    while True:
        pos = f.tell()
        data = f.read(chunk_size)
        if not data:
            break
        
        # 在当前数据块中寻找可能作为 marker 的 0xEE 或 0xFF
        for i, byte in enumerate(data):
            if byte in (0xEE, 0xFF):
                # 找到了候选标记，将文件指针定位到这里，交回给主循环尝试解析
                f.seek(pos + i)
                return
        
        # 如果整块都没找到，继续往下读


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python mentha_viewer.py <输入二进制文件> [输出JSON文件]")
    else:
        input_file = sys.argv[1]
        output_file = sys.argv[2] if len(sys.argv) > 2 else "profile_trace.json"
        parse_mentha_profile(input_file, output_file)