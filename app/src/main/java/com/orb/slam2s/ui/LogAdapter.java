package com.orb.slam2s.ui;

import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import com.orb.slam2s.R;

import java.util.ArrayList;
import java.util.List;

/**
 * 日志适配器，用于在界面上的 RecyclerView 中显示系统日志
 */
public class LogAdapter extends RecyclerView.Adapter<LogAdapter.LogViewHolder> {

    /**
     * 单条日志信息的数据结构
     */
    public static class LogItem {
        public String originalLine;
        public String timestamp;
        public String level;
        public String tag;
        public String content;
        public int count = 1;

        // 用于模糊匹配（合并相似日志）
        private String fuzzySignature;

        public LogItem(String line) {
            this.originalLine = line;
            parseLine(line);
            this.fuzzySignature = generateFuzzySignature(this.content);
        }

        private void parseLine(String line) {
            try {
                if (line.length() > 18 && line.charAt(18) == ' ') {
                    this.timestamp = line.substring(6, 18); // 分:秒.毫秒，忽略月-日
                } else {
                    this.timestamp = "";
                }

                int levelIdx = -1;
                char[] levels = { 'V', 'D', 'I', 'W', 'E', 'F', 'A' };

                int slashIdx = line.indexOf('/', 18);
                if (slashIdx > 0 && slashIdx > 18) {
                    char l = line.charAt(slashIdx - 1);
                    for (char c : levels) {
                        if (c == l) {
                            this.level = String.valueOf(c);
                            levelIdx = slashIdx - 1;
                            break;
                        }
                    }

                    if (this.level != null) {
                        int colonIdx = line.indexOf(':', slashIdx);
                        if (colonIdx > slashIdx) {
                            this.tag = line.substring(slashIdx + 1, colonIdx).trim();
                            // 如果 Tag 中包含 PID 则移除，例如 "Tag( 123)"
                            int pidStart = this.tag.indexOf('(');
                            if (pidStart > 0) {
                                this.tag = this.tag.substring(0, pidStart).trim();
                            }
                            this.content = line.substring(colonIdx + 1).trim();
                        } else {
                            // 回退处理
                            this.content = line.substring(slashIdx + 1).trim();
                        }
                    }
                }

                if (this.level == null) {
                    this.level = "V"; // 默认级别
                    this.content = line;
                    if (this.timestamp.length() > 0 && line.length() > 19) {
                        this.content = line.substring(19);
                    }
                }
            } catch (Exception e) {
                this.level = "V";
                this.content = line;
            }
        }

        private String generateFuzzySignature(String content) {
            if (content == null)
                return "";
            // 将数字替换为 #，以便匹配类似 "已处理第 1 帧" 和 "已处理第 2 帧" 的日志
            return content.replaceAll("\\d+", "#");
        }

        public boolean isSimilar(LogItem other) {
            if (other == null)
                return false;
            // 必须级别相同且模糊处理后的内容相同
            return this.level.equals(other.level) &&
                    this.fuzzySignature.equals(other.fuzzySignature);
        }
    }

    private List<LogItem> logList = new ArrayList<>();
    private RecyclerView recyclerView;

    @NonNull
    @Override
    public LogViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        View view = LayoutInflater.from(parent.getContext())
                .inflate(R.layout.item_log_message, parent, false);
        return new LogViewHolder(view);
    }

    @Override
    public void onBindViewHolder(@NonNull LogViewHolder holder, int position) {
        LogItem item = logList.get(position);
        holder.timeText.setText(item.timestamp);
        holder.contentText.setText(item.content);

        // 根据级别设置颜色
        int color;
        switch (item.level) {
            case "E":
            case "F": // Fatal
                color = 0xFFF44336; // Red 500
                break;
            case "W":
                color = 0xFFFFEB3B; // Yellow 500
                break;
            case "I":
                color = 0xFF2196F3; // Blue 500
                break;
            case "D":
                color = 0xFFFFFFFF; // White
                break;
            case "V":
            default:
                color = 0xFFBDBDBD; // Grey 400
                break;
        }
        holder.contentText.setTextColor(color);

        if (item.count > 1) {
            holder.countText.setVisibility(View.VISIBLE);
            holder.countText.setText("x" + item.count);
        } else {
            holder.countText.setVisibility(View.GONE);
        }
    }

    @Override
    public int getItemCount() {
        return logList.size();
    }

    @Override
    public void onAttachedToRecyclerView(@NonNull RecyclerView recyclerView) {
        super.onAttachedToRecyclerView(recyclerView);
        this.recyclerView = recyclerView;
    }

    public void addLog(String line) {
        LogItem newItem = new LogItem(line);

        // 检查过滤（如果内容为空则忽略）
        if (newItem.content.isEmpty())
            return;

        // 尝试与最后一条日志进行模糊合并
        if (!logList.isEmpty()) {
            LogItem lastItem = logList.get(logList.size() - 1);
            if (lastItem.isSimilar(newItem)) {
                lastItem.count++;
                lastItem.timestamp = newItem.timestamp; // 更新时间为最新
                lastItem.content = newItem.content; // 更新内容（显示最新的数值）
                notifyItemChanged(logList.size() - 1);
                return;
            }
        }

        logList.add(newItem);
        // 限制日志列表大小，防止内存溢出
        if (logList.size() > 1000) {
            logList.remove(0);
            notifyItemRemoved(0);
        } else {
            notifyItemInserted(logList.size() - 1);
        }

        // 自动滚动到底部
        if (recyclerView != null) {
            recyclerView.scrollToPosition(logList.size() - 1);
        }
    }

    public void clear() {
        int size = logList.size();
        logList.clear();
        notifyItemRangeRemoved(0, size);
    }

    static class LogViewHolder extends RecyclerView.ViewHolder {
        TextView timeText;
        TextView contentText;
        TextView countText;

        public LogViewHolder(@NonNull View itemView) {
            super(itemView);
            timeText = itemView.findViewById(R.id.log_time);
            contentText = itemView.findViewById(R.id.log_content);
            countText = itemView.findViewById(R.id.log_count);
        }
    }
}
