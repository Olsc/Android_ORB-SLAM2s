package com.orb.slam2s.ui;

import android.graphics.Color;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import com.orb.slam2s.R;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class LogAdapter extends RecyclerView.Adapter<LogAdapter.LogViewHolder> {

    public static class LogItem {
        public String originalLine;
        public String timestamp;
        public String level;
        public String tag;
        public String content;
        public int count = 1;

        // For fuzzy matching
        private String fuzzySignature;

        public LogItem(String line) {
            this.originalLine = line;
            parseLine(line);
            this.fuzzySignature = generateFuzzySignature(this.content);
        }

        private void parseLine(String line) {
            // Typical format: 12-16 14:14:36.123 V/Tag( 1234): Message
            // Or: 12-16 14:14:36.123 V/Tag: Message

            // Regex to capture timestamp and level
            // Group 1: Timestamp (MM-DD HH:MM:SS.mmm)
            // Group 2: Level (V, D, I, W, E, A)
            // Group 3: Tag
            // Group 4: PID (Optional)
            // Group 5: Message

            // Simple robust parsing:
            // 1. Extract Time (first 18 chars usually)
            // 2. Look for LEVEL/TAG

            try {
                if (line.length() > 18 && line.charAt(18) == ' ') {
                    this.timestamp = line.substring(6, 18); // HH:MM:SS.mmm, ignoring MM-DD
                } else {
                    this.timestamp = "";
                }

                // Find Level marker
                // Search for " V/" or " D/" or just start scanning
                // logcat -v time puts level after time + space

                int levelIdx = -1;
                char[] levels = { 'V', 'D', 'I', 'W', 'E', 'F', 'A' };

                // Assuming standard "MM-DD HH:MM:SS.mmm L/Tag: msg"
                // The level char is usually at index 19 or 20 depends on spacing
                // Regex might be cleaner

                // Let's use simple logic: find the first '/' after the timestamp
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
                            // Remove PID if present in tag e.g. "Tag( 123)"
                            int pidStart = this.tag.indexOf('(');
                            if (pidStart > 0) {
                                this.tag = this.tag.substring(0, pidStart).trim();
                            }
                            this.content = line.substring(colonIdx + 1).trim();
                        } else {
                            // Fallback
                            this.content = line.substring(slashIdx + 1).trim();
                        }
                    }
                }

                if (this.level == null) {
                    this.level = "V"; // Default
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
            // Replace numbers with # to match "Frame 1 processed" and "Frame 2 processed"
            return content.replaceAll("\\d+", "#");
        }

        public boolean isSimilar(LogItem other) {
            if (other == null)
                return false;
            // Must match level and fuzzy content
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

        // Color based on level
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

        // Check filtering (ignore simple noise strings if needed)
        if (newItem.content.isEmpty())
            return;

        // Try fuzzy merge with last item
        if (!logList.isEmpty()) {
            LogItem lastItem = logList.get(logList.size() - 1);
            if (lastItem.isSimilar(newItem)) {
                lastItem.count++;
                lastItem.timestamp = newItem.timestamp; // Update time to latest
                lastItem.content = newItem.content; // Update content (showing latest numbers)
                notifyItemChanged(logList.size() - 1);
                return;
            }
        }

        logList.add(newItem);
        // Limit size
        if (logList.size() > 1000) {
            logList.remove(0);
            notifyItemRemoved(0);
        } else {
            notifyItemInserted(logList.size() - 1);
        }

        // Auto scroll
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
