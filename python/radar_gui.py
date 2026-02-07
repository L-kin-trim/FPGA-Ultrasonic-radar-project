#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
超声波雷达上位机程序
功能：
1. 接收UART串口数据（距离和角度）
2. 显示扇形雷达界面
3. 参数设置（量程、测距时间、旋转角度）
"""

import serial
import serial.tools.list_ports
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import re
import time
from statistics import median

class RadarGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("超声波雷达系统")
        self.root.geometry("1200x700")
        
        # 串口相关
        self.serial_port = None
        self.serial_thread = None
        self.is_connected = False
        self.serial_lock = threading.Lock()
        
        # 数据存储
        self.distance_data = []  # [(angle, distance), ...]
        self.current_distance = 0
        self.current_angle = 0
        
        # 参数设置（默认值）
        self.max_range = 3000  # 最大量程，单位mm，默认3m
        self.measure_time = 0.5  # 测距时间，单位秒，默认0.5s
        self.angle_step = 10  # 旋转角度步进，单位度，默认10度
        self.min_angle = 0  # 扫描最小角度
        self.max_angle = 180  # 扫描最大角度
        self.scan_mode = 0  # 0: 往返, 1: 单向, 2: 固定角度
        self.auto_range_enabled = False
        self.multi_measure_enabled = False
        self.multi_measure_count = 3
        self.stability_enabled = False
        self.stability_window = 5
        self.stability_history = {}
        self.buzzer_enabled = False
        self.view_mode_var = tk.IntVar(value=180)
        self.point_size_var = tk.IntVar(value=4)
        self.pending_measure_time = True
        self.pending_angle_step = True
        self.pending_min_angle = True
        self.pending_max_angle = True
        self.pending_scan_mode = True
        self.pending_auto_range = True
        self.pending_multi_measure = True
        self.pending_multi_measure_count = True
        self.pending_buzzer = True
        self.pending_max_range = True
        
        # 创建界面
        self.create_widgets()
        
        # 启动数据接收线程
        self.start_data_thread()
    
    def create_widgets(self):
        # 基础样式
        self.root.configure(bg="#f5f7fb")
        style = ttk.Style()
        try:
            for theme in ("vista", "xpnative", "clam"):
                if theme in style.theme_names():
                    style.theme_use(theme)
                    break
        except tk.TclError:
            pass
        style.configure("App.TFrame", background="#f5f7fb")
        style.configure("App.TLabelframe", background="#f5f7fb")
        style.configure("App.TLabelframe.Label", font=("Arial", 11, "bold"))
        style.configure("Title.TLabel", font=("Arial", 14, "bold"))
        style.configure("TLabel", background="#f5f7fb")
        style.configure("TCheckbutton", background="#f5f7fb")
        style.configure("TButton", padding=(10, 4))

        # 主框架
        main_frame = ttk.Frame(self.root, padding=(16, 12), style="App.TFrame")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 左侧：雷达显示区域
        left_frame = ttk.Frame(main_frame, style="App.TFrame")
        left_frame.grid(row=0, column=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))
        left_frame.columnconfigure(0, weight=1)
        left_frame.rowconfigure(1, weight=1)
        
        radar_label = ttk.Label(left_frame, text="雷达扫描界面", style="Title.TLabel")
        radar_label.grid(row=0, column=0, sticky=tk.N, pady=(0, 8))
        
        canvas_container = ttk.Frame(left_frame, style="App.TFrame")
        canvas_container.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        canvas_container.columnconfigure(0, weight=1)
        canvas_container.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(
            canvas_container,
            width=600,
            height=600,
            bg="black",
            highlightthickness=1,
            highlightbackground="#374151",
            bd=0,
        )
        self.canvas.grid(row=0, column=0)

        control_frame = ttk.Frame(left_frame, style="App.TFrame")
        control_frame.grid(row=2, column=0, sticky=tk.W, pady=(6, 0))
        ttk.Label(control_frame, text="视图:").pack(side=tk.LEFT)
        ttk.Radiobutton(
            control_frame,
            text="180°",
            variable=self.view_mode_var,
            value=180,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Radiobutton(
            control_frame,
            text="360°",
            variable=self.view_mode_var,
            value=360,
        ).pack(side=tk.LEFT, padx=4)

        slider_frame = ttk.Frame(left_frame, style="App.TFrame")
        slider_frame.grid(row=3, column=0, sticky=tk.W, pady=(4, 0))
        ttk.Label(slider_frame, text="目标点大小:").pack(side=tk.LEFT)
        tk.Scale(
            slider_frame,
            from_=2,
            to=10,
            orient=tk.HORIZONTAL,
            variable=self.point_size_var,
            length=180,
            showvalue=True,
            relief=tk.FLAT,
            background="#f5f7fb",
            highlightthickness=0,
        ).pack(side=tk.LEFT, padx=6)
        
        # 右侧：数据和控制区域
        right_frame = ttk.Frame(main_frame, style="App.TFrame")
        right_frame.grid(row=0, column=1, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 串口连接区域
        conn_frame = ttk.LabelFrame(right_frame, text="串口连接", padding="10", style="App.TLabelframe")
        conn_frame.pack(fill=tk.X, pady=(0, 8))
        
        ttk.Label(conn_frame, text="串口:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(conn_frame, width=15)
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()
        
        ttk.Label(conn_frame, text="波特率:").grid(row=0, column=2, padx=5)
        self.baud_combo = ttk.Combobox(conn_frame, values=["9600", "115200"], width=10, state="readonly")
        self.baud_combo.set("115200")
        self.baud_combo.grid(row=0, column=3, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=5)
        
        refresh_btn = ttk.Button(conn_frame, text="刷新", command=self.refresh_ports)
        refresh_btn.grid(row=0, column=5, padx=5)
        
        # 数据显示区域
        data_frame = ttk.LabelFrame(right_frame, text="接收数据", padding="10", style="App.TLabelframe")
        data_frame.pack(fill=tk.BOTH, expand=True, pady=8)
        
        text_frame = ttk.Frame(data_frame)
        text_frame.pack(fill=tk.BOTH, expand=True)
        self.data_text = tk.Text(text_frame, height=15, width=40, wrap=tk.WORD)
        self.data_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.data_text.configure(
            background="#ffffff",
            foreground="#111827",
            insertbackground="#111827",
            relief=tk.FLAT,
        )
        
        scrollbar = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.data_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.data_text.config(yscrollcommand=scrollbar.set)
        
        # 参数设置区域
        param_frame = ttk.LabelFrame(right_frame, text="参数设置", padding="10", style="App.TLabelframe")
        param_frame.pack(fill=tk.X, pady=8)
        
        # 最大量程设置
        ttk.Label(param_frame, text="最大量程(mm):").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.range_var = tk.StringVar(value=str(self.max_range))
        self.range_entry = ttk.Entry(param_frame, textvariable=self.range_var, width=15)
        self.range_entry.grid(row=0, column=1, padx=5, pady=5)
        self.range_set_btn = ttk.Button(param_frame, text="设置", command=self.set_max_range)
        self.range_set_btn.grid(row=0, column=2, padx=5, pady=5)
        
        # 测距时间设置
        ttk.Label(param_frame, text="测距时间(s):").grid(row=0, column=3, sticky=tk.W, pady=5)
        self.time_var = tk.StringVar(value=str(self.measure_time))
        time_entry = ttk.Entry(param_frame, textvariable=self.time_var, width=15)
        time_entry.grid(row=0, column=4, padx=5, pady=5)
        ttk.Button(param_frame, text="设置", command=self.set_measure_time).grid(row=0, column=5, padx=5, pady=5)
        
        # 旋转角度设置
        ttk.Label(param_frame, text="旋转角度(度):").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.angle_var = tk.StringVar(value=str(self.angle_step))
        angle_entry = ttk.Entry(param_frame, textvariable=self.angle_var, width=15)
        angle_entry.grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(param_frame, text="设置", command=self.set_angle_step).grid(row=1, column=2, padx=5, pady=5)
        
        # 扫描角度范围设置
        ttk.Label(param_frame, text="最小角度(度):").grid(row=1, column=3, sticky=tk.W, pady=5)
        self.min_angle_var = tk.StringVar(value=str(self.min_angle))
        min_angle_entry = ttk.Entry(param_frame, textvariable=self.min_angle_var, width=15)
        min_angle_entry.grid(row=1, column=4, padx=5, pady=5)
        ttk.Button(param_frame, text="设置", command=self.set_min_angle).grid(row=1, column=5, padx=5, pady=5)
        
        ttk.Label(param_frame, text="最大角度(度):").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.max_angle_var = tk.StringVar(value=str(self.max_angle))
        max_angle_entry = ttk.Entry(param_frame, textvariable=self.max_angle_var, width=15)
        max_angle_entry.grid(row=2, column=1, padx=5, pady=5)
        ttk.Button(param_frame, text="设置", command=self.set_max_angle).grid(row=2, column=2, padx=5, pady=5)
        
        # 扫描模式设置
        ttk.Label(param_frame, text="扫描模式:").grid(row=2, column=3, sticky=tk.W, pady=5)
        self.scan_mode_var = tk.StringVar(value="往返扫描")
        self.scan_mode_combo = ttk.Combobox(
            param_frame,
            values=["往返扫描", "单向扫描", "固定角度点测"],
            textvariable=self.scan_mode_var,
            width=15,
            state="readonly",
        )
        self.scan_mode_combo.grid(row=2, column=4, padx=5, pady=5)
        ttk.Button(param_frame, text="设置", command=self.set_scan_mode).grid(row=2, column=5, padx=5, pady=5)
        
        # 自动量程设置
        self.auto_range_var = tk.BooleanVar(value=self.auto_range_enabled)
        ttk.Checkbutton(
            param_frame,
            text="自动量程",
            variable=self.auto_range_var,
            command=self.toggle_auto_range,
        ).grid(row=3, column=0, columnspan=3, sticky=tk.W, pady=5)
        
        # 多次测量设置
        self.multi_measure_var = tk.BooleanVar(value=self.multi_measure_enabled)
        ttk.Checkbutton(
            param_frame,
            text="多次测量",
            variable=self.multi_measure_var,
            command=self.toggle_multi_measure,
        ).grid(row=3, column=3, sticky=tk.W, pady=5)
        self.multi_count_var = tk.StringVar(value=str(self.multi_measure_count))
        self.multi_count_entry = ttk.Entry(param_frame, textvariable=self.multi_count_var, width=15)
        self.multi_count_entry.grid(row=3, column=4, padx=5, pady=5)
        self.multi_count_btn = ttk.Button(param_frame, text="设置次数", command=self.set_multi_measure_count)
        self.multi_count_btn.grid(row=3, column=5, padx=5, pady=5)
        
        # 蜂鸣器报警
        self.buzzer_var = tk.BooleanVar(value=self.buzzer_enabled)
        ttk.Checkbutton(
            param_frame,
            text="蜂鸣器报警",
            variable=self.buzzer_var,
            command=self.toggle_buzzer,
        ).grid(row=4, column=0, columnspan=3, sticky=tk.W, pady=5)
        
        # 目标稳定性评估
        self.stability_var = tk.BooleanVar(value=self.stability_enabled)
        ttk.Checkbutton(
            param_frame,
            text="目标稳定性评估",
            variable=self.stability_var,
            command=self.toggle_stability,
        ).grid(row=4, column=3, sticky=tk.W, pady=5)
        stability_hint = ttk.Label(param_frame, text="?", cursor="question_arrow")
        stability_hint.grid(row=4, column=4, sticky=tk.W, pady=5)
        self.attach_tooltip(
            stability_hint,
            "基于MAD(中位数绝对偏差)评估波动度，\n"
            "计算CV_robust=1.4826*MAD/median并映射可信度。\n"
            "需要同一角度连续数据，窗口默认5次。",
        )
        
        # 伺服标定
        ttk.Label(param_frame, text="伺服标定:").grid(row=5, column=0, sticky=tk.W, pady=5)
        calib_frame = ttk.Frame(param_frame)
        calib_frame.grid(row=5, column=1, columnspan=2, sticky=tk.W, pady=5)
        ttk.Button(calib_frame, text="0°", command=lambda: self.calibrate_servo(0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(calib_frame, text="90°", command=lambda: self.calibrate_servo(90)).pack(side=tk.LEFT, padx=2)
        ttk.Button(calib_frame, text="180°", command=lambda: self.calibrate_servo(180)).pack(side=tk.LEFT, padx=2)
        
        # 清空数据按钮
        clear_btn = ttk.Button(param_frame, text="清空雷达数据", command=self.clear_radar)
        clear_btn.grid(row=5, column=3, columnspan=3, pady=10, sticky=tk.E)
        
        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=2)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
        right_frame.columnconfigure(0, weight=1)
        right_frame.rowconfigure(1, weight=1)
        
        # 启动雷达绘制定时器
        self.update_radar()
        self.apply_auto_range_state()
    
    def refresh_ports(self):
        """刷新串口列表"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if not self.is_connected:
            self.connect_serial()
        else:
            self.disconnect_serial()
    
    def connect_serial(self):
        """连接串口"""
        try:
            port = self.port_combo.get()
            baud = int(self.baud_combo.get())
            
            if not port:
                messagebox.showerror("错误", "请选择串口")
                return
            
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self.is_connected = True
            self.connect_btn.config(text="断开")
            self.port_combo.config(state="disabled")
            self.baud_combo.config(state="disabled")
            
            self.add_data("串口连接成功: {} @ {} baud\n".format(port, baud))
            self.sync_pending_config()
        except Exception as e:
            messagebox.showerror("错误", "连接串口失败: {}".format(str(e)))
    
    def disconnect_serial(self):
        """断开串口"""
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
        self.is_connected = False
        self.connect_btn.config(text="连接")
        self.port_combo.config(state="readonly")
        self.baud_combo.config(state="readonly")
        self.add_data("串口已断开\n")
    
    def start_data_thread(self):
        """启动数据接收线程"""
        self.data_thread_running = True
        self.data_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.data_thread.start()
    
    def read_serial_data(self):
        """读取串口数据（在独立线程中运行）"""
        buffer = ""
        while self.data_thread_running:
            if self.is_connected and self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                        buffer += data
                        
                        # 解析数据格式：D数字A数字\n
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            self.parse_data(line)
                except Exception as e:
                    self.add_data("读取错误: {}\n".format(str(e)), level="ERROR")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)
    
    def parse_data(self, line):
        """解析接收到的数据"""
        # 格式：D距离A角度
        # 例如：D1234A90
        pattern = r'D(\d+)A(\d+)'
        match = re.search(pattern, line)
        
        if match:
            distance = int(match.group(1))
            angle = int(match.group(2))
            
            self.current_distance = distance
            self.current_angle = angle
            
            # 存储数据（自动量程时不过滤量程）
            store_max = 3000 if self.auto_range_enabled else self.max_range
            if distance > 0 and distance <= store_max:
                # 更新或添加该角度的数据
                found = False
                for i, (a, d) in enumerate(self.distance_data):
                    if a == angle:
                        self.distance_data[i] = (angle, distance)
                        found = True
                        break
                if not found:
                    self.distance_data.append((angle, distance))
                if self.stability_enabled:
                    self.update_stability_history(angle, distance)
            
            if self.auto_range_enabled:
                self.update_auto_range_from_data()
            
            # 显示数据
            self.add_data("角度: {}°, 距离: {}mm\n".format(angle, distance))
        else:
            if line.strip():
                self.add_data("未识别数据: {}\n".format(line), level="WARN")
    
    def add_data(self, text, level="INFO"):
        """添加数据到显示区域"""
        level_tag = "[{}] ".format(level.lower())
        self.root.after(0, lambda: self.data_text.insert(tk.END, level_tag + text))
        self.root.after(0, lambda: self.data_text.see(tk.END))

    def send_command(self, text):
        """发送指令到FPGA"""
        if not (self.is_connected and self.serial_port and self.serial_port.is_open):
            self.add_data("未连接串口，参数未下发\n", level="WARN")
            return False
        try:
            with self.serial_lock:
                self.serial_port.write(text.encode("ascii"))
            self.add_data("下发参数: {}\n".format(text.strip()))
            return True
        except Exception as e:
            self.add_data("发送失败: {}\n".format(str(e)), level="ERROR")
            return False

    def send_measure_time(self):
        """下发测距时间配置（毫秒）"""
        ms = int(round(self.measure_time * 1000))
        return self.send_command("T{:05d}\n".format(ms))

    def send_angle_step(self):
        """下发旋转角度步进配置"""
        return self.send_command("S{:03d}\n".format(self.angle_step))
    
    def send_min_angle(self):
        """下发扫描最小角度配置"""
        return self.send_command("L{:03d}\n".format(self.min_angle))
    
    def send_max_angle(self):
        """下发扫描最大角度配置"""
        return self.send_command("H{:03d}\n".format(self.max_angle))
    
    def send_max_range_cfg(self):
        """下发量程配置（mm）"""
        return self.send_command("G{:04d}\n".format(self.max_range))
    
    def send_buzzer_enable(self):
        """下发蜂鸣器报警开关"""
        return self.send_command("B{:01d}\n".format(1 if self.buzzer_enabled else 0))
    
    def send_scan_mode(self):
        """下发扫描模式配置"""
        return self.send_command("M{:01d}\n".format(self.scan_mode))
    
    def send_auto_range(self):
        """下发自动量程开关"""
        return self.send_command("R{:01d}\n".format(1 if self.auto_range_enabled else 0))
    
    def send_multi_measure_enable(self):
        """下发多次测量开关"""
        return self.send_command("E{:01d}\n".format(1 if self.multi_measure_enabled else 0))
    
    def send_multi_measure_count(self):
        """下发多次测量次数"""
        return self.send_command("N{:02d}\n".format(self.multi_measure_count))

    def sync_pending_config(self):
        """在连接后自动下发待同步配置"""
        if self.pending_measure_time:
            if self.send_measure_time():
                self.pending_measure_time = False
        if self.pending_angle_step:
            if self.send_angle_step():
                self.pending_angle_step = False
        if self.pending_min_angle:
            if self.send_min_angle():
                self.pending_min_angle = False
        if self.pending_max_angle:
            if self.send_max_angle():
                self.pending_max_angle = False
        if self.pending_max_range:
            if self.send_max_range_cfg():
                self.pending_max_range = False
        if self.pending_scan_mode:
            if self.send_scan_mode():
                self.pending_scan_mode = False
        if self.pending_auto_range:
            if self.send_auto_range():
                self.pending_auto_range = False
        if self.pending_multi_measure:
            if self.send_multi_measure_enable():
                self.pending_multi_measure = False
        if self.pending_multi_measure_count:
            if self.send_multi_measure_count():
                self.pending_multi_measure_count = False
        if self.pending_buzzer:
            if self.send_buzzer_enable():
                self.pending_buzzer = False

    def update_auto_range_from_data(self):
        """根据当前数据自动调整量程"""
        if not self.auto_range_enabled:
            return
        valid = [d for _, d in self.distance_data if d > 0]
        if not valid:
            return
        max_distance = max(valid)
        new_range = ((max_distance + 499) // 500) * 500
        if new_range < 500:
            new_range = 500
        if new_range > 3000:
            new_range = 3000
        if new_range != self.max_range:
            self.max_range = new_range
            self.range_var.set(str(new_range))
    
    def apply_auto_range_state(self):
        """根据自动量程状态调整控件"""
        state = "disabled" if self.auto_range_enabled else "normal"
        self.range_entry.config(state=state)
        self.range_set_btn.config(state=state)

    def toggle_stability(self):
        """切换目标稳定性评估"""
        self.stability_enabled = bool(self.stability_var.get())
        if not self.stability_enabled:
            self.stability_history = {}

    def update_stability_history(self, angle, distance):
        """更新稳定性计算窗口数据"""
        if distance <= 0 or distance > 3000:
            return
        history = self.stability_history.setdefault(angle, [])
        history.append(distance)
        if len(history) > self.stability_window:
            history.pop(0)

    def compute_stability_score(self, angle):
        """基于MAD计算可信度评分"""
        samples = self.stability_history.get(angle, [])
        if len(samples) < 3:
            return None
        med = median(samples)
        if med <= 0:
            return None
        mad = median([abs(x - med) for x in samples])
        cv_robust = 1.4826 * mad / med
        cv_low = 0.02
        cv_high = 0.15
        if cv_robust <= cv_low:
            return 100
        if cv_robust >= cv_high:
            return 0
        score = 100 * (1 - (cv_robust - cv_low) / (cv_high - cv_low))
        return int(round(max(0, min(100, score))))

    def attach_tooltip(self, widget, text):
        """为控件添加悬浮提示"""
        tooltip = {"window": None}

        def show(_event):
            if tooltip["window"] is not None:
                return
            x = widget.winfo_rootx() + widget.winfo_width() + 6
            y = widget.winfo_rooty() + 6
            tw = tk.Toplevel(widget)
            tw.wm_overrideredirect(True)
            tw.wm_geometry("+%d+%d" % (x, y))
            label = tk.Label(
                tw,
                text=text,
                background="#ffffe0",
                relief=tk.SOLID,
                borderwidth=1,
                font=("Arial", 9),
                justify=tk.LEFT,
            )
            label.pack(ipadx=4, ipady=2)
            tooltip["window"] = tw

        def hide(_event):
            tw = tooltip["window"]
            if tw is not None:
                tw.destroy()
                tooltip["window"] = None

        widget.bind("<Enter>", show)
        widget.bind("<Leave>", hide)
    
    def angle_to_xy(self, angle, distance, center_x, center_y, max_range, radius):
        """将角度和距离转换为屏幕坐标（0度向右，90度向上）"""
        import math
        # 角度转换为弧度，0度向右，90度向上，180度向左，270度向下
        rad = math.radians(angle)
        # 计算显示半径（归一化到最大量程）
        max_display_range = max_range if max_range <= 3000 else 3000
        r = radius * (distance / max_display_range)
        # 极坐标转直角坐标（0度向右，90度向上）
        x = center_x + r * math.cos(rad)
        y = center_y - r * math.sin(rad)
        return x, y
    
    def update_radar(self):
        """更新雷达显示"""
        self.canvas.delete("all")
        
        # 雷达中心点
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        if width < 50:
            width = int(self.canvas["width"])
        if height < 50:
            height = int(self.canvas["height"])
        center_x = width / 2
        center_y = height / 2
        radius = min(width, height) * 0.4
        view_mode = 180 if self.view_mode_var.get() != 360 else 360
        
        import math
        
        # 绘制扇形边界（180度模式）
        if view_mode == 180:
            x0, y0 = self.angle_to_xy(0, self.max_range, center_x, center_y, self.max_range, radius)
            x180, y180 = self.angle_to_xy(180, self.max_range, center_x, center_y, self.max_range, radius)
            self.canvas.create_line(center_x, center_y, x0, y0, fill="gray", width=1)  # 0度线
            self.canvas.create_line(center_x, center_y, x180, y180, fill="gray", width=1)  # 180度线
        
        # 绘制网格线（每30度）
        for angle in range(0, view_mode, 30):
            if view_mode == 180 and angle in (0, 180):
                continue
            x, y = self.angle_to_xy(angle, self.max_range, center_x, center_y, self.max_range, radius)
            self.canvas.create_line(center_x, center_y, x, y, fill="gray", width=1)
        
        # 绘制距离圆（每500mm）
        max_display_range = self.max_range if self.max_range <= 3000 else 3000
        for r_mm in range(500, int(self.max_range) + 1, 500):
            circle_radius = radius * (r_mm / max_display_range)
            if circle_radius <= radius:
                self.canvas.create_oval(center_x - circle_radius, center_y - circle_radius,
                                       center_x + circle_radius, center_y + circle_radius,
                                       outline="gray", width=1)
                # 标注距离
                x_label, y_label = self.angle_to_xy(45, r_mm, center_x, center_y, self.max_range, radius)
                self.canvas.create_text(x_label, y_label, text="{}mm".format(r_mm), 
                                       fill="white", font=("Arial", 8))
        
        # 绘制扇形/圆形弧线
        points = []
        for angle in range(0, view_mode + 1, 5):
            x, y = self.angle_to_xy(angle, self.max_range, center_x, center_y, self.max_range, radius)
            points.append((x, y))
        if len(points) > 1:
            for i in range(len(points) - 1):
                self.canvas.create_line(points[i][0], points[i][1], 
                                       points[i+1][0], points[i+1][1], 
                                       fill="darkgray", width=1)
        
        # 绘制角度标记
        label_angles = [0, 90, 180] if view_mode == 180 else [0, 90, 180, 270]
        for angle in label_angles:
            x, y = self.angle_to_xy(angle, radius + 15, center_x, center_y, self.max_range, radius)
            self.canvas.create_text(x, y, text="{}°".format(angle), fill="white", font=("Arial", 10))
        
        # 绘制检测到的物体（红色点）
        point_size = int(self.point_size_var.get())
        for angle, distance in self.distance_data:
            if distance > 0 and distance <= self.max_range:
                x, y = self.angle_to_xy(angle, distance, center_x, center_y, self.max_range, radius)
                # 绘制红色点
                self.canvas.create_oval(
                    x - point_size,
                    y - point_size,
                    x + point_size,
                    y + point_size,
                    fill="red",
                    outline="red",
                    width=1,
                )
        
        # 绘制当前扫描角度线（绿色）
        if self.current_angle >= 0:
            x, y = self.angle_to_xy(self.current_angle, self.max_range, center_x, center_y, self.max_range, radius)
            self.canvas.create_line(center_x, center_y, x, y, fill="green", width=2)
        
        # 显示当前信息
        info_text = "角度: {}°\n距离: {}mm\n量程: {}mm".format(
            self.current_angle, self.current_distance, self.max_range)
        if self.stability_enabled:
            score = self.compute_stability_score(self.current_angle)
            if score is None:
                info_text += "\n可信度: --"
            else:
                info_text += "\n可信度: {}%".format(score)
        self.canvas.create_text(center_x, center_y + 40, text=info_text, fill="white", 
                               font=("Arial", 10), justify=tk.CENTER)
        
        # 定时更新
        self.root.after(100, self.update_radar)
    
    def set_max_range(self):
        """设置最大量程"""
        if self.auto_range_enabled:
            messagebox.showinfo("提示", "已启用自动量程，手动量程不可用")
            return
        try:
            value = int(self.range_var.get())
            if 0 < value <= 3000:
                self.max_range = value
                self.pending_max_range = True
                sent = self.send_max_range_cfg()
                if sent:
                    self.pending_max_range = False
                messagebox.showinfo("成功", "最大量程已设置为 {}mm".format(value))
            else:
                messagebox.showerror("错误", "最大量程必须在1-3000mm之间")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
    

    def set_scan_mode(self):
        """设置扫描模式"""
        mode_text = self.scan_mode_var.get()
        mode_map = {"往返扫描": 0, "单向扫描": 1, "固定角度点测": 2}
        if mode_text not in mode_map:
            messagebox.showerror("错误", "请选择有效的扫描模式")
            return
        self.scan_mode = mode_map[mode_text]
        self.pending_scan_mode = True
        sent = self.send_scan_mode()
        if sent:
            self.pending_scan_mode = False
            messagebox.showinfo("成功", "扫描模式已设置为 {}".format(mode_text))
        else:
            messagebox.showinfo("成功", "扫描模式已设置为 {}（未下发，连接后自动下发）".format(mode_text))

    def toggle_auto_range(self):
        """切换自动量程"""
        self.auto_range_enabled = bool(self.auto_range_var.get())
        self.apply_auto_range_state()
        self.pending_auto_range = True
        sent = self.send_auto_range()
        if sent:
            self.pending_auto_range = False
        if self.auto_range_enabled:
            self.update_auto_range_from_data()

    def toggle_multi_measure(self):
        """切换多次测量"""
        self.multi_measure_enabled = bool(self.multi_measure_var.get())
        self.pending_multi_measure = True
        sent = self.send_multi_measure_enable()
        if sent:
            self.pending_multi_measure = False

    def toggle_buzzer(self):
        """切换蜂鸣器报警"""
        self.buzzer_enabled = bool(self.buzzer_var.get())
        self.pending_buzzer = True
        sent = self.send_buzzer_enable()
        if sent:
            self.pending_buzzer = False

    def set_multi_measure_count(self):
        """设置多次测量次数"""
        try:
            value = int(self.multi_count_var.get())
            if 1 <= value <= 20:
                self.multi_measure_count = value
                self.pending_multi_measure_count = True
                sent = self.send_multi_measure_count()
                if sent:
                    self.pending_multi_measure_count = False
                    messagebox.showinfo("成功", "多次测量次数已设置为 {}".format(value))
                else:
                    messagebox.showinfo("成功", "多次测量次数已设置为 {}（未下发，连接后自动下发）".format(value))
            else:
                messagebox.showerror("错误", "多次测量次数必须在1-20之间")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
    
    def calibrate_servo(self, angle):
        """伺服标定角度下发"""
        if not (0 <= angle <= 180):
            self.add_data("标定角度无效: {}\n".format(angle), level="WARN")
            return
        sent = self.send_command("C{:03d}\n".format(angle))
        if sent:
            self.add_data("伺服标定角度已下发: {}°\n".format(angle))
        else:
            self.add_data("伺服标定角度未下发: {}°\n".format(angle), level="WARN")

    def set_measure_time(self):
        """设置测距时间"""
        try:
            value = float(self.time_var.get())
            if value > 0:
                ms = int(round(value * 1000))
                if ms > 60000:
                    messagebox.showerror("错误", "测距时间不能超过60s")
                    return
                self.measure_time = value
                self.pending_measure_time = True
                sent = self.send_measure_time()
                if sent:
                    self.pending_measure_time = False
                    messagebox.showinfo("成功", "测距时间已设置为 {}s 并已下发".format(value))
                else:
                    messagebox.showinfo("成功", "测距时间已设置为 {}s（未下发，连接后自动下发）".format(value))
            else:
                messagebox.showerror("错误", "测距时间必须大于0")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
    
    def set_angle_step(self):
        """设置旋转角度"""
        try:
            value = int(self.angle_var.get())
            if 0 < value <= 180:
                self.angle_step = value
                self.pending_angle_step = True
                sent = self.send_angle_step()
                if sent:
                    self.pending_angle_step = False
                    messagebox.showinfo("成功", "旋转角度已设置为 {}° 并已下发".format(value))
                else:
                    messagebox.showinfo("成功", "旋转角度已设置为 {}°（未下发，连接后自动下发）".format(value))
            else:
                messagebox.showerror("错误", "旋转角度必须在1-180度之间")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
    
    def set_min_angle(self):
        """设置扫描最小角度"""
        try:
            value = int(self.min_angle_var.get())
            if 0 <= value <= 180:
                if value > self.max_angle:
                    messagebox.showerror("错误", "最小角度不能大于最大角度")
                    return
                self.min_angle = value
                self.pending_min_angle = True
                sent = self.send_min_angle()
                if sent:
                    self.pending_min_angle = False
                    messagebox.showinfo("成功", "最小角度已设置为 {}° 并已下发".format(value))
                else:
                    messagebox.showinfo("成功", "最小角度已设置为 {}°（未下发，连接后自动下发）".format(value))
            else:
                messagebox.showerror("错误", "最小角度必须在0-180度之间")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
    
    def set_max_angle(self):
        """设置扫描最大角度"""
        try:
            value = int(self.max_angle_var.get())
            if 0 <= value <= 180:
                if value < self.min_angle:
                    messagebox.showerror("错误", "最大角度不能小于最小角度")
                    return
                self.max_angle = value
                self.pending_max_angle = True
                sent = self.send_max_angle()
                if sent:
                    self.pending_max_angle = False
                    messagebox.showinfo("成功", "最大角度已设置为 {}° 并已下发".format(value))
                else:
                    messagebox.showinfo("成功", "最大角度已设置为 {}°（未下发，连接后自动下发）".format(value))
            else:
                messagebox.showerror("错误", "最大角度必须在0-180度之间")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")
    
    def clear_radar(self):
        """清空雷达数据"""
        self.distance_data = []
        self.current_distance = 0
        self.current_angle = 0
        messagebox.showinfo("成功", "雷达数据已清空")
    
    def on_closing(self):
        """关闭程序"""
        self.data_thread_running = False
        if self.serial_port:
            self.serial_port.close()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = RadarGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
