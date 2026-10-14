"""ESP32-S3 检测设备的 UDP/JSON 上位机：实时显示遥测数据并支持 CSV 导出。"""

import csv
import json
import socket
import sqlite3
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import pyqtgraph as pg
from PyQt5.QtCore import QEvent, QObject, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtWebEngineWidgets import QWebEngineSettings, QWebEngineView


# 曲线显示最近一段时间的数据；定时器只负责刷新界面，不参与网络接收。
HISTORY_SECONDS = 60
UI_REFRESH_MS = 100
CSV_FIELDS = [
    "received_at", "uptime_ms", "device_id", "rssi", "shot_count",
    "mic_raw", "mic_p2p", "mic_rms", "vibration_x", "vibration_y",
    "vibration_z", "vibration_magnitude", "vibration_jerk", "gps_valid",
    "latitude", "longitude", "altitude_m", "speed_kmph", "satellites",
]

TRACK_MAP_HTML = """<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>html,body,#map{height:100%;margin:0} #map-error{position:absolute;z-index:1000;top:12px;left:50%;transform:translateX(-50%);padding:8px 12px;background:#fff;border:1px solid #d33;color:#900;display:none}</style>
<script>window._AMapSecurityConfig={securityJsCode:__AMAP_SECURITY_CODE__};</script>
<script src="https://webapi.amap.com/loader.js"></script>
</head><body><div id="map"></div><script>
const mapError = document.createElement('div'); mapError.id='map-error'; document.body.appendChild(mapError);
let map=null, latestTrackRequest=0;
function showMapError(message){mapError.textContent=message;mapError.style.display='block';}
function hideMapError(){mapError.style.display='none';}
function initTrackMap(){
  AMapLoader.load({key:__AMAP_KEY__, version:'2.0', plugins:['AMap.ToolBar','AMap.Scale']})
    .then(function(AMap){
      map=new AMap.Map('map',{zoom:16,center:[120.2,30.3],viewMode:'2D'});
      map.addControl(new AMap.ToolBar());
      map.addControl(new AMap.Scale());
      map.on('complete',hideMapError);
      if(window.pendingTrack) showTrack(window.pendingTrack);
    }).catch(function(error){
      console.error(error); showMapError('高德地图加载失败：请检查网络、Key 和安全密钥配置。');
    });
}
function convertGpsPoints(points, requestId){
  const converted=[];
  function next(start){
    if(requestId!==latestTrackRequest) return;
    if(start>=points.length){drawTrack(points,converted);return;}
    const batch=points.slice(start,start+40);
    const gps=batch.map(point=>[Number(point.lon),Number(point.lat)]);
    AMap.convertFrom(gps,'gps',function(status,result){
      if(status!=='complete'||result.info!=='ok'||!result.locations){
        showMapError('GPS 坐标转换失败，请检查高德服务权限或网络。');return;
      }
      converted.push(...result.locations.map(location=>[location.getLng(),location.getLat()]));
      next(start+batch.length);
    });
  }
  next(0);
}
function drawTrack(points,coords){
  if(!map) return;
  hideMapError(); map.clearMap();
  if(!points||!points.length) return;
  const overlays=[],segments=[];let segment=[];
  points.forEach((point,index)=>{
    if(index>0&&point.uptimeMs-points[index-1].uptimeMs>30000){if(segment.length)segments.push(segment);segment=[];}
    segment.push(coords[index]);
  });
  if(segment.length)segments.push(segment);
  segments.forEach(path=>{if(path.length>1)overlays.push(new AMap.Polyline({path,strokeColor:'#1565c0',strokeWeight:5,strokeOpacity:0.85}));});
  const infoWindow=new AMap.InfoWindow({offset:new AMap.Pixel(0,-28)});
  function addMarker(point,position,label,color){
    const marker=new AMap.Marker({position,title:label,content:'<div style="width:16px;height:16px;border-radius:50%;background:'+color+';border:2px solid white;box-shadow:0 1px 4px #555"></div>',offset:new AMap.Pixel(-9,-9)});
    marker.on('click',function(){infoWindow.setContent(label+'<br>'+ (point.timestamp||('设备运行 '+point.uptimeMs+' ms'))+'<br>HDOP: '+(point.hdop||'未知')+', 卫星: '+point.satellites);infoWindow.open(map,position);});
    overlays.push(marker);
  }
  addMarker(points[0],coords[0],'起点','#2e7d32');
  if(points.length>1)addMarker(points[points.length-1],coords[coords.length-1],'终点','#c62828');
  map.add(overlays);
  if(points.length===1)map.setZoomAndCenter(17,coords[0]);else map.setFitView(overlays,false,[30,30,30,30],18);
}
function showTrack(points){
  const requestId=++latestTrackRequest;window.pendingTrack=points||[];
  if(!map)return;
  map.clearMap();if(!points||!points.length)return;
  convertGpsPoints(points,requestId);
}
window.initTrackMap=initTrackMap;
initTrackMap();
</script></body></html>"""


class UdpReceiver(QObject):
    """在后台线程接收 UDP 数据，并通过 Qt 信号安全地交给界面线程。"""
    packet_received = pyqtSignal(dict)
    receiver_error = pyqtSignal(str)
    listening = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self._stop = threading.Event()
        self._thread = None
        self._socket = None

    def start(self, host, port):
        """停止旧监听后，以指定地址和端口启动新的后台接收线程。"""
        self.stop()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._receive_loop, args=(host, port), daemon=True
        )
        self._thread.start()

    def stop(self):
        """设置停止标志并关闭套接字，以唤醒阻塞中的 recvfrom。"""
        self._stop.set()
        if self._socket:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None

    def _receive_loop(self, host, port):
        """绑定 UDP 端口、解析 JSON 包，并报告非法数据或套接字错误。"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket = sock
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((host, port))
            sock.settimeout(0.5)
            self.listening.emit(True, f"监听中：{host}:{port}")
            while not self._stop.is_set():
                try:
                    data, address = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError:
                    if not self._stop.is_set():
                        raise
                    break
                try:
                    packet = json.loads(data.decode("utf-8-sig").strip())
                    if isinstance(packet, dict):
                        packet["_sender"] = f"{address[0]}:{address[1]}"
                        packet["_sender_ip"] = address[0]
                        packet["_sender_port"] = address[1]
                        self.packet_received.emit(packet)
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    self.receiver_error.emit(f"收到非 JSON 数据：{exc}")
            self.listening.emit(False, "已停止监听")
        except OSError as exc:
            self.listening.emit(False, "监听失败")
            self.receiver_error.emit(f"无法监听 {host}:{port}：{exc}")
        finally:
            try:
                sock.close()
            except OSError:
                pass
            if self._socket is sock:
                self._socket = None


class MonitorWindow(QMainWindow):
    """主窗口：呈现状态摘要、实时曲线、最近事件并导出历史数据。"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Gun Detection WiFi Monitor")
        self.resize(1280, 850)
        self.receiver = UdpReceiver()
        self.receiver.packet_received.connect(self.on_packet)
        self.receiver.receiver_error.connect(self.show_error)
        self.receiver.listening.connect(self.on_listening)
        self.samples = deque(maxlen=20000)
        self.x_values = deque(maxlen=20000)
        self.start_uptime = None
        self.last_packet_time = None
        self.shot_count = 0
        self.track_db_path = Path.home() / ".gun_wifi_monitor" / "tracks.sqlite3"
        self.track_db_path.parent.mkdir(parents=True, exist_ok=True)
        self.track_db = sqlite3.connect(self.track_db_path)
        self._init_track_database()
        self._build_ui()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_display)
        self.timer.start(UI_REFRESH_MS)

    def _build_ui(self):
        """创建连接控制栏、状态卡片和传感器曲线。"""
        root = QWidget()
        main = QVBoxLayout(root)

        connection = QHBoxLayout()
        connection.addWidget(QLabel("监听地址"))
        self.host_edit = QLineEdit("10.126.104.83")
        self.host_edit.setMaximumWidth(150)
        connection.addWidget(self.host_edit)
        connection.addWidget(QLabel("端口"))
        self.port_box = QSpinBox()
        self.port_box.setRange(1, 65535)
        self.port_box.setValue(8888)
        connection.addWidget(self.port_box)
        self.start_button = QPushButton("开始监听")
        self.start_button.clicked.connect(self.toggle_listening)
        connection.addWidget(self.start_button)
        self.save_button = QPushButton("导出 CSV")
        self.save_button.clicked.connect(self.export_csv)
        connection.addWidget(self.save_button)
        self.clear_button = QPushButton("清空曲线")
        self.clear_button.clicked.connect(self.clear_history)
        connection.addWidget(self.clear_button)
        self.connection_label = QLabel("尚未监听")
        connection.addWidget(self.connection_label, 1)
        main.addLayout(connection)

        summary = QGridLayout()
        self.values = {}
        fields = [
            ("device", "设备"), ("rssi", "WiFi 信号"), ("shots", "检测次数"),
            ("flags", "检测标志"), ("gps", "GPS 状态"),
            ("coordinates", "经纬度"), ("altitude", "海拔"),
            ("speed", "速度"), ("satellites", "卫星数"),
            ("track", "轨迹缓存"),
            ("mic", "麦克风"), ("accel", "三轴原始值"),
            ("vibration", "振动量 / 变化量"),
            ("last_shot", "最近事件"), ("last_seen", "最近数据"),
        ]
        for index, (key, title) in enumerate(fields):
            box = QGroupBox(title)
            layout = QVBoxLayout(box)
            label = QLabel("—")
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            label.setWordWrap(True)
            layout.addWidget(label)
            self.values[key] = label
            summary.addWidget(box, index // 4, index % 4)
        main.addLayout(summary)

        self.pages = QTabWidget()
        main.addWidget(self.pages, 1)

        plot_page = QWidget()
        plot_layout = QVBoxLayout(plot_page)
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground("w")
        plot_layout.addWidget(self.plot_widget)
        self.accel_plot = self.plot_widget.addPlot(row=0, col=0, title="三轴加速度原始 ADC 值")
        self.accel_plot.setLabel("bottom", "时间", units="s")
        self.accel_plot.showGrid(x=True, y=True, alpha=0.25)
        # 将颜色说明和可见性开关叠放在图表右上角，复选框控制对应曲线。
        self.accel_curves = [
            self.accel_plot.plot(pen=pg.mkPen("#d62728", width=1.5)),
            self.accel_plot.plot(pen=pg.mkPen("#2ca02c", width=1.5)),
            self.accel_plot.plot(pen=pg.mkPen("#1f77b4", width=1.5)),
        ]
        legend_panel = QWidget(self.plot_widget.viewport())
        legend_layout = QVBoxLayout(legend_panel)
        legend_layout.setContentsMargins(8, 5, 8, 5)
        legend_layout.setSpacing(2)
        legend_panel.setStyleSheet(
            "QWidget { background-color: rgba(255, 255, 255, 220); "
            "border: 1px solid #bbbbbb; border-radius: 4px; }"
        )
        self.axis_checks = []
        axis_items = [("X  红色", "#d62728"), ("Y  绿色", "#2ca02c"), ("Z  蓝色", "#1f77b4")]
        for curve, (label, color) in zip(self.accel_curves, axis_items):
            check = QCheckBox(label, legend_panel)
            check.setChecked(True)
            check.setStyleSheet(f"QCheckBox {{ color: {color}; font-weight: bold; }}")
            check.toggled.connect(curve.setVisible)
            legend_layout.addWidget(check)
            self.axis_checks.append(check)
        legend_panel.adjustSize()
        self.legend_panel = legend_panel
        self.plot_widget.viewport().installEventFilter(self)
        self._position_legend()

        map_page = QWidget()
        map_layout = QVBoxLayout(map_page)
        map_controls = QHBoxLayout()
        map_controls.addWidget(QLabel("已保存轨迹"))
        self.track_combo = QComboBox()
        self.track_combo.currentIndexChanged.connect(self.show_selected_track)
        map_controls.addWidget(self.track_combo, 1)
        refresh_tracks_button = QPushButton("刷新列表")
        refresh_tracks_button.clicked.connect(self.refresh_track_sessions)
        map_controls.addWidget(refresh_tracks_button)
        export_track_button = QPushButton("导出 GeoJSON")
        export_track_button.clicked.connect(self.export_selected_track)
        map_controls.addWidget(export_track_button)
        map_layout.addLayout(map_controls)
        self.track_info = QLabel("轨迹数据保存在本机用户目录。地图底图需要网络连接。")
        map_layout.addWidget(self.track_info)
        self.map_view = QWebEngineView()
        self.map_view.settings().setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
        self.map_view.setHtml(self._track_map_html())
        self.map_view.loadFinished.connect(lambda _ok: self.show_selected_track())
        map_layout.addWidget(self.map_view, 1)

        shot_page = QWidget()
        shot_layout = QVBoxLayout(shot_page)
        shot_controls = QHBoxLayout()
        shot_controls.addWidget(QLabel("电脑已接收并保存的射击事件"))
        refresh_shots_button = QPushButton("刷新列表")
        refresh_shots_button.clicked.connect(self.refresh_shot_events)
        shot_controls.addWidget(refresh_shots_button)
        export_shots_button = QPushButton("导出射击事件 CSV")
        export_shots_button.clicked.connect(self.export_shot_events)
        shot_controls.addWidget(export_shots_button)
        shot_layout.addLayout(shot_controls)
        shot_layout.addWidget(QLabel(
            "事件写入电脑数据库后会自动确认设备上传；导出会另存 CSV，不会删除电脑数据库记录。"
        ))
        self.shot_table = QTableWidget(0, 10)
        self.shot_table.setHorizontalHeaderLabels([
            "接收时间", "设备", "事件编号", "检测次数", "事件时间", "GPS 状态",
            "纬度", "经度", "声音 P2P", "振动峰值",
        ])
        self.shot_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.shot_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.shot_table.setAlternatingRowColors(True)
        self.shot_table.horizontalHeader().setStretchLastSection(True)
        shot_layout.addWidget(self.shot_table, 1)

        self.pages.addTab(plot_page, "加速度曲线")
        self.pages.addTab(map_page, "GPS 轨迹地图")
        self.pages.addTab(shot_page, "射击事件")
        self.refresh_track_sessions()
        self.refresh_shot_events()
        main.setContentsMargins(8, 8, 8, 8)
        self.setCentralWidget(root)

    def _track_map_html(self):
        """读取本地高德密钥配置并填入地图页面，密钥文件不纳入版本控制。"""
        config_path = Path(__file__).with_name("amap_config.json")
        try:
            config = json.loads(config_path.read_text(encoding="utf-8-sig"))
            api_key = config["key"]
            security_code = config["securityJsCode"]
            if not api_key or not security_code:
                raise ValueError("高德 Key 或安全密钥为空")
        except (OSError, json.JSONDecodeError, KeyError, ValueError) as exc:
            QMessageBox.warning(
                self, "高德地图配置缺失",
                f"无法读取地图密钥配置：{exc}\n请在 data_show/amap_config.json 中设置 key 和 securityJsCode。",
            )
            api_key = ""
            security_code = ""
        return TRACK_MAP_HTML.replace("__AMAP_KEY__", json.dumps(api_key)).replace(
            "__AMAP_SECURITY_CODE__", json.dumps(security_code)
        )

    def _init_track_database(self):
        """创建本机轨迹数据库；会话和点均使用唯一键支持安全去重。"""
        self.track_db.execute("PRAGMA journal_mode=WAL")
        self.track_db.execute(
            """CREATE TABLE IF NOT EXISTS track_points (
                device_id TEXT NOT NULL,
                session_id TEXT NOT NULL,
                sequence INTEGER NOT NULL,
                uptime_ms INTEGER NOT NULL,
                utc_date INTEGER NOT NULL,
                utc_time_ms INTEGER NOT NULL,
                latitude REAL NOT NULL,
                longitude REAL NOT NULL,
                hdop REAL NOT NULL,
                satellites INTEGER NOT NULL,
                received_at TEXT NOT NULL,
                PRIMARY KEY (device_id, session_id, sequence)
            )"""
        )
        self.track_db.execute(
            """CREATE TABLE IF NOT EXISTS shot_events (
                device_id TEXT NOT NULL,
                session_id TEXT NOT NULL,
                sequence INTEGER NOT NULL,
                shot_id INTEGER NOT NULL,
                shot_count INTEGER NOT NULL,
                uptime_ms INTEGER NOT NULL,
                uptime_us INTEGER NOT NULL,
                timestamp_source TEXT NOT NULL,
                utc_date INTEGER NOT NULL,
                utc_time_ms INTEGER NOT NULL,
                voice_p2p INTEGER NOT NULL,
                voice_rms REAL NOT NULL,
                voice_rise_ratio REAL NOT NULL,
                accel_x INTEGER NOT NULL,
                accel_y INTEGER NOT NULL,
                accel_z INTEGER NOT NULL,
                accel_peak REAL NOT NULL,
                accel_jerk REAL NOT NULL,
                accel_saturated INTEGER NOT NULL,
                gps_valid INTEGER NOT NULL,
                latitude REAL NOT NULL,
                longitude REAL NOT NULL,
                altitude_m REAL NOT NULL,
                speed_kmph REAL NOT NULL,
                hdop REAL NOT NULL,
                satellites INTEGER NOT NULL,
                gps_age_ms INTEGER NOT NULL,
                received_at TEXT NOT NULL,
                PRIMARY KEY (device_id, session_id, sequence)
            )"""
        )
        self.track_db.commit()

    def _position_legend(self):
        """将轴颜色图例固定在绘图区域右上角。"""
        if not hasattr(self, "legend_panel"):
            return
        margin = 14
        self.legend_panel.move(
            self.plot_widget.viewport().width() - self.legend_panel.width() - margin,
            margin,
        )

    def resizeEvent(self, event):
        """窗口尺寸变化时同步移动右上角图例。"""
        super().resizeEvent(event)
        self._position_legend()

    def eventFilter(self, watched, event):
        """绘图区视口尺寸改变时重新定位图例。"""
        if watched is self.plot_widget.viewport() and event.type() == QEvent.Resize:
            self._position_legend()
        return super().eventFilter(watched, event)

    def toggle_listening(self):
        """根据当前接收线程状态开始或停止监听。"""
        if self.receiver._thread and self.receiver._thread.is_alive() and not self.receiver._stop.is_set():
            self.receiver.stop()
            self.start_button.setText("开始监听")
            return
        self.receiver.start(self.host_edit.text().strip() or "0.0.0.0", self.port_box.value())

    def on_listening(self, active, message):
        self.connection_label.setText(message)
        self.start_button.setText("停止监听" if active else "开始监听")

    def on_packet(self, packet):
        """记录收到数据的时间，并按协议类型分发状态包或射击事件包。"""
        self.last_packet_time = time.time()
        kind = packet.get("type", "?")
        self.values["last_seen"].setText(
            f"{datetime.now().strftime('%H:%M:%S')}  {packet.get('_sender', '')}  [{kind}]"
        )
        if kind == "status":
            self.handle_status(packet)
        elif kind == "shot":
            self.handle_shot(packet)
        elif kind == "track_chunk":
            self.handle_track_chunk(packet)

    def handle_track_chunk(self, packet):
        """先持久化一批轨迹点，再回 ACK；未提交的数据不会确认给设备。"""
        device_id = str(packet.get("deviceId", ""))
        session_id = str(packet.get("sessionId", ""))
        points = packet.get("points")
        if not device_id or not session_id or not isinstance(points, list) or not points:
            self.show_error("收到格式无效的轨迹数据")
            return

        received_at = datetime.now().isoformat(timespec="milliseconds")
        rows = []
        previous_sequence = None
        try:
            for point in points:
                sequence = int(point["seq"])
                latitude = float(point["lat"])
                longitude = float(point["lon"])
                if not -90 <= latitude <= 90:
                    raise ValueError("纬度超出范围")
                if not -180 <= longitude <= 180 or sequence <= 0:
                    raise ValueError("经度或序号超出范围")
                if previous_sequence is not None and sequence != previous_sequence + 1:
                    raise ValueError("批次内轨迹序号不连续")
                previous_sequence = sequence
                rows.append((
                    device_id, session_id, sequence, int(point.get("uptimeMs", 0)),
                    int(point.get("utcDate", 0)), int(point.get("utcTimeMs", 0)),
                    latitude, longitude, float(point.get("hdop", 0.0)),
                    int(point.get("satellites", 0)), received_at,
                ))
            self.track_db.executemany(
                """INSERT OR IGNORE INTO track_points
                   (device_id, session_id, sequence, uptime_ms, utc_date, utc_time_ms,
                    latitude, longitude, hdop, satellites, received_at)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                rows,
            )
            self.track_db.commit()
        except (KeyError, TypeError, ValueError, sqlite3.Error) as exc:
            self.track_db.rollback()
            self.show_error(f"轨迹保存失败，设备会稍后重传：{exc}")
            return

        self._send_track_ack(packet, session_id, previous_sequence)
        self.refresh_track_sessions((device_id, session_id))
        self.track_info.setText(
            f"已接收并保存 {len(rows)} 个轨迹点；数据库：{self.track_db_path}"
        )

    def _send_track_ack(self, packet, session_id, sequence):
        """将数据库提交后的确认发回设备 UDP 源地址和端口。"""
        host = packet.get("_sender_ip")
        port = packet.get("_sender_port")
        if not host or not port:
            return
        payload = json.dumps({
            "type": "track_ack", "sessionId": session_id, "seq": sequence,
        }, separators=(",", ":")).encode("utf-8")
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ack_socket:
                ack_socket.sendto(payload, (host, int(port)))
        except OSError as exc:
            self.show_error(f"轨迹已保存，但确认包发送失败：{exc}")

    def refresh_track_sessions(self, preferred=None):
        """刷新本机保存的轨迹会话列表，并尽量保留当前选择。"""
        if not hasattr(self, "track_combo"):
            return
        current = preferred or self.track_combo.currentData()
        sessions = self.track_db.execute(
            """SELECT device_id, session_id, COUNT(*), MIN(received_at), MIN(sequence)
               FROM track_points
               GROUP BY device_id, session_id
               ORDER BY MIN(received_at) DESC"""
        ).fetchall()
        self.track_combo.blockSignals(True)
        self.track_combo.clear()
        selected_index = -1
        for device_id, session_id, count, first_seen, first_sequence in sessions:
            first_point = self.track_db.execute(
                """SELECT utc_date, utc_time_ms FROM track_points
                   WHERE device_id=? AND session_id=? AND sequence=?""",
                (device_id, session_id, first_sequence),
            ).fetchone()
            if first_point and first_point[0]:
                start_time = self._track_timestamp(first_point[0], first_point[1], 0)
            else:
                received = datetime.fromisoformat(first_seen).astimezone()
                start_time = received.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + " 本地时间（电脑接收）"
            self.track_combo.addItem(
                f"{device_id} · {session_id[:8]} · {count} 点 · 起始 {start_time}",
                (device_id, session_id),
            )
            if (device_id, session_id) == current:
                selected_index = self.track_combo.count() - 1
        if selected_index < 0 and self.track_combo.count():
            selected_index = 0
        self.track_combo.setCurrentIndex(selected_index)
        self.track_combo.blockSignals(False)
        self.show_selected_track()

    @staticmethod
    def _track_timestamp(utc_date, utc_time_ms, uptime_ms):
        """将 GPS UTC 时间转换为电脑本地时间；日期无效时回退到设备运行时间。"""
        if utc_date:
            try:
                date_value = int(utc_date)
                milliseconds = int(utc_time_ms)
                if not 10_000_101 <= date_value <= 99_991_231:
                    raise ValueError("GPS 日期超出有效范围")
                if not 0 <= milliseconds < 86_400_000:
                    raise ValueError("GPS 当天时间超出有效范围")
                date = f"{date_value:08d}"
                hour, milliseconds = divmod(milliseconds, 3_600_000)
                minute, milliseconds = divmod(milliseconds, 60_000)
                second, milliseconds = divmod(milliseconds, 1_000)
                gps_utc = datetime.strptime(
                    f"{date} {hour:02d}:{minute:02d}:{second:02d}.{milliseconds:03d}",
                    "%Y%m%d %H:%M:%S.%f",
                ).replace(tzinfo=timezone.utc)
                local_time = gps_utc.astimezone()
                return local_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + " 本地时间"
            except (TypeError, ValueError, OverflowError):
                pass
        return f"设备运行 {uptime_ms / 1000:.1f} 秒"

    def _selected_track_points(self):
        key = self.track_combo.currentData()
        if not key:
            return None, []
        device_id, session_id = key
        rows = self.track_db.execute(
            """SELECT sequence, uptime_ms, utc_date, utc_time_ms, latitude, longitude,
                      hdop, satellites
               FROM track_points
               WHERE device_id=? AND session_id=? ORDER BY sequence""",
            (device_id, session_id),
        ).fetchall()
        points = [{
            "seq": row[0], "uptimeMs": row[1],
            "timestamp": self._track_timestamp(row[2], row[3], row[1]),
            "lat": row[4], "lon": row[5], "hdop": row[6], "satellites": row[7],
        } for row in rows]
        return key, points

    def show_selected_track(self, *_args):
        """把所选会话 GPS 坐标传给高德地图，转换后绘制路线和起终点。"""
        if not hasattr(self, "map_view"):
            return
        key, points = self._selected_track_points()
        if key is None:
            self.track_info.setText(
                f"暂无已上传轨迹。轨迹数据库：{self.track_db_path}；地图底图需要网络。"
            )
            points = []
        elif points:
            self.track_info.setText(
                f"{key[0]} · {key[1]}：{len(points)} 点，"
                f"{points[0]['timestamp']} 至 {points[-1]['timestamp']}"
            )
        script = "showTrack(" + json.dumps(points, separators=(",", ":")) + ");"
        self.map_view.page().runJavaScript(script)

    def export_selected_track(self):
        """将当前会话导出为 GeoJSON LineString，便于在 GIS 软件中查看。"""
        key, points = self._selected_track_points()
        if key is None or not points:
            QMessageBox.information(self, "没有轨迹", "先接收至少一个轨迹点再导出。")
            return
        default_name = f"track_{key[1]}.geojson"
        path, _ = QFileDialog.getSaveFileName(
            self, "导出轨迹", default_name, "GeoJSON 文件 (*.geojson *.json)"
        )
        if not path:
            return
        geojson = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "properties": {"deviceId": key[0], "sessionId": key[1]},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[point["lon"], point["lat"]] for point in points],
                },
            }],
        }
        try:
            with open(path, "w", encoding="utf-8") as file:
                json.dump(geojson, file, ensure_ascii=False, indent=2)
            self.connection_label.setText(f"轨迹已导出：{path}")
        except OSError as exc:
            QMessageBox.critical(self, "导出失败", str(exc))

    @staticmethod
    def number(value, digits=1):
        """将数值格式化为指定精度；缺失或非法值显示为占位符。"""
        try:
            return f"{float(value):.{digits}f}"
        except (TypeError, ValueError):
            return "—"

    def handle_status(self, packet):
        """更新实时状态卡片，并将状态样本存入曲线缓存和 CSV 缓存。"""
        mic = packet.get("mic") or {}
        vibration = packet.get("vibration") or {}
        gps = packet.get("gps") or {}
        flags = packet.get("flags") or {}
        track = packet.get("track") or {}
        uptime = packet.get("uptimeMs")
        try:
            uptime = float(uptime)
        except (TypeError, ValueError):
            uptime = time.monotonic() * 1000
        if self.start_uptime is None:
            self.start_uptime = uptime
        # 以设备运行时间为横轴，避免电脑本地时钟变化影响曲线连续性。
        elapsed = max(0.0, (uptime - self.start_uptime) / 1000.0)

        row = {
            "received_at": datetime.now().isoformat(timespec="milliseconds"),
            "uptime_ms": uptime,
            "device_id": packet.get("deviceId", ""),
            "rssi": packet.get("rssi", ""),
            "shot_count": packet.get("shotCount", ""),
            "mic_raw": mic.get("raw", ""), "mic_p2p": mic.get("p2p", ""),
            "mic_rms": mic.get("rms", ""),
            "vibration_x": vibration.get("x", ""),
            "vibration_y": vibration.get("y", ""),
            "vibration_z": vibration.get("z", ""),
            "vibration_magnitude": vibration.get("magnitude", ""),
            "vibration_jerk": vibration.get("jerk", ""),
            "gps_valid": gps.get("valid", ""),
            "latitude": gps.get("latitude", ""), "longitude": gps.get("longitude", ""),
            "altitude_m": gps.get("altitudeM", ""),
            "speed_kmph": gps.get("speedKmph", ""),
            "satellites": gps.get("satellites", ""),
        }
        self.samples.append(row)
        self.x_values.append(elapsed)
        self.values["device"].setText(str(packet.get("deviceId", "—")))
        self.values["rssi"].setText(f"{packet.get('rssi', '—')} dBm")
        self.values["shots"].setText(str(packet.get("shotCount", self.shot_count)))
        self.values["flags"].setText(
            f"声音：{'触发' if flags.get('voice') else '无'}；振动：{'触发' if flags.get('vibration') else '无'}"
        )
        valid = bool(gps.get("valid", False))
        self.values["gps"].setText("有效" if valid else "无有效定位")
        self.values["coordinates"].setText(
            f"{self.number(gps.get('latitude'), 7)}, {self.number(gps.get('longitude'), 7)}"
        )
        self.values["altitude"].setText(f"{self.number(gps.get('altitudeM'))} m")
        self.values["speed"].setText(f"{self.number(gps.get('speedKmph'))} km/h")
        self.values["satellites"].setText(str(gps.get("satellites", "—")))
        self.values["track"].setText(
            f"{'可用' if track.get('storageReady') else '不可用'}；"
            f"待上传 {track.get('pendingPoints', 0)} 点；"
            f"记录失败 {track.get('droppedPoints', 0)} 点；"
            f"射击事件存储 {'可用' if (packet.get('shotEvents') or {}).get('storageReady') else '不可用'}；"
            f"待传 {(packet.get('shotEvents') or {}).get('pending', 0)} 条；"
            f"记录失败 {(packet.get('shotEvents') or {}).get('dropped', 0)} 条"
        )
        self.values["mic"].setText(
            f"raw {mic.get('raw', '—')}   P2P {mic.get('p2p', '—')}   RMS {self.number(mic.get('rms'))}"
        )
        self.values["accel"].setText(
            f"X {vibration.get('x', '—')}   Y {vibration.get('y', '—')}   Z {vibration.get('z', '—')}"
        )
        self.values["vibration"].setText(
            f"幅值 {self.number(vibration.get('magnitude'))}   变化 {self.number(vibration.get('jerk'))}"
        )
        self.shot_count = int(packet.get("shotCount", self.shot_count) or 0)
        if len(self.x_values) > 1:
            left = max(0.0, elapsed - HISTORY_SECONDS)
            self.accel_plot.setXRange(left, max(HISTORY_SECONDS, elapsed), padding=0)

    def handle_shot(self, packet):
        """先保存持久化射击事件并回 ACK，再更新最近事件摘要。"""
        # 事件包一到就更新界面；数据库异常不能让已收到的事件在界面上消失。
        shot_id = packet.get("shotId", "?")
        self.shot_count = int(packet.get("shotCount", self.shot_count + 1) or 0)
        self.values["shots"].setText(str(self.shot_count))
        lat, lon = packet.get("latitude"), packet.get("longitude")
        location = f"GPS {self.number(lat, 7)}, {self.number(lon, 7)}" if packet.get("gpsValid") else "GPS 无效"
        self.values["last_shot"].setText(
            f"#{shot_id}  时间 {self._shot_timestamp(packet)}  声音 P2P {packet.get('voiceP2P', '—')}  "
            f"振动峰值 {self.number(packet.get('accelPeak'))}  {location}"
        )

        device_id = str(packet.get("deviceId", ""))
        session_id = str(packet.get("sessionId", ""))
        sequence = packet.get("seq")
        if device_id and session_id and sequence is not None:
            try:
                sequence = int(sequence)
                if sequence <= 0:
                    raise ValueError("射击事件序号无效")
                received_at = datetime.now().isoformat(timespec="milliseconds")
                self.track_db.execute(
                    """INSERT OR IGNORE INTO shot_events (
                        device_id, session_id, sequence, shot_id, shot_count,
                        uptime_ms, uptime_us, timestamp_source, utc_date, utc_time_ms,
                        voice_p2p, voice_rms, voice_rise_ratio, accel_x, accel_y, accel_z,
                        accel_peak, accel_jerk, accel_saturated, gps_valid, latitude,
                        longitude, altitude_m, speed_kmph, hdop, satellites, gps_age_ms, received_at
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                    (
                        device_id, session_id, sequence, int(packet.get("shotId", 0)),
                        int(packet.get("shotCount", 0)), int(packet.get("uptimeMs", 0)),
                        int(packet.get("uptimeUs", 0)), str(packet.get("timestampSource", "uptime")),
                        int(packet.get("utcDate", 0)), int(packet.get("utcTimeMs", 0)),
                        int(packet.get("voiceP2P", 0)), float(packet.get("voiceRms", 0)),
                        float(packet.get("voiceRiseRatio", 0)), int(packet.get("accelX", 0)),
                        int(packet.get("accelY", 0)), int(packet.get("accelZ", 0)),
                        float(packet.get("accelPeak", 0)), float(packet.get("accelJerk", 0)),
                        int(bool(packet.get("accelSaturated", False))),
                        int(bool(packet.get("gpsValid", False))), float(packet.get("latitude", 0)),
                        float(packet.get("longitude", 0)), float(packet.get("altitudeM", 0)),
                        float(packet.get("speedKmph", 0)), float(packet.get("hdop", 0)),
                        int(packet.get("satellites", 0)), int(packet.get("gpsAgeMs", 0xFFFFFFFF)),
                        received_at,
                    ),
                )
                self.track_db.commit()
            except (TypeError, ValueError, sqlite3.Error) as exc:
                self.track_db.rollback()
                self.show_error(f"射击事件保存失败，设备会稍后重传：{exc}")
                return
            self._send_shot_ack(packet, session_id, sequence)
            self.refresh_shot_events()

    def _send_shot_ack(self, packet, session_id, sequence):
        """射击事件提交到本机 SQLite 后，向设备确认这条 session/seq 记录。"""
        host = packet.get("_sender_ip")
        port = packet.get("_sender_port")
        if not host or not port:
            return
        payload = json.dumps({
            "type": "shot_ack", "sessionId": session_id, "seq": sequence,
        }, separators=(",", ":")).encode("utf-8")
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ack_socket:
                ack_socket.sendto(payload, (host, int(port)))
        except OSError as exc:
            self.show_error(f"射击事件已保存，但确认包发送失败：{exc}")

    def refresh_shot_events(self):
        """从本机 SQLite 刷新射击事件页；只列出已成功接收并提交的事件。"""
        if not hasattr(self, "shot_table"):
            return
        rows = self.track_db.execute(
            """SELECT received_at, device_id, shot_id, shot_count, uptime_ms,
                      timestamp_source, utc_date, utc_time_ms, gps_valid, latitude,
                      longitude, voice_p2p, accel_peak
               FROM shot_events ORDER BY received_at DESC, shot_count DESC"""
        ).fetchall()
        self.shot_table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            received_at, device_id, shot_id, shot_count, uptime_ms = row[:5]
            timestamp_source, utc_date, utc_time_ms = row[5:8]
            gps_valid, latitude, longitude, voice_p2p, accel_peak = row[8:]
            timestamp_packet = {
                "uptimeMs": uptime_ms,
                "timestampSource": timestamp_source,
                "utcDate": utc_date,
                "utcTimeMs": utc_time_ms,
            }
            values = [
                received_at,
                device_id,
                str(shot_id),
                str(shot_count),
                self._shot_timestamp(timestamp_packet),
                "有效" if gps_valid else "无效",
                self.number(latitude, 7) if gps_valid else "—",
                self.number(longitude, 7) if gps_valid else "—",
                str(voice_p2p),
                self.number(accel_peak),
            ]
            for column, value in enumerate(values):
                self.shot_table.setItem(row_index, column, QTableWidgetItem(str(value)))

    def export_shot_events(self):
        """将本机数据库中已接收的射击事件完整导出为 CSV。"""
        rows = self.track_db.execute(
            "SELECT * FROM shot_events ORDER BY received_at ASC, shot_count ASC"
        ).fetchall()
        if not rows:
            QMessageBox.information(self, "没有射击事件", "电脑数据库中还没有已接收的射击事件。")
            return
        columns = [item[1] for item in self.track_db.execute("PRAGMA table_info(shot_events)")]
        default_name = f"gun_shot_events_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _ = QFileDialog.getSaveFileName(
            self, "导出射击事件", default_name, "CSV 文件 (*.csv)"
        )
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as file:
                writer = csv.writer(file)
                writer.writerow(columns)
                writer.writerows(rows)
            self.connection_label.setText(
                f"已导出 {len(rows)} 条射击事件：{path}（电脑数据库中的记录仍保留）"
            )
        except OSError as exc:
            QMessageBox.critical(self, "导出失败", str(exc))

    @classmethod
    def _shot_timestamp(cls, packet):
        """GPS 已校时时显示本地时间；否则明确显示设备运行时间而不伪造绝对时间。"""
        if packet.get("utcDate"):
            local_time = cls._track_timestamp(
                int(packet.get("utcDate", 0)), int(packet.get("utcTimeMs", 0)),
                int(packet.get("uptimeMs", 0)),
            )
            if packet.get("timestampSource") == "gps_holdover" and not local_time.startswith("设备运行"):
                return local_time + "（由最近 GPS 校时推算）"
            return local_time
        return f"设备运行 {int(packet.get('uptimeMs', 0)) / 1000:.1f} 秒（未校时）"

    def refresh_display(self):
        """由 Qt 定时器批量刷新曲线，并提示数据流中断。"""
        if not self.samples:
            return
        rows = list(self.samples)
        xs = list(self.x_values)
        self.accel_curves[0].setData(xs, [self.as_float(r["vibration_x"]) for r in rows])
        self.accel_curves[1].setData(xs, [self.as_float(r["vibration_y"]) for r in rows])
        self.accel_curves[2].setData(xs, [self.as_float(r["vibration_z"]) for r in rows])
        if self.last_packet_time and time.time() - self.last_packet_time > 3:
            self.connection_label.setText("已超过 3 秒未收到数据")

    @staticmethod
    def as_float(value):
        """将曲线字段转换为浮点数，缺失值按零显示。"""
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def clear_history(self):
        """清空本地缓存、重置时间原点并清除所有曲线。"""
        self.samples.clear()
        self.x_values.clear()
        self.start_uptime = None
        for curve in self.accel_curves:
            curve.clear()

    def export_csv(self):
        """将已缓存的状态包样本导出为带 BOM 的 UTF-8 CSV 文件。"""
        if not self.samples:
            QMessageBox.information(self, "没有数据", "收到状态数据后才能导出 CSV。")
            return
        default_name = f"gun_telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _ = QFileDialog.getSaveFileName(self, "导出遥测数据", default_name, "CSV 文件 (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as file:
                writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
                writer.writeheader()
                writer.writerows(self.samples)
            self.connection_label.setText(f"已导出 {len(self.samples)} 条状态数据：{path}")
        except OSError as exc:
            QMessageBox.critical(self, "导出失败", str(exc))

    def show_error(self, message):
        """在窗口连接状态栏显示接收线程错误。"""
        self.connection_label.setText(message)

    def closeEvent(self, event):
        """关闭窗口前停止后台接收，避免线程遗留。"""
        self.receiver.stop()
        self.track_db.close()
        event.accept()


def main():
    """创建 Qt 应用并进入桌面事件循环。"""
    app = QApplication(sys.argv)
    app.setOrganizationName("GunDetection")
    app.setApplicationName("GunWifiMonitor")
    window = MonitorWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
