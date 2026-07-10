#!/usr/bin/env python3
#==============================================================================
# grid_http_server.py
# 机器人端 HTTP 地图服务
#
# 接口:
#   GET /map.png   → 返回最新 OccupancyGrid 的 PNG 图片
#   GET /trigger   → 触发 /generate_grid 重新生成地图, 返回 JSON
#
# 手柄端只需发 HTTP 请求, 无需装 ROS
#==============================================================================

import io, json, socket, threading
import rospy
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    """多线程 HTTP 服务器, /trigger 不再阻塞其他请求"""
    daemon_threads = True
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerRequest
from PIL import Image, ImageDraw
import tf

latest_png = b''
latest_marker = None
tf_listener = None
selected_pub = None
marker_lock = threading.Lock()

def grid_to_image(msg):
    """OccupancyGrid → PNG bytes (叠加 yolo 检测标记点)"""
    w, h = msg.info.width, msg.info.height
    arr = np.array(msg.data, dtype=np.int8).reshape(h, w)

    # 底图: 未知→灰, 空闲→白, 占用→黑 (RGB 三通道便于叠加彩色标记)
    img = np.full((h, w, 3), 127, dtype=np.uint8)
    img[arr == 0]   = [255, 255, 255]
    img[arr == 100] = [0, 0, 0]
    pil_img = Image.fromarray(img, 'RGB')

    # --- 叠加 marker 投影点 ---
    with marker_lock:
        marker = latest_marker

    if marker is not None and tf_listener is not None and len(marker.points) > 0:
        try:
            grid_frame = msg.header.frame_id
            marker_frame = marker.header.frame_id

            # 获取从 marker 坐标系到 grid 坐标系的变换
            if grid_frame and marker_frame and grid_frame != marker_frame:
                (trans, rot) = tf_listener.lookupTransform(
                    grid_frame, marker_frame, rospy.Time(0))
            else:
                trans, rot = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)

            # 四元数 → 旋转矩阵
            R = tf.transformations.quaternion_matrix(rot)[:3, :3]

            draw = ImageDraw.Draw(pil_img)
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            res = msg.info.resolution
            circle_r = max(3, int(1.0 / res))  # 约 1m 半径映射到像素, 至少 3px

            for pt, color in zip(marker.points, marker.colors):
                # 原始坐标 (marker 自身坐标系, 即 camera_init)
                wx, wy = pt.x, pt.y

                # 坐标变换: marker_frame → grid_frame
                p_local = np.array([wx, wy, pt.z])
                p_grid = R.dot(p_local) + np.array(trans)

                # 世界坐标 → 像素坐标
                px = int((p_grid[0] - origin_x) / res)
                py = int((p_grid[1] - origin_y) / res)

                if 0 <= px < w and 0 <= py < h:
                    r = int(color.r * 255)
                    g = int(color.g * 255)
                    b = int(color.b * 255)
                    draw.ellipse(
                        [px - circle_r, py - circle_r,
                         px + circle_r, py + circle_r],
                        fill=(r, g, b),
                        outline=(r, g, b))
                    # 在旁边标注原始坐标
                    label = "(%.1f, %.1f)" % (wx, wy)
                    draw.text((px + circle_r + 2, py - 6), label, fill=(r, g, b))
        except Exception as e:
            rospy.logwarn_throttle(30, "Marker overlay failed: %s" % e)

    buf = io.BytesIO()
    pil_img.save(buf, format='PNG')
    return buf.getvalue()

def grid_callback(msg):
    global latest_png
    try:
        latest_png = grid_to_image(msg)
        rospy.loginfo_throttle(10, "Grid updated: %dx%d" % (msg.info.width, msg.info.height))
    except Exception as e:
        rospy.logerr("Convert failed: %s" % e)

def marker_callback(msg):
    """订阅 /yolo/detection_area, 缓存最新 marker"""
    global latest_marker
    with marker_lock:
        latest_marker = msg

class Handler(BaseHTTPRequestHandler):
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        if self.path == '/map.png':
            self._send_map()
        elif self.path == '/markers.json':
            self._send_markers()
        elif self.path == '/trigger':
            self._send_trigger()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/select':
            self._handle_select()
        else:
            self.send_response(404)
            self.end_headers()

    def _send_map(self):
        if latest_png:
            self._send_ok(latest_png, 'image/png')
        else:
            self.send_response(503)
            self.end_headers()

    def _send_markers(self):
        """GET /markers.json → 返回 /yolo/detection_area 中的原始标记点"""
        with marker_lock:
            marker = latest_marker

        if marker is None:
            body = json.dumps({'markers': []}).encode()
            self._send_ok(body, 'application/json')
            return

        points = []
        for pt, color in zip(marker.points, marker.colors):
            r = int(color.r * 255)
            g = int(color.g * 255)
            b = int(color.b * 255)
            if r > 200 and g < 100:
                label = 'defect'
            elif g > 150 and r < 100:
                label = 'person'
            else:
                label = 'both'
            points.append({
                'wx': round(pt.x, 2), 'wy': round(pt.y, 2),
                'r': r, 'g': g, 'b': b,
                'label': label,
            })

        body = json.dumps({'markers': points}).encode()
        self._send_ok(body, 'application/json')

    def _handle_select(self):
        """POST /select → 手柄端回传选中的点, 发布到 /selected_markers"""
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length))
            points_data = body.get('points', [])

            if not points_data:
                self._send_ok(json.dumps({'ok': True, 'count': 0}).encode(), 'application/json')
                return

            marker = Marker()
            marker.header.frame_id = "camera_init"
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.SPHERE_LIST
            marker.ns = "selected"
            marker.id = 0
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = marker.scale.z = 0.5

            label_colors = {
                'person': (0.2, 0.8, 0.4),
                'defect': (1.0, 0.5, 0.1),
                'both':   (0.8, 0.2, 0.8),
            }

            for p in points_data:
                pt = Point(x=p['wx'], y=p['wy'], z=0)
                marker.points.append(pt)
                lc = label_colors.get(p.get('label', ''), (1.0, 1.0, 0.0))
                marker.colors.append(ColorRGBA(r=lc[0], g=lc[1], b=lc[2], a=0.9))

            if selected_pub is not None:
                selected_pub.publish(marker)

            self._send_ok(
                json.dumps({'ok': True, 'count': len(points_data)}).encode(),
                'application/json')

        except Exception as e:
            rospy.logerr("POST /select failed: %s" % e)
            body = json.dumps({'ok': False, 'error': str(e)}).encode()
            self.send_response(400)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)

    def _send_trigger(self):
        try:
            rospy.wait_for_service('/generate_grid', timeout=3.0)
            result = rospy.ServiceProxy('/generate_grid', Trigger)(TriggerRequest())
            body = json.dumps({
                'success': result.success,
                'message': result.message
            }).encode()
            self._send_ok(body, 'application/json')
        except Exception as e:
            body = json.dumps({'success': False, 'message': str(e)}).encode()
            self.send_response(500)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)

    def _send_ok(self, data, mime):
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', len(data))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):
        pass

if __name__ == '__main__':
    rospy.init_node('grid_http_server')

    tf_listener = tf.TransformListener()
    rospy.sleep(0.5)  # 给 TF 缓冲一点时间

    selected_pub = rospy.Publisher('/selected_markers', Marker, queue_size=1)

    rospy.Subscriber('/global_map_grid', OccupancyGrid, grid_callback, queue_size=1)
    rospy.Subscriber('/yolo/detection_area', Marker, marker_callback, queue_size=1)
    rospy.loginfo("Subscribed to /yolo/detection_area for marker overlay")

    server = ThreadingHTTPServer(('0.0.0.0', 8765), Handler)
    server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rospy.loginfo("HTTP map server on http://0.0.0.0:8765")
    server.serve_forever()

