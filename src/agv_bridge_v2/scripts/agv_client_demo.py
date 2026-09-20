#!/usr/bin/env python3
"""
上位机对接 rosbridge 的参考实现（裸 JSON 协议，不依赖 roslibpy）

用途：作为 SpringBoot 侧实现 rosbridge 客户端的对照物。
      这里出现的每一个 JSON 都可以 1:1 翻译成 Java 的 Map/Jackson 对象。

依赖：pip install websocket-client

用法：
    python3 agv_client_demo.py                     # 只订阅状态与反馈
    python3 agv_client_demo.py --curve             # 额外下发一条贝塞尔曲线移动
    python3 agv_client_demo.py --curve --cancel    # 下发后 5s 取消
"""

import argparse
import json
import sys
import threading
import time

try:
    from websocket import WebSocketApp
except ImportError:
    sys.exit("请先安装依赖: pip install websocket-client")

FOLLOW_EDGE = "agv_bridge_v2_interfaces/action/FollowEdge"


class RosbridgeClient:
    """极简 rosbridge 客户端：只覆盖本项目用到的 5 个 op。"""

    def __init__(self, host: str, port: int):
        self.url = f"ws://{host}:{port}"
        self.ws = WebSocketApp(
            self.url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=lambda _, e: print(f"[WS ERROR] {e}"),
            on_close=lambda *_: print("[WS CLOSED]"),
        )
        self._ready = threading.Event()
        self._seq = 0

    # ---------------- 生命周期 ----------------

    def run_forever(self, on_ready):
        self._on_ready = on_ready
        self.ws.run_forever(ping_interval=5, ping_timeout=15)

    def _on_open(self, _):
        print(f"[WS OPEN] {self.url}")
        self._ready.set()
        self._on_ready(self)

    def _on_message(self, _, raw):
        # 生产环境注意：大消息会被分片（op == "fragment"），需要按 id 重组。
        # 本项目订阅的 /agv/status、action_feedback 都很小，不会触发分片。
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            print(f"[RAW] {raw}")
            return

        op = msg.get("op")
        if op in ("publish",):
            print(f"[{msg.get('topic')}] {json.dumps(msg.get('msg'), ensure_ascii=False)}")
        elif op == "action_feedback":
            print(f"[FEEDBACK {msg.get('id')}] {json.dumps(msg.get('values'), ensure_ascii=False)}")
        elif op == "action_result":
            print(f"[RESULT {msg.get('id')}] status={msg.get('status')} "
                  f"result={msg.get('result')} values={json.dumps(msg.get('values'), ensure_ascii=False)}")
        elif op == "service_response":
            print(f"[SERVICE {msg.get('service')}] result={msg.get('result')} "
                  f"values={json.dumps(msg.get('values'), ensure_ascii=False)}")
        else:
            print(f"[{op}] {json.dumps(msg, ensure_ascii=False)}")

    def _send(self, payload: dict, mid: str = None):
        if mid is None:
            self._seq += 1
            mid = f"c{self._seq}"
        payload["id"] = mid
        self.ws.send(json.dumps(payload))
        return mid

    # ---------------- 协议 op ----------------

    def subscribe(self, topic: str, mid: str, throttle_rate: int = 0, compression: str = None):
        """上报：订阅一个 topic。throttle_rate 单位 ms。"""
        payload = {"op": "subscribe", "topic": topic, "throttle_rate": throttle_rate}
        if compression:
            payload["compression"] = compression
        return self._send(payload, mid)

    def unsubscribe(self, topic: str, mid: str):
        return self._send({"op": "unsubscribe", "topic": topic}, mid)

    def publish(self, topic: str, msg: dict):
        """下发：发布一个 topic（/cmd_vel、/initialpose）。"""
        return self._send({"op": "publish", "topic": topic, "msg": msg})

    def call_service(self, service: str, args: dict, mid: str, timeout: float = 5.0):
        return self._send(
            {"op": "call_service", "service": service, "args": args, "timeout": timeout}, mid
        )

    def send_action_goal(self, action: str, action_type: str, args: dict, mid: str):
        """下发移动指令：直线 / 贝塞尔曲线 / 倒车都走这一个 action。"""
        return self._send(
            {
                "op": "send_action_goal",
                "action": action,
                "action_type": action_type,
                "args": args,
                "feedback": True,
            },
            mid,
        )

    def cancel_action_goal(self, action: str, mid: str):
        """停止移动：mid 必须与 send_action_goal 时完全一致。"""
        return self._send({"op": "cancel_action_goal", "action": action}, mid)

    def message_details(self, iface_type: str, mid: str):
        """下载消息定义（自动生成 DTO 的依据）。"""
        return self.call_service("/rosapi/message_details", {"type": iface_type}, mid)


# --------------------------------------------------------------------------

def straight_goal() -> dict:
    return {
        "command_id": "move_demo_straight",
        "node_id": "I006",
        "x": -8.30,
        "y": -1.35,
        "theta": 0.1788,
        "edge_id": "E066",
        "edge_type": "STRAIGHT",
        "source_id": "I006",
        "target_id": "S008",
        "max_speed": 0.6,
        "back_up": False,
        "reverse": False,
        "step": 0.1,
        "end_point": False,
    }


def bezier_goal() -> dict:
    """二阶/三阶贝塞尔：control_points 只允许 1 个（二阶）或 2 个（三阶）。"""
    return {
        "command_id": "move_demo_curve",
        "node_id": "S008",
        "x": -5.30,
        "y": -0.40,
        "theta": 1.02,
        "edge_id": "E101",
        "edge_type": "CURVE",
        "source_id": "I006",
        "target_id": "S008",
        "max_speed": 0.4,
        "back_up": False,
        "reverse": False,
        "step": 0.05,
        "end_point": True,
        "control_points": [
            {"x": -7.10, "y": -2.00},
            {"x": -6.05, "y": -0.85},
        ],
    }


def on_ready(client: RosbridgeClient, args):
    print("[READY] 开始订阅状态上报")
    client.subscribe("/agv/status", "status", throttle_rate=1000)
    client.message_details(FOLLOW_EDGE + "_Goal", "schema")

    if not args.curve:
        return

    time.sleep(1.0)

    print("[SEND] 下发贝塞尔曲线移动指令")
    client.send_action_goal("/agv/follow_edge", FOLLOW_EDGE, bezier_goal(), "move_curve")

    if args.cancel:
        threading.Timer(5.0, lambda: (
            print("[SEND] 取消移动"),
            client.cancel_action_goal("/agv/follow_edge", "move_curve"),
        )).start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--curve", action="store_true", help="额外下发一条贝塞尔曲线指令")
    parser.add_argument("--cancel", action="store_true", help="下发 5s 后取消")
    parser.add_argument("--set-control", choices=["start", "stop", "reset"],
                        help="调用 /agv/set_control 后退出")
    parser.add_argument("--cmd-vel", type=float, metavar="VX",
                        help="发布一次 /cmd_vel 后退出")
    args = parser.parse_args()

    client = RosbridgeClient(args.host, args.port)

    if args.set_control or args.cmd_vel is not None:
        def one_shot(c: RosbridgeClient):
            if args.cmd_vel is not None:
                c.publish("/cmd_vel", {
                    "linear": {"x": args.cmd_vel, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                })
            if args.set_control:
                c.call_service("/agv/set_control", {"action": args.set_control}, "ctl")
            time.sleep(2.0)
            c.ws.close()

        client.run_forever(one_shot)
        return

    client.run_forever(lambda c: on_ready(c, args))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[EXIT]")
