#!/usr/bin/env python3
"""Option 19: press Enter once, bypass one obstacle, and return to the start line."""
import select
import signal
import sys
import time


class SessionStopped(Exception):
    pass


class SingleButtonSession:
    def __init__(self, node, spin, read_line=None, clock=time.monotonic):
        self.node = node
        self.spin = spin
        self.read_line = read_line
        self.clock = clock
        self.last_state = None

    def pump(self):
        self.spin(self.node, timeout_sec=0.05)
        state = self.node.core.state
        if state != self.last_state:
            print(f'[Option 19] {state}: {self.node.core.reason}', flush=True)
            self.last_state = state

    def input_ready(self):
        if self.read_line is not None:
            return True
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        return bool(ready)

    def read(self):
        answer = self.read_line() if self.read_line is not None else sys.stdin.readline()
        if answer is None or answer == '':
            raise SessionStopped('input closed')
        return answer.rstrip('\r\n')

    def wait_for_button(self):
        print('\n选项 19：单键直行绕障测试', flush=True)
        print('按 Enter 后底盘沿当前直线前进；请站在前方约 1 米处。', flush=True)
        print('检测到障碍后将自动绕过，并用轮式里程计回到原直线路径。', flush=True)
        print('运行中输入 ! 并回车可触发手动急停，Ctrl+C 可停止。', flush=True)
        print('\n准备好后按 Enter：', flush=True)
        while True:
            self.pump()
            if not self.input_ready():
                continue
            answer = self.read()
            if answer == '':
                return
            if answer in ('!', 'q', 'Q'):
                if answer == '!':
                    self.node.assert_manual_estop()
                raise SessionStopped('operator stop before start')
            print('输入无效；只需按 Enter。', flush=True)

    def run_to_completion(self):
        while self.node.core.state not in ('COMPLETE', 'FAULT_HOLD', 'ABORTED'):
            self.pump()
            if self.read_line is None and self.input_ready():
                answer = self.read()
                if answer in ('!', 'q', 'Q'):
                    if answer == '!':
                        self.node.assert_manual_estop()
                    raise SessionStopped('operator stop during motion')
        if self.node.core.state != 'COMPLETE':
            raise SessionStopped(self.node.core.reason)
        print('绕障完成：已通过里程计确认回到原直线路径，底盘已停止。', flush=True)

    def run(self):
        self.wait_for_button()
        self.node.start_motion()
        print('已启动：收到前方深度和里程计数据后直接向前行驶。', flush=True)
        self.run_to_completion()


def main(args=None):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from guided_detour_node import LineReturnDetourNode

    node = None
    previous_signals = {}

    def interrupted(signum, frame):
        raise SessionStopped(signal.Signals(signum).name)

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_signals[signum] = signal.signal(signum, interrupted)
        node = LineReturnDetourNode()
        SingleButtonSession(node, rclpy.spin_once).run()
        return 0
    except (SessionStopped, KeyboardInterrupt, EOFError) as error:
        print('选项 19 已停止：' + (str(error) or type(error).__name__), flush=True)
        return 1
    finally:
        for signum in previous_signals:
            signal.signal(signum, signal.SIG_IGN)
        if node is not None:
            node.close_zero()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_signals.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    sys.exit(main())
