#!/usr/bin/env python3
"""Call a std_srvs/Trigger service without going through ros2 CLI."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


class TriggerServiceCaller(Node):
    def __init__(self, service_name: str) -> None:
        super().__init__('traymover_trigger_service_caller')
        self.client = self.create_client(Trigger, service_name)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('service_name')
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = TriggerServiceCaller(args.service_name)

    try:
        if not node.client.wait_for_service(timeout_sec=args.timeout):
            print(f'service_not_available: {args.service_name}')
            return 1

        future = node.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            print(f'service_call_timeout: {args.service_name}')
            return 1

        response = future.result()
        if response is None:
            print(f'service_call_failed: {args.service_name}')
            return 1

        print(f'success={response.success} message={response.message}')
        return 0 if response.success else 2
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
