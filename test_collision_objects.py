#!/usr/bin/env python3
"""
Stage 2 test: MoveIt planning scene에 collision object 추가/제거.

테이블 평면 + 전방 장애물 박스 + 측면 장애물 박스로 cuMotion 장애물 회피 검증.
cuMotion action server와 move_group이 실행 중인 상태에서 사용.

Usage:
  python3 test_collision_objects.py          # 장애물 추가
  python3 test_collision_objects.py --clear  # 장애물 제거

장애물 배치 (world 프레임, 단위: m):
  table:          2x2x0.05m  at (0.0,  0.0, -0.025)  - 바닥 지면
  obstacle_front: 0.2x0.2x0.6m at (0.7,  0.0,  0.5)  - 전방 장애물 (UR10e reach 내)
  obstacle_right: 0.15x0.15x0.4m at (0.3, -0.6,  0.4) - 우측 장애물
"""

import argparse
import sys

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


class CollisionObjectPublisher(Node):
    def __init__(self):
        super().__init__('collision_object_publisher')
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info('Waiting for /apply_planning_scene service...')
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                'apply_planning_scene not available. Is move_group running?'
            )
        self.get_logger().info('Connected.')

    def _make_box(self, obj_id, x, y, z, sx, sy, sz, frame='world'):
        obj = CollisionObject()
        obj.header = Header(frame_id=frame, stamp=self.get_clock().now().to_msg())
        obj.id = obj_id
        obj.operation = CollisionObject.ADD
        obj.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[sx, sy, sz])
        )
        obj.primitive_poses.append(
            Pose(
                position=Point(x=x, y=y, z=z),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
        return obj

    def _apply_scene(self, objects):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.extend(objects)
        fut = self.client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result() is not None and fut.result().success

    def add_objects(self):
        objects = [
            # 바닥 테이블: 로봇 베이스 아래 지면 표현
            self._make_box('table',          0.0,  0.0, -0.025, 2.0,  2.0,  0.05),
            # 전방 장애물: UR10e reach 내 (약 0.7m 전방)
            self._make_box('obstacle_front', 0.7,  0.0,  0.5,   0.2,  0.2,  0.6),
            # 우측 장애물
            self._make_box('obstacle_right', 0.3, -0.6,  0.4,   0.15, 0.15, 0.4),
        ]
        if self._apply_scene(objects):
            self.get_logger().info(
                'Collision objects added successfully:\n'
                '  table:          2x2x0.05m  at (0.0,  0.0, -0.025)\n'
                '  obstacle_front: 0.2x0.2x0.6m at (0.7,  0.0,  0.5)\n'
                '  obstacle_right: 0.15x0.15x0.4m at (0.3, -0.6,  0.4)\n'
                '\n'
                'cuMotion이 자동으로 이 장애물들을 반영합니다.\n'
                'RViz MotionPlanning 패널에서 Plan 버튼으로 우회 경로를 확인하세요.'
            )
        else:
            self.get_logger().error('Failed to add collision objects!')

    def clear_objects(self):
        objects = []
        for obj_id in ['table', 'obstacle_front', 'obstacle_right']:
            obj = CollisionObject()
            obj.header = Header(frame_id='world', stamp=self.get_clock().now().to_msg())
            obj.id = obj_id
            obj.operation = CollisionObject.REMOVE
            objects.append(obj)
        if self._apply_scene(objects):
            self.get_logger().info('All collision objects cleared.')
        else:
            self.get_logger().error('Failed to clear collision objects!')


def main():
    parser = argparse.ArgumentParser(
        description='Stage 2 cuMotion obstacle avoidance test'
    )
    parser.add_argument(
        '--clear', action='store_true', help='장애물 제거 (기본: 추가)'
    )
    args = parser.parse_args()

    rclpy.init()
    try:
        node = CollisionObjectPublisher()
        node.clear_objects() if args.clear else node.add_objects()
        node.destroy_node()
    except RuntimeError as e:
        print(f'Error: {e}', file=sys.stderr)
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
