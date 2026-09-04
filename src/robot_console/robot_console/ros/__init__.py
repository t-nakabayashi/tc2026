"""robot_console のROS 2結合層。

`console_node.py` は ROS 2 topic購読・publish のみを担当し、状態集約・
Snapshot生成のロジックは持たない（`core/console_core.py` へ委譲する）。
"""
