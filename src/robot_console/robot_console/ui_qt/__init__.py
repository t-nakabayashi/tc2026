"""robot_console のPyQt5ローカルUI。

MainWindowと4タブ（ダッシュボード、自己位置・センサ情報、起動・設定、
コンソールログ）で構成する。QWidgetはROS 2 publisher/subscriberを直接
持たず、`robot_console.core` が提供するSnapshotのみを参照する
（robot_console_gui_architecture_design.md 14.2節）。
"""
