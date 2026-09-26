"""各ノードが `/diagnostics` へ自己申告診断を配信するための共通ヘルパ。

規約は `docs/ノード健全性監視設計.md` を正とする。
"""

from .diagnostic_core import (
    ASPECT_DEVICE,
    ASPECT_LIVENESS,
    ASPECT_QUALITY,
    ERROR,
    OK,
    STALE,
    WARN,
    DiagnosticNameError,
    build_array,
    build_status,
    compose_name,
    worst_level,
)
from .reporter import DIAGNOSTICS_TOPIC, DiagnosticReporter, report_alive

__all__ = [
    'ASPECT_DEVICE',
    'ASPECT_LIVENESS',
    'ASPECT_QUALITY',
    'DIAGNOSTICS_TOPIC',
    'ERROR',
    'OK',
    'STALE',
    'WARN',
    'DiagnosticNameError',
    'DiagnosticReporter',
    'build_array',
    'build_status',
    'compose_name',
    'report_alive',
    'worst_level',
]
