"""
Safety module for autonomous driving system.

Provides independent safety override logic that must not be bypassed.
"""

from .safety_override import SafetyOverride

__all__ = ['SafetyOverride']

