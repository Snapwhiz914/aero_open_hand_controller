"""Aero Open Hand — top-level package.

Exports the abstract Hand base class for use as a type annotation.
Concrete implementations live in hand.ttl (TTL serial) and hand.sdk (ESP32).
"""

from .hand import Hand
from .macro import MacroRecorder, MacroPlayer, MacroMetadata, list_macros, delete_macro
