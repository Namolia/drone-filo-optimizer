from dataclasses import dataclass
from typing import Tuple, List


@dataclass
class Drone:
    """Teslimat dronu temel modeli."""
    id: int
    max_weight: float           # kg
    battery_capacity: float     # Wh
    speed: float                # m/s
    start_pos: Tuple[float, float]  # (lat, lon)


@dataclass
class Delivery:
    """Tek bir teslimat görevi."""
    id: int
    pos: Tuple[float, float]
    weight: float               # kg
    priority: int               # 1 = yüksek, 3 = düşük
    time_window: Tuple[str, str]  # ("HH:MM", "HH:MM")


@dataclass
class NoFlyZone:
    """Uçuşa yasak bölge (çokgen)."""
    id: int
    coordinates: List[Tuple[float, float]]  # [(lat, lon), …]
    active_time: Tuple[str, str]            # ("HH:MM", "HH:MM")
