"""
CSP (Constraint Satisfaction) katmanı:
- Her drone için toplu rota listesini alır
- Ağırlık, batarya ve zaman penceresi kısıtlarını doğrular
- Uygun değilse 'False', uygunsa 'True' + kalan batarya döndürür
"""
from typing import List, Tuple
from models import Drone, Delivery
from graph import haversine

# Basit enerji modeli: 1 Wh ≈ 15 m (örnek)
METRE_PER_WH = 40.0


def check_route(drone: Drone,
                route: List[Delivery],
                takeoff_time: int = 8 * 60) -> Tuple[bool, float]:
    """
    route: teslimat sırası (Delivery objeleri)
    takeoff_time: dakikada (örn. 8:00 ➔ 480)
    Dönüş: (uygun_mu, kalan_batarya_Wh)
    """
    battery = drone.battery_capacity
    current_pos = drone.start_pos
    current_time = takeoff_time

    for d in route:
        # 1) Ağırlık
        if d.weight > drone.max_weight:
            return False, battery

        # 2) Mesafe & enerji
        dist = haversine(current_pos, d.pos)      # metre
        energy_need = dist / METRE_PER_WH         # Wh
        if energy_need > battery:
            return False, battery
        battery -= energy_need
        # 3) Varış zamanı (dk)  — sabit hız
        current_time += dist / drone.speed / 60   # m/s -> dk
        start_win = int(d.time_window[0][:2]) * 60 + int(d.time_window[0][3:])
        end_win   = int(d.time_window[1][:2]) * 60 + int(d.time_window[1][3:])
        if not (start_win <= current_time <= end_win):
            return False, battery

        # Teslimat sonrası konumu güncelle
        current_pos = d.pos

    # Eve dönüş kontrolü (opsiyonel)
    dist_back = haversine(current_pos, drone.start_pos)
    energy_need = dist_back / METRE_PER_WH
    if energy_need > battery:
        return False, battery
    battery -= energy_need

    return True, battery
