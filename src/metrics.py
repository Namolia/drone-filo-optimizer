# src/metrics.py
from typing import Dict, List, Tuple
from models import Drone, Delivery, NoFlyZone
from graph import haversine, intersects_nfz, build_nfz_polygons

def route_metrics(drone: Drone,
                  deliveries: List[Delivery],
                  route_ids: List[int],
                  zones: List[NoFlyZone]) -> Dict[str, float]:
    """Tek drone için mesafe (km), enerji (Wh), süre (dk), gecikme (dk), NFZ_ihlali(bool)."""
    id2del = {d.id: d for d in deliveries}
    polygons = build_nfz_polygons(zones)

    dist_m, late_min = 0.0, 0.0
    pos = drone.start_pos
    time = 8 * 60                       # dakikada
    NFZ_hit = False

    for rid in route_ids:
        nxt = id2del[rid]
        seg = haversine(pos, nxt.pos)
        if intersects_nfz(pos, nxt.pos, polygons):
            NFZ_hit = True
        dist_m += seg
        time   += seg / drone.speed / 60
        deadline = int(nxt.time_window[1][:2]) * 60 + int(nxt.time_window[1][3:])
        if time > deadline:
            late_min += time - deadline
        pos = nxt.pos

    # dönüş
    seg = haversine(pos, drone.start_pos)
    if intersects_nfz(pos, drone.start_pos, polygons):
        NFZ_hit = True
    dist_m += seg
    time   += seg / drone.speed / 60

    energy_wh = dist_m / 40.0                    # METRE_PER_WH = 40
    return dict(
        distance_km = dist_m / 1000,
        energy_wh   = energy_wh,
        total_min   = time - 8*60,
        late_min    = late_min,
        nfz_violate = NFZ_hit
    )
