"""
Teslimat noktaları + drone başlangıç konumlarından tam bağlantılı
yönsüz bir graf üretir. Kenar ağırlığı = Haversine mesafe (metre).
NFZ içinde kalan kenarlar otomatik olarak atılır.
"""
from typing import List, Tuple
import math
import networkx as nx
from shapely.geometry import Polygon, LineString, Point
from models import Drone, Delivery, NoFlyZone


R_EARTH = 6371000  # metre

def haversine(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(h))


def build_nfz_polygons(zones: List[NoFlyZone]) -> List[Polygon]:
    return [Polygon(zone.coordinates) for zone in zones]


def intersects_nfz(p1: Tuple[float, float], p2: Tuple[float, float], polygons: List[Polygon]) -> bool:
    line = LineString([p1, p2])
    return any(line.intersects(poly) for poly in polygons)


def build_graph(drones: List[Drone], delivers: List[Delivery], zones: List[NoFlyZone]) -> nx.Graph:
    g = nx.Graph()
    polygons = build_nfz_polygons(zones)

    # Node ekle
    for d in drones:
        g.add_node(f"drone_{d.id}", pos=d.start_pos, kind="start")
    for dlv in delivers:
        g.add_node(f"del_{dlv.id}", pos=dlv.pos, kind="delivery", weight=dlv.weight)

    # Tüm çiftler arasında kenar (tam bağlantılı grafa yakın)
    nodes = list(g.nodes(data=True))
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            n1, dat1 = nodes[i]
            n2, dat2 = nodes[j]
            p1, p2 = dat1["pos"], dat2["pos"]

            if intersects_nfz(p1, p2, polygons):
                continue  # NFZ çizgisini kesiyor, kenarı atla

            dist = haversine(p1, p2)  # metre
            g.add_edge(n1, n2, distance=dist, cost=dist)  # cost şimdilik = mesafe

    return g
