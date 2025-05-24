"""
Tam bağlantılı yönsüz grafik:
• Düğümler   : drone başlangıç konumları + teslimat noktaları
• Kenar ağırlığı : Haversine mesafe (metre)
• NFZ (No‑Fly Zone) poligonlarını kesen kenarlar otomatik atılır
Hız optimizasyonu:
  –  Bounding‑box ön filtresi
  –  Shapely LineString.intersects   (C seviyesinde hızlı)
  –  Gerekirse seyrek örnekleme (step_m = 2 000 m)
"""
from typing import List, Tuple
import math
import networkx as nx
from shapely.geometry import Polygon, LineString, Point, box
from functools import lru_cache
from models import Drone, Delivery, NoFlyZone

# ------------------------------------------------------------------ #
#  Temel yardımcılar                                                 #
# ------------------------------------------------------------------ #
R_EARTH = 6_371_000  # metre — dünya yarıçapı

def haversine(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """İki WGS‑84 noktası arasında Haversine mesafesi (metre)."""
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat  = lat2 - lat1
    dlon  = lon2 - lon1
    h     = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 2 * R_EARTH * math.asin(math.sqrt(h))


@lru_cache(maxsize=None)
def _cached_point(lat: float, lon: float) -> Point:
    """Shapely Point nesnesini koordinatlara göre önbelleğe alarak
       binlerce gereksiz nesne yaratımını engeller."""
    return Point(lat, lon)


# ------------------------------------------------------------------ #
#  NFZ yardımcıları                                                  #
# ------------------------------------------------------------------ #
def build_nfz_polygons(zones: List[NoFlyZone]) -> Tuple[Polygon, ...]:
    """NoFlyZone listesini Shapely Polygon tuple'ına dönüştür."""
    return tuple(Polygon(z.coordinates) for z in zones)


def _segment_samples(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    step_m: float = 2000.0
) -> Tuple[Point, ...]:
    """p1‑p2 kenarını ~step_m aralıklarla örnekleyip Point tuple'ı döndürür."""
    total = haversine(p1, p2)
    if total < step_m:
        return ()
    n   = int(total // step_m)
    lat1, lon1 = p1
    lat2, lon2 = p2
    dlat = (lat2 - lat1) / n
    dlon = (lon2 - lon1) / n
    # ±6 ondalık ≈ 0.11 m hassasiyet
    return tuple(_cached_point(round(lat1 + i*dlat, 6), round(lon1 + i*dlon, 6))
                 for i in range(1, n))


def intersects_nfz(p1, p2, polygons, step_m=500):
    line = LineString([p1, p2])
    lb   = box(*line.bounds)

    cand = [poly for poly in polygons if lb.intersects(poly)]
    if not cand:
        return False

    # Hızlı: tam kesişim varsa zaten NFZ'ye çarpıyor → True dön
    if any(line.intersects(poly) for poly in cand):
        return True          # <‑‑ ayrıntılı taramaya gerek yok

    # Buraya gelirse çizgi NFZ'yi "dokunmadan" geçiyor olabilir.
    # Yine de çok yakın kenarlar için seyrek örneklemeyle kontrol:
    for pt in _segment_samples(p1, p2, step_m):   # step=500m
        if any(pt.within(poly) for poly in cand):
            return True
    return False



# ------------------------------------------------------------------ #
#  Grafik inşası                                                      #
# ------------------------------------------------------------------ #
def build_graph(
    drones: List[Drone],
    deliveries: List[Delivery],
    zones: List[NoFlyZone],
) -> nx.Graph:
    """NFZ kesişen kenarları atlayarak tam bağlantılı grafik üretir."""
    g = nx.Graph()
    polygons = build_nfz_polygons(zones)

    # Düğümler
    for d in drones:
        g.add_node(f"drone_{d.id}", pos=d.start_pos, kind="start")
    for dlv in deliveries:
        g.add_node(
            f"del_{dlv.id}", pos=dlv.pos, kind="delivery", weight=dlv.weight
        )

    # Kenarlar
    nodes = list(g.nodes(data=True))
    for i in range(len(nodes)):
        n1, dat1 = nodes[i]
        p1 = dat1["pos"]
        for j in range(i + 1, len(nodes)):
            n2, dat2 = nodes[j]
            p2 = dat2["pos"]
            if intersects_nfz(p1, p2, polygons, step_m=2000.0):
                continue  # NFZ ihlali → kenarı atla
            dist = haversine(p1, p2)
            g.add_edge(n1, n2, distance=dist, cost=dist)

    return g
