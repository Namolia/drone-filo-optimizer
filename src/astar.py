"""
Basit A* implementasyonu:
- Heuristik: düz çizgi (Haversine) mesafesi
- Kenar maliyeti: graph.edge[u][v]['cost']
"""
import math
import heapq
import networkx as nx


R_EARTH = 6371000


def haversine(a, b):
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(h))


def astar(g: nx.Graph, start: str, goal: str):
    """G: networkx graph, start & goal node id.  Dönüş: (toplam_maliyet, yol_listesi)."""
    open_set = [(0, start)]
    g_score = {start: 0}
    came_from = {}

    pos = nx.get_node_attributes(g, "pos")
    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # yol oluştur
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return g_score[goal], list(reversed(path))

        for neighbor in g.neighbors(current):
            tentative = g_score[current] + g[current][neighbor]["cost"]
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                f_score = tentative + haversine(pos[neighbor], pos[goal])
                heapq.heappush(open_set, (f_score, neighbor))

    raise ValueError("Goal not reachable")
