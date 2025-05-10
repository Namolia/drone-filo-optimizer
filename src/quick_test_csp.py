# src/quick_test_csp.py
import json
from pathlib import Path
from models import Drone, Delivery, NoFlyZone
from graph import build_graph
from astar import astar
from csp import check_route


def load_json(path, cls):
    with open(path, encoding="utf-8") as f:
        return [cls(**o) for o in json.load(f)]


drones     = load_json(Path("data/drones_s1.json"),     Drone)
deliveries = load_json(Path("data/deliveries_s1.json"), Delivery)
zones      = load_json(Path("data/nofly_s1.json"),      NoFlyZone)

g = build_graph(drones, deliveries, zones)

# Drone‑1 için ilk 3 teslimatı sırayla deneyelim
route_deliveries = deliveries[:3]
nodes = ["drone_1"] + [f"del_{d.id}" for d in route_deliveries]

# A* ile ardışık yolları birleştir
full_path = []
total_cost = 0
for a, b in zip(nodes, nodes[1:]):
    cost, p = astar(g, a, b)
    total_cost += cost
    full_path += p[:-1]      # son düğüm sonraki parçanın ilkidir
full_path.append(nodes[-1])

ok, bat_left = check_route(drones[0], route_deliveries)
print("Uygun mu:", ok, "• Kalan batarya:", round(bat_left, 1), "Wh")
