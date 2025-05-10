from pathlib import Path
import json
from models import Drone, Delivery, NoFlyZone
from graph import build_graph
from astar import astar

# --- JSON'ları oku ---
def load_json(path, cls):
    with open(path, encoding="utf-8") as f:
        return [cls(**obj) for obj in json.load(f)]

drones     = load_json(Path("data/drones_s1.json"),     Drone)
deliveries = load_json(Path("data/deliveries_s1.json"), Delivery)
zones      = load_json(Path("data/nofly_s1.json"),      NoFlyZone)

g = build_graph(drones, deliveries, zones)

# İlk drone -> ilk teslimat yolunu bulalım
start = "drone_1"
goal  = "del_1"

cost, path = astar(g, start, goal)
print(f"En kısa yol maliyeti: {cost:.1f} m")
print("Yol:", " -> ".join(path))
