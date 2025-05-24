# src/report_metrics.py
import json, pandas as pd
from pathlib import Path
from models import Drone, Delivery, NoFlyZone
from ga import GAOptimizer
from graph import build_graph
from metrics import route_metrics

def load(p, cls):
    with open(p, encoding="utf-8") as f:
        return [cls(**o) for o in json.load(f)]

drones     = load(Path("data/drones_s1.json"),     Drone)
deliveries = load(Path("data/deliveries_s1.json"), Delivery)
zones      = load(Path("data/nofly_s1.json"),      NoFlyZone)
g = build_graph(drones, deliveries, zones)

ga = GAOptimizer(
        drones, deliveries, g, zones,
        pop_size=5,
        mutation_rate=0.2,
        generations=5)

_, best = ga.run()

rows = []
for dr in drones:
    m = route_metrics(dr, deliveries, best[dr.id], zones)
    rows.append(dict(drone=dr.id, **m))

df = pd.DataFrame(rows)
print(df.to_string(index=False, float_format="%.2f"))

print("\n⬤  NFZ ihlali var mı? :", df.nfz_violate.any())
