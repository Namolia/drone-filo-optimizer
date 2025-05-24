import json, folium, random
from pathlib import Path
from models import Drone, Delivery, NoFlyZone
from graph import haversine, build_graph
from ga import GAOptimizer
from cluster import kmeans_partition

COLORS = ["blue", "green", "purple", "orange", "darkred",
          "cadetblue", "darkgreen", "pink", "gray", "black"]

def load(p, cls):
    with open(p, encoding="utf-8") as f:
        return [cls(**o) for o in json.load(f)]

def main():
    drones     = load(Path("data/drones_s1.json"),     Drone)
    deliveries = load(Path("data/deliveries_s1.json"), Delivery)
    zones      = load(Path("data/nofly_s1.json"),      NoFlyZone)

    # --- K‑Means kümeleri ---
    clusters = kmeans_partition(deliveries, len(drones))
    init_chrom = {dr.id: [dlv.id for dlv in cl]
                  for dr, cl in zip(drones, clusters)}

    g = build_graph(drones, deliveries, zones)

    ga = GAOptimizer(
            drones, deliveries, g, zones,
            pop_size=15,
            generations=30,
            mutation_rate=0.2,
            init_chrom=init_chrom)

    _, best = ga.run()      # best = chrom dict

    # --- Harita ---
    m = folium.Map(location=drones[0].start_pos, zoom_start=12, tiles="OpenStreetMap")

    # NFZ
    for z in zones:
        folium.Polygon(z.coordinates, color="red", fill=True, fill_opacity=0.3).add_to(m)

    # Teslimatlar
    for d in deliveries:
        folium.CircleMarker(d.pos, radius=5, color="orange",
                            popup=f"Del-{d.id} ({d.weight:.1f} kg)").add_to(m)

    # Rotalar
    for idx, dr in enumerate(drones):
        color = COLORS[idx % len(COLORS)]
        folium.Marker(dr.start_pos,
                      icon=folium.Icon(color="blue", icon="home"),
                      popup=f"Drone {dr.id}").add_to(m)

        coords = [dr.start_pos] + \
                 [next(dl.pos for dl in deliveries if dl.id == rid) for rid in best[dr.id]] + \
                 [dr.start_pos]
        folium.PolyLine(coords, color=color, weight=3, opacity=0.8,
                        popup=f"Drone {dr.id} rotası").add_to(m)

    m.save("map.html")
    print("✔  Harita map.html dosyasına kaydedildi. Tarayıcıda açabilirsiniz.")

if __name__ == "__main__":
    main()
