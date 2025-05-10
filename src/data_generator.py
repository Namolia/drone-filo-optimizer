import json
import random
from pathlib import Path
from models import Drone, Delivery, NoFlyZone

# --- Sabitler ---------------------------------------------------------------
RAND    = random.Random(42)
CENTER  = (40.765, 29.940)   # Kocaeli civarı örnek
SPAN    = 0.05               # ± derecelik yayılma
ROOT    = Path(__file__).resolve().parent.parent   # proje kökü
DATA_DIR = ROOT / "data"                          # <‑‑ tek meydana kayıt

# --- Yardımcılar ------------------------------------------------------------
def rand_coord():
    lat = CENTER[0] + RAND.uniform(-SPAN, SPAN)
    lon = CENTER[1] + RAND.uniform(-SPAN, SPAN)
    return (lat, lon)


def make_drones(n=5):
    return [
        Drone(
            id=i,
            max_weight=5.0,
            battery_capacity=1500.0,   # 1500 Wh
            speed=20.0,                # 20 m/s ≈ 72 km‑s
            start_pos=CENTER,
        )
        for i in range(1, n + 1)
    ]


def make_deliveries(n=20):
    return [
        Delivery(
            id=i,
            pos=rand_coord(),
            weight=RAND.uniform(0.2, 3.0),
            priority=RAND.randint(1, 3),
            time_window=("08:00", "20:00"),
        )
        for i in range(1, n + 1)
    ]


def make_nfz(n=2):
    zones = []
    for i in range(1, n + 1):
        center = rand_coord()
        size = 0.01
        zones.append(
            NoFlyZone(
                id=i,
                coordinates=[
                    (center[0] - size, center[1] - size),
                    (center[0] - size, center[1] + size),
                    (center[0] + size, center[1] + size),
                    (center[0] + size, center[1] - size),
                ],
                active_time=("00:00", "23:59"),
            )
        )
    return zones


def save_json(obj, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump([o.__dict__ for o in obj], f, indent=2)


# --- Ana --------------------------------------------------------------------
def main():
    drones     = make_drones()
    deliveries = make_deliveries()
    zones      = make_nfz()

    save_json(drones,     DATA_DIR / "drones_s1.json")
    save_json(deliveries, DATA_DIR / "deliveries_s1.json")
    save_json(zones,      DATA_DIR / "nofly_s1.json")
    print("✔  Örnek veri seti proje içindeki data/ klasörüne kaydedildi.")


if __name__ == "__main__":
    main()
