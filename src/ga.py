import random, copy, json
from pathlib import Path
from typing import List, Dict
from models import Drone, Delivery, NoFlyZone
from csp import check_route
from graph import (
    build_graph,
    build_nfz_polygons,
    intersects_nfz,
    haversine,
)

# ----------------- Küresel sabitler (ödül / ceza) -----------------
DELIVERY_REWARD   = 1_000.0
ENERGY_WEIGHT     = 1.0
LATE_PENALTY_MIN  = 50.0
NFZ_PENALTY       = float("inf")          # NFZ → diskalifiye
METRE_PER_WH      = 40.0

# ----------------- GA Optimizer -----------------------------------
class GAOptimizer:
    def __init__(
        self,
        drones: List[Drone],
        deliveries: List[Delivery],
        graph,
        zones: List[NoFlyZone],
        pop_size: int = 60,
        elite_ratio: float = 0.2,
        mutation_rate: float = 0.2,
        generations: int = 150,
    ):
        self.drones = drones
        self.deliveries = deliveries
        self.graph = graph
        self.zones = zones
        self.pop_size = pop_size
        self.elite = max(2, int(pop_size * elite_ratio))
        self.mutation_rate = mutation_rate
        self.generations = generations
        self.rand = random.Random(42)

    # --------- kromozom: {drone_id: [del_id, …]} ----------
    def random_chromosome(self) -> Dict[int, List[int]]:
        chrom = {d.id: [] for d in self.drones}
        ids = [dlv.id for dlv in self.deliveries]
        self.rand.shuffle(ids)
        for i, did in enumerate(ids):
            chrom[self.drones[i % len(self.drones)].id].append(did)
        return chrom

    # --------- fitness ---------------------------------------------
    def fitness(self, chrom) -> float:
        score     = 0.0
        polygons  = build_nfz_polygons(self.zones)
        id2del    = {d.id: d for d in self.deliveries}

        for dr in self.drones:
            route = [id2del[r] for r in chrom[dr.id]]
            ok, bat_left = check_route(dr, route)
            if not ok:
                return -NFZ_PENALTY          # batarya / zaman ihlali

            score += len(route) * DELIVERY_REWARD
            score -= (dr.battery_capacity - bat_left) * ENERGY_WEIGHT

            pos, time_min, nfz_hit = dr.start_pos, 8*60, False
            for task in route:
                dist = haversine(pos, task.pos)
                time_min += dist / dr.speed / 60
                if intersects_nfz(pos, task.pos, polygons, step_m=500):
                    nfz_hit = True
                deadline = int(task.time_window[1][:2])*60 + int(task.time_window[1][3:])
                if time_min > deadline:
                    score -= (time_min - deadline) * LATE_PENALTY_MIN
                pos = task.pos

            dist_back = haversine(pos, dr.start_pos)
            if intersects_nfz(pos, dr.start_pos, polygons, step_m=500):
                nfz_hit = True
            score -= (dist_back / METRE_PER_WH) * ENERGY_WEIGHT

            if nfz_hit:
                return -NFZ_PENALTY          # NFZ kesen rota diskalifiye
        return score

    # --------- crossover ----------
    def crossover(self, p1, p2):
        child = copy.deepcopy(p1)
        for d in self.drones:
            if self.rand.random() < 0.5:
                child[d.id] = copy.deepcopy(p2[d.id])
        return child

    # --------- mutasyon ----------
    def mutate(self, chrom):
        for d in self.drones:
            if self.rand.random() < self.mutation_rate and len(chrom[d.id]) > 1:
                i, j = self.rand.sample(range(len(chrom[d.id])), 2)
                chrom[d.id][i], chrom[d.id][j] = chrom[d.id][j], chrom[d.id][i]

    # --------- repair ----------
    def repair(self, chrom):
        all_ids = {dlv.id for dlv in self.deliveries}
        used, duplicates = set(), []
        for lst in chrom.values():
            for rid in lst:
                (duplicates if rid in used else used).append(rid) if rid in used else used.add(rid)
        missing = list(all_ids - used)
        self.rand.shuffle(missing)
        for lst in chrom.values():
            for i, rid in enumerate(lst):
                if rid in duplicates:
                    lst[i] = missing.pop() if missing else rid
                    duplicates.remove(rid)



    # --------- NFZ düzeltme ----------
    def fix_nfz(self, chrom):
        """Her drone rotasını NFZ'den çıkana kadar karıştırır; 30 denemeden sonra vazgeçer."""
        polygons = build_nfz_polygons(self.zones)
        id2del   = {d.id: d for d in self.deliveries}
        MAX_TRIES = 30          # sonsuz döngüyü engelle

        for dr in self.drones:
            lst = chrom[dr.id]
            tries = 0
            while tries < MAX_TRIES:
                hit = False
                prev = dr.start_pos
                # rota içi kenarlar
                for rid in lst:
                    nxt = id2del[rid].pos
                    if intersects_nfz(prev, nxt, polygons, step_m=1000):
                        hit = True
                        break
                    prev = nxt
                # dönüş kenarı
                if not hit and intersects_nfz(prev, dr.start_pos, polygons, step_m=1000):
                    hit = True

                if not hit:
                    break          # NFZ'yi hiç kesmiyor → rota kabul
                # kesişiyorsa karıştır ve tekrar dene
                self.rand.shuffle(lst)
                tries += 1


    # --------- ana döngü ----------
    def run(self):
        pop = [self.random_chromosome() for _ in range(self.pop_size)]
        for gen in range(self.generations):
            print("debug → evaluating generation", gen)
            scored = sorted(((self.fitness(c), c) for c in pop), key=lambda x: x[0], reverse=True)
            best_score = scored[0][0]
            pop = [c for s, c in scored[: self.elite] if s > -NFZ_PENALTY]
            while len(pop) < self.pop_size:
                p1, p2 = self.rand.sample(scored[: self.elite], 2)
                child = self.crossover(p1[1], p2[1])
                self.mutate(child)
                self.repair(child)
                self.fix_nfz(child)
                pop.append(child)
            if gen % 10 == 0:
                print(f"Gen {gen:3d}  best={best_score:,.0f}")
        return scored[0]  # (fitness, chrom)


# ------------------- Hızlı test -----------------------------------
if __name__ == "__main__":
    def load(p, cls):
        with open(p, encoding="utf-8") as f:
            return [cls(**o) for o in json.load(f)]

    drones     = load(Path("data/drones_s1.json"),     Drone)
    deliveries = load(Path("data/deliveries_s1.json"), Delivery)
    zones      = load(Path("data/nofly_s1.json"),      NoFlyZone)

    g = build_graph(drones, deliveries, zones)
    ga = GAOptimizer(drones, deliveries, g, zones)
    fit, best = ga.run()
    print("\nEn iyi fitness:", fit)
    for did, lst in best.items():
        print(f"Drone {did}: {lst}")
