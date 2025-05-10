import random, copy, json
from pathlib import Path
from typing import List, Dict
from models import Drone, Delivery, NoFlyZone
from csp import check_route
from graph import build_graph

ENERGY_WEIGHT     = 1.0      # Wh başına ceza
LATE_PENALTY_MIN  = 50.0     # Dakika başına ceza
DELIVERY_REWARD   = 1_000.0  # Her başarılı teslimat için ödül

# ---------- GA Optimizer ----------
class GAOptimizer:
    def __init__(self,
                 drones: List[Drone],
                 deliveries: List[Delivery],
                 graph,
                 pop_size=30,
                 elite_ratio=0.2,
                 mutation_rate=0.1,
                 generations=100):
        self.drones = drones
        self.deliveries = deliveries
        self.graph = graph
        self.pop_size = pop_size
        self.elite = int(pop_size * elite_ratio)
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

    # --------- fitness ----------
    def fitness(self, chrom) -> float:
        score = 0.0
        for d in self.drones:
            route_ids = chrom[d.id]
            route = [next(x for x in self.deliveries if x.id == rid) for rid in route_ids]
            ok, bat_left = check_route(d, route)
            if not ok:
                score -= 1_000_000
                continue
            score += len(route) * 1_000    # teslimat sayısı
            score += bat_left              # kalan batarya
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

    # --------- ana döngü ----------
    def run(self):
        pop = [self.random_chromosome() for _ in range(self.pop_size)]
        for gen in range(self.generations):
            scored = sorted(((self.fitness(c), c) for c in pop),
                key=lambda x: x[0],   # 0 = fitness
                reverse=True)
            pop = [c for _, c in scored[:self.elite]]  # elitler
            while len(pop) < self.pop_size:
                p1, p2 = self.rand.sample(scored[: self.elite], 2)
                child = self.crossover(p1[1], p2[1])
                self.mutate(child)
                self.repair(child) 
                pop.append(child)
            if gen % 10 == 0:
                print(f"Gen {gen:3d}  best={scored[0][0]:,}")
        return scored[0]  # (fitness, chrom)

# ---------- repair ----------
    def repair(self, chrom):
        """Her teslimat tam bir kez atanmış mı?  Yineleri eksiklerle değiştirir."""
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



    # --- fitness -----------------------------------------------------------------
def fitness(self, chrom) -> float:
    score = 0.0
    for d in self.drones:
        route_ids = chrom[d.id]
        route = [next(x for x in self.deliveries if x.id == rid) for rid in route_ids]

        ok, bat_left = check_route(d, route)
        if not ok:
            score -= 1_000_000        # ağır ceza, rota geçersiz
            continue

        # 1) Teslimat sayısı ödülü
        score += len(route) * DELIVERY_REWARD

        # 2) Enerji tüketimi cezası (başlangıç - kalan)
        energy_used = d.battery_capacity - bat_left
        score -= energy_used * ENERGY_WEIGHT

        # 3) Geç kalma cezası  (basit: varış zamanı - deadline)
        current_pos = d.start_pos
        current_time = 8 * 60        # 08:00 dakikada
        late_penalty = 0

        for task in route:
            # mesafe → süre (dk)
            dist = haversine(current_pos, task.pos)
            current_time += dist / d.speed / 60
            deadline = int(task.time_window[1][:2]) * 60 + int(task.time_window[1][3:])
            if current_time > deadline:
                late_penalty += (current_time - deadline) * LATE_PENALTY_MIN
            current_pos = task.pos

        score -= late_penalty
    return score

# ------------- hızlı test -------------
if __name__ == "__main__":
    def load(p, cls):
        with open(p, encoding="utf-8") as f:
            return [cls(**o) for o in json.load(f)]

    drones     = load(Path("data/drones_s1.json"),     Drone)
    deliveries = load(Path("data/deliveries_s1.json"), Delivery)
    zones      = load(Path("data/nofly_s1.json"),      NoFlyZone)

    g = build_graph(drones, deliveries, zones)

    ga = GAOptimizer(drones, deliveries, g, generations=50)
    (fit, best) = ga.run()
    print("\nEn iyi fitness:", fit)
    for did, lst in best.items():
        print(f"Drone {did}: {lst}")
