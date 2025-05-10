import random, copy, json
from pathlib import Path
from typing import List, Dict
from models import Drone, Delivery, NoFlyZone
from csp import check_route
from graph import build_graph

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
                pop.append(child)
            if gen % 10 == 0:
                print(f"Gen {gen:3d}  best={scored[0][0]:,}")
        return scored[0]  # (fitness, chrom)


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
