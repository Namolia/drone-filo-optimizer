# src/cluster.py
from sklearn.cluster import KMeans
import numpy as np
from typing import List

def kmeans_partition(deliveries, n_clusters, seed=42) -> List[List]:
    """deliveries listesini k coğrafi kümeye ayırır; her küme listesi teslimat nesnelerini içerir."""
    coords = np.array([dlv.pos for dlv in deliveries])
    km = KMeans(n_clusters=n_clusters, random_state=seed, n_init=10).fit(coords)
    labels = km.labels_
    clusters = [[] for _ in range(n_clusters)]
    for dlv, lab in zip(deliveries, labels):
        clusters[lab].append(dlv)
    return clusters
