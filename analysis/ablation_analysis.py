#!/usr/bin/env python3
"""Paired ablation comparisons: python analysis/ablation_analysis.py full.csv no_gap.csv no_marginal.csv"""
import csv, random, statistics, sys
from collections import defaultdict
if len(sys.argv)!=4: raise SystemExit(__doc__)
variants=['q_learning','q_learning_no_gap','q_learning_no_marginal']
rows=defaultdict(dict)
for path,variant in zip(sys.argv[1:],variants):
    with open(path,newline='',encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            if r['policy']!=variant: continue
            key=(r['replicate'],r['side'],r['density'],r['seed'])
            if key in rows and variant in rows[key]: raise ValueError('Duplicate map')
            rows[key][variant]=r
complete=[v for v in rows.values() if all(p in v for p in variants)]
reachable=[v for v in complete if all(v[p]['reachable']=='1' for p in variants)]
print(f'Matched maps: {len(complete)}; reachable: {len(reachable)}')
if not reachable: raise SystemExit('No matched reachable maps')
for p in variants:
    rs=[v[p] for v in reachable]
    print(f'{p}: median_wall_ms={statistics.median(float(r["wall_ms"]) for r in rs):.6f}; mean_error={statistics.mean(float(r["relative_error"]) for r in rs):.6%}; median_expanded={statistics.median(int(r["expanded"]) for r in rs)}')
rng=random.Random(20261110)
for p in variants[1:]:
    delta=[float(v['q_learning']['wall_ms'])-float(v[p]['wall_ms']) for v in reachable]
    n=len(delta)
    boots=sorted(statistics.median(delta[rng.randrange(n)] for _ in range(n)) for _ in range(2000))
    print(f'full minus {p}: faster={sum(d<0 for d in delta)}/{n}; median_delta_ms={statistics.median(delta):.6f}; bootstrap95=[{boots[49]:.6f},{boots[1949]:.6f}]')
print('Note: bootstrap resamples maps, not independent training replicates; three training seeds are insufficient for robust across-training uncertainty.')
