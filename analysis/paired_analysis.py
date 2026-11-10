#!/usr/bin/env python3
"""Paired, reproducible policy comparisons. Standard library only.
Usage: python analysis/paired_analysis.py results/run_*.csv
"""
import csv, glob, math, random, statistics, sys
from collections import defaultdict

FILES = [p for arg in sys.argv[1:] for p in glob.glob(arg)]
if not FILES:
    raise SystemExit('Usage: python analysis/paired_analysis.py results/rl_v10_run*.csv')
by_map = defaultdict(dict)
for path in FILES:
    with open(path, newline='', encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            key = (r.get('replicate', path), r['side'], r['density'], r['seed'])
            policy = r['policy']
            if policy in by_map[key]:
                raise ValueError(f'Duplicate map/policy: {key} {policy}')
            by_map[key][policy] = r

policies = ['q_learning','quality_proxy','jump_to_one','fixed_half','stop_first']
reachable = {k:v for k,v in by_map.items() if next(iter(v.values()))['reachable']=='1'}
print(f'Files: {len(FILES)}; maps: {len(by_map)}; reachable: {len(reachable)}')
print('Only reachable maps with all five policies are included in paired analysis.\n')
complete = {k:v for k,v in reachable.items() if all(p in v for p in policies)}
if len(complete) != len(reachable):
    print(f'WARNING: {len(reachable)-len(complete)} incomplete reachable maps')

def num(row, col):
    return float(row[col])

def median(vals):
    return statistics.median(vals)

print('POLICY SUMMARY (reachable maps; wall_ms includes decision overhead)')
print('policy,n,median_wall_ms,mean_error,median_expanded,median_estimated_bytes')
for p in policies:
    rows=[v[p] for v in complete.values()]
    print(f'{p},{len(rows)},{median([num(r,"wall_ms") for r in rows]):.6f},'
          f'{statistics.mean(num(r,"relative_error") for r in rows):.6%},'
          f'{median([num(r,"expanded") for r in rows]):.1f},'
          f'{median([num(r,"estimated_total_bytes") for r in rows]):.1f}')

# Paired differences: a negative difference favors RL. Bootstrap resamples maps,
# rather than individual policy records, to preserve pairing.
rng = random.Random(20261110)
print('\nPAIRED RL - BASELINE (negative wall/expansion delta favors RL)')
print('baseline,n,rl_faster,median_delta_ms,CI95_delta_ms,mean_delta_error_pp,median_delta_expanded')
for baseline in policies[1:]:
    pairs=[(v['q_learning'],v[baseline]) for v in complete.values()]
    dtime=[num(a,'wall_ms')-num(b,'wall_ms') for a,b in pairs]
    derr=[100*(num(a,'relative_error')-num(b,'relative_error')) for a,b in pairs]
    dexp=[num(a,'expanded')-num(b,'expanded') for a,b in pairs]
    n=len(pairs)
    if not n: continue
    boots=sorted(median([dtime[rng.randrange(n)] for _ in range(n)]) for _ in range(2000))
    print(f'{baseline},{n},{sum(d<0 for d in dtime)}/{n},'
          f'{median(dtime):.6f},[{boots[49]:.6f},{boots[1949]:.6f}],'
          f'{statistics.mean(derr):.6f},{median(dexp):.1f}')

# Pareto dominance per map for 3 objectives: time, error, estimated memory.
print('\nPARETO NON-DOMINATED COUNTS (wall_ms, relative_error, estimated_total_bytes)')
counts=defaultdict(int)
for rows in complete.values():
    vectors={p:tuple(num(rows[p],col) for col in ('wall_ms','relative_error','estimated_total_bytes')) for p in policies}
    for p,x in vectors.items():
        if not any(all(y[i]<=x[i] for i in range(3)) and any(y[i]<x[i] for i in range(3))
                   for q,y in vectors.items() if q!=p):
            counts[p]+=1
for p in policies: print(f'{p}: {counts[p]}/{len(complete)}')

print('\nPER SIZE (paired medians, ms)')
for size in sorted({int(k[1]) for k in complete}):
    rows=[v for k,v in complete.items() if int(k[1])==size]
    print(f'{size}: n={len(rows)} RL={median([num(v["q_learning"],"wall_ms") for v in rows]):.6f} '
          f'proxy={median([num(v["quality_proxy"],"wall_ms") for v in rows]):.6f}')
