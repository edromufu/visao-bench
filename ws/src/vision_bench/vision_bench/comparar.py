#!/usr/bin/env python3
"""Tabela comparando as medições salvas pelo medidor (mediana por label).

  ros2 run vision_bench comparar                  # ~/visao-bench/resultados
  ros2 run vision_bench comparar /outra/pasta
"""
import csv
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

COLS = [('fps_in', 'fps in', '{:.1f}'), ('fps_out', 'fps out', '{:.1f}'), ('perdidos_pct', 'perd%', '{:.0f}'),
        ('lat_proc_p50_ms', 'lat p50', '{:.0f}'), ('lat_proc_p95_ms', 'lat p95', '{:.0f}'),
        ('lat_total_p50_ms', 'total p50', '{:.0f}'),
        ('cpu_visao_media_pct', 'CPU visão%', '{:.0f}'), ('cpu_resto_launch_media_pct', 'CPU resto%', '{:.0f}'),
        ('cpu_sistema_media_pct', 'CPU sist%', '{:.0f}'), ('ram_visao_max_mb', 'RAM MB', '{:.0f}'),
        ('temp_cpu_max_c', 'temp°C', '{:.0f}'), ('gpu_media_pct', 'GPU%', '{:.0f}')]


def main():
    res = Path(sys.argv[1] if len(sys.argv) > 1 else Path.home() / 'visao-bench' / 'resultados')
    runs = defaultdict(list)
    for f in sorted(res.glob('*.json')):
        r = json.loads(f.read_text())
        runs[r['label']].append(r)
    if not runs:
        print(f'nenhum resultado em {res}')
        return

    rows = []
    for label, rs in runs.items():
        row = {'branch': label, 'commit': rs[-1].get('commit', ''), 'n': len(rs)}
        for k, _, _ in COLS:
            v = [r[k] for r in rs if r.get(k) is not None]
            row[k] = statistics.median(v) if v else None
        rows.append(row)
    rows.sort(key=lambda r: -(r['fps_out'] or 0))

    hdr = ['branch', 'commit', 'n'] + [h for _, h, _ in COLS]
    table = [[r['branch'], r['commit'], str(r['n'])] +
             [('-' if r[k] is None else fmt.format(r[k])) for k, _, fmt in COLS] for r in rows]
    w = [max(len(x) for x in col) for col in zip(hdr, *table)]
    print('  '.join(h.ljust(n) for h, n in zip(hdr, w)))
    for t in table:
        print('  '.join(c.ljust(n) for c, n in zip(t, w)))
    print('\nlatências em ms; CPU% = soma dos núcleos (100 = 1 núcleo cheio); mediana de n medições.')

    with open(res / 'resumo.csv', 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)


if __name__ == '__main__':
    main()
