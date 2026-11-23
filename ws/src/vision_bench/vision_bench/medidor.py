#!/usr/bin/env python3
"""Mede a visão em tempo real enquanto o launch de uma branch roda em outro terminal.

A cada segundo mostra fps de entrada e de saída, latência, CPU/RAM do nó de visão e do resto do
launch, CPU/RAM/temperatura/frequência da máquina e GPU (quando der para ler).
No fim (Ctrl+C ou `duracao`) salva <saida>/<label>_<data>.json (resumo) e .csv (série por segundo).

Latências (casando a imagem e a saída pelo header.stamp, que todas as branches copiam):
  lat_proc  = chegou a imagem no medidor -> chegou balls_in_image no medidor (custo da visão)
  lat_total = header.stamp da imagem -> chegou balls_in_image (inclui driver da câmera + transporte)
"""
import csv
import glob
import json
import os
import statistics
import subprocess
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import psutil
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image

OUT_TYPES = {
    'soccer_vision_2d_msgs/msg/BallArray',
    'soccer_vision_2d_msgs/msg/GoalpostArray',
    'soccer_vision_2d_msgs/msg/RobotArray',
    'soccer_vision_2d_msgs/msg/MarkingArray',
}
QOS_OUT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=100)


def stamp_key(h):
    return (h.stamp.sec, h.stamp.nanosec)


def n_detections(msg):
    return sum(len(getattr(msg, f)) for f, t in msg.get_fields_and_field_types().items() if t.startswith('sequence'))


def pct(v, p):
    if not v:
        return None
    v = sorted(v)
    return v[min(len(v) - 1, int(round(p / 100 * (len(v) - 1))))]


def mean(v):
    v = [x for x in v if x is not None]
    return statistics.mean(v) if v else None


def fmt(x, f='{:.0f}'):
    return '-' if x is None else f.format(x)


def gpu_busy():
    """% de uso da GPU: AMD/Intel(xe) via sysfs, NVIDIA via nvidia-smi. None se não houver."""
    vals = []
    for f in glob.glob('/sys/class/drm/card*/device/gpu_busy_percent'):
        try:
            vals.append(float(Path(f).read_text()))
        except (OSError, ValueError):
            pass
    if not vals and os.path.exists('/usr/bin/nvidia-smi'):
        try:
            out = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                                 capture_output=True, text=True, timeout=1).stdout
            vals = [float(x) for x in out.split()]
        except (OSError, ValueError, subprocess.TimeoutExpired):
            pass
    return max(vals) if vals else None


def cpu_freq_mhz():
    """Frequência média dos núcleos (sysfs; o psutil erra a unidade em algumas CPUs AMD)."""
    vals = []
    for f in glob.glob('/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq'):
        try:
            vals.append(int(Path(f).read_text()) / 1000)
        except (OSError, ValueError):
            pass
    if vals:
        return statistics.mean(vals)
    f = psutil.cpu_freq()
    return f.current if f else None


def cpu_temp():
    try:
        temps = psutil.sensors_temperatures()
    except (AttributeError, OSError):
        return None
    vals = [t.current for k, ts in temps.items() if k in ('coretemp', 'k10temp', 'cpu_thermal', 'zenpower', 'acpitz')
            for t in ts]
    return max(vals) if vals else None


class Medidor(Node):
    def __init__(self):
        super().__init__('vision_bench')
        p = lambda n, d: self.declare_parameter(n, d).value
        self.duracao = p('duracao', 0.0)
        self.ignorar = p('ignorar_s', 5.0)
        self.label = p('label', '')
        self.padrao = p('processo', 'object_finder/finder')
        self.saida = Path(os.path.expanduser(p('saida', '~/visao-bench/resultados')))
        image_topic = p('image_topic', '/camera/image_raw')

        self.t_in = {}                       # stamp -> t chegada (monotônico)
        self.janela = deque()                # (t, tipo, valor) do último segundo
        self.frames = []                     # (t, lat_proc_ms, lat_total_ms) por frame respondido
        self.n_in_total = 0
        self.topics, self.dets = {}, {}
        self.t_primeira_saida = None
        self.linhas = []                     # amostras por segundo (CSV)
        self.procs = {}                      # pid -> psutil.Process (mantém o histórico do cpu_percent)
        self.visao_cache, self.ultima_busca = [], 0.0
        self.commit = ''
        self.fim = False
        self.eu = psutil.Process()
        self.eu.cpu_percent()
        psutil.cpu_percent(percpu=True)

        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(1.0, self.amostra)
        self.create_timer(2.0, self.descobrir)
        self.get_logger().info(f'esperando a visão ({self.padrao}) e as saídas soccer_vision_2d_msgs...')
        print('\n   t  fps_in fps_out  lat_proc(p50/p95)  lat_total  | CPU visão  resto  sistema  núcleo_max | '
              'RAM visão  sist%  | temp  freq   GPU%  | det/frame', flush=True)

    # ---------------------------------------------------------------- ROS
    def descobrir(self):
        if len(self.topics) >= 4:
            return
        for name, types in self.get_topic_names_and_types():
            if types and types[0] in OUT_TYPES and name not in self.topics:
                self.topics[name] = types[0]
                self.dets[name] = []
                self.create_subscription(get_message(types[0]), name,
                                         lambda m, n=name: self.on_out(n, m), QOS_OUT)
                self.get_logger().info(f'medindo {name}')

    def principal(self):
        return '/balls_in_image' if '/balls_in_image' in self.topics else next(iter(self.topics), None)

    def on_image(self, msg):
        now = time.monotonic()
        self.t_in[stamp_key(msg.header)] = now
        self.n_in_total += 1
        self.janela.append((now, 'in', None))
        if len(self.t_in) > 2000:            # não cresce para sempre
            for k in list(self.t_in)[:1000]:
                del self.t_in[k]

    def on_out(self, topic, msg):
        now = time.monotonic()
        if self.t_primeira_saida is None:
            self.t_primeira_saida = now
            self.get_logger().info(f'1ª saída recebida; resumo começa em {self.ignorar:.0f}s (aquecimento)')
        if self.medindo(now):
            self.dets[topic].append(n_detections(msg))
        if topic != self.principal():
            return
        t_img = self.t_in.get(stamp_key(msg.header))
        lat_proc = (now - t_img) * 1000 if t_img is not None else None
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lat_total = (time.time() - st) * 1000 if st > 0 else None
        self.janela.append((now, 'out', (lat_proc, lat_total)))
        if self.medindo(now):
            self.frames.append((now, lat_proc, lat_total))

    def medindo(self, now):
        return self.t_primeira_saida is not None and now - self.t_primeira_saida >= self.ignorar

    # ---------------------------------------------------------------- processos
    def processos(self):
        """(processos da visão, outros processos do mesmo launch: usb_cam, ipm...)."""
        visao = [v for v in self.visao_cache if v.is_running()]
        if not visao or time.monotonic() - self.ultima_busca > 5.0:   # varrer tudo é caro: só a cada 5 s
            self.ultima_busca = time.monotonic()
            visao = []
            for proc in psutil.process_iter(['pid', 'cmdline']):
                argv = proc.info['cmdline'] or []
                # o executável (argv[0], ou o script depois do python) termina com o padrão
                exe = next((a for a in argv[:2] if a.endswith(self.padrao)), None)
                if exe and proc.pid != self.eu.pid:
                    visao.append(proc)
                    if not self.commit:
                        self.identificar(exe)
            self.visao_cache = visao
        resto = []
        pais = {v.ppid() for v in visao if v.is_running()}
        ids_visao = {v.pid for v in visao}
        for pp in pais:
            try:
                resto += [c for c in psutil.Process(pp).children(recursive=True) if c.pid not in ids_visao]
            except psutil.Error:
                pass
        return [self.cache(x) for x in visao], [self.cache(x) for x in resto]

    def cache(self, proc):
        if proc.pid not in self.procs:
            try:
                proc.cpu_percent()           # 1ª leitura é sempre 0
            except psutil.Error:
                pass
            self.procs[proc.pid] = proc
        return self.procs[proc.pid]

    def identificar(self, exe):
        """Descobre branch/commit pelo caminho do install de onde o finder roda."""
        exe = os.path.realpath(exe)
        ws = exe.split('/install/')[0] if '/install/' in exe else os.path.dirname(exe)
        git = lambda *a: subprocess.run(['git', '-C', ws, *a], capture_output=True, text=True).stdout.strip()
        self.commit = git('rev-parse', '--short', 'HEAD') or '?'
        branch = git('rev-parse', '--abbrev-ref', 'HEAD')
        if not self.label:
            self.label = branch if branch and branch != 'HEAD' else os.path.basename(ws)
        self.get_logger().info(f'visão: {exe}  (branch {branch or "?"}, commit {self.commit})')

    @staticmethod
    def soma(procs):
        cpu = ram = 0.0
        for proc in procs:
            try:
                cpu += proc.cpu_percent()
                ram += proc.memory_info().rss / 2**20
            except psutil.Error:
                pass
        return cpu, ram

    # ---------------------------------------------------------------- amostragem
    def amostra(self):
        now = time.monotonic()
        while self.janela and now - self.janela[0][0] > 1.0:
            self.janela.popleft()
        fps_in = sum(1 for _, k, _ in self.janela if k == 'in')
        outs = [v for _, k, v in self.janela if k == 'out']
        lp = [a for a, _ in outs if a is not None]
        lt = [b for _, b in outs if b is not None]

        visao, resto = self.processos()
        cpu_v, ram_v = self.soma(visao)
        cpu_r, ram_r = self.soma(resto)
        cores = psutil.cpu_percent(percpu=True)
        row = {
            't': round(now - self.t_primeira_saida, 1) if self.t_primeira_saida else None,
            'medindo': self.medindo(now),
            'fps_in': fps_in, 'fps_out': len(outs),
            'lat_proc_p50_ms': pct(lp, 50), 'lat_proc_p95_ms': pct(lp, 95), 'lat_total_p50_ms': pct(lt, 50),
            'cpu_visao_pct': cpu_v, 'cpu_resto_launch_pct': cpu_r,
            'cpu_sistema_pct': sum(cores) / len(cores), 'cpu_nucleo_max_pct': max(cores),
            'ram_visao_mb': ram_v, 'ram_resto_launch_mb': ram_r, 'ram_sistema_pct': psutil.virtual_memory().percent,
            'temp_cpu_c': cpu_temp(), 'freq_mhz': cpu_freq_mhz(), 'gpu_pct': gpu_busy(),
            'cpu_medidor_pct': self.eu.cpu_percent(),
        }
        self.linhas.append(row)

        if not visao:
            print(f'   (nenhum processo com "{self.padrao}" — o launch da branch está rodando?)', flush=True)
            return
        det = ' '.join(f'{n.strip("/").split("_")[0][:4]}={statistics.mean(d[-30:]):.1f}'
                       for n, d in self.dets.items() if d)
        tag = ' ' if row['medindo'] else '*'
        print(f'{tag}{fmt(row["t"], "{:4.0f}")}  {fps_in:6d} {len(outs):7d}  '
              f'{fmt(row["lat_proc_p50_ms"], "{:7.0f}")}/{fmt(row["lat_proc_p95_ms"], "{:<6.0f}")}ms  '
              f'{fmt(row["lat_total_p50_ms"], "{:6.0f}")}ms  | {cpu_v:8.0f}% {cpu_r:5.0f}% {row["cpu_sistema_pct"]:7.0f}% '
              f'{row["cpu_nucleo_max_pct"]:10.0f}% | {ram_v:7.0f}MB {row["ram_sistema_pct"]:5.0f}% | '
              f'{fmt(row["temp_cpu_c"], "{:3.0f}")}°C {fmt(row["freq_mhz"], "{:5.0f}")} {fmt(row["gpu_pct"], "{:5.0f}")}% | {det}',
              flush=True)

        if self.duracao > 0 and self.medindo(now) and now - self.t_primeira_saida - self.ignorar >= self.duracao:
            self.fim = True

    # ---------------------------------------------------------------- resumo
    def resumo(self):
        rows = [r for r in self.linhas if r['medindo']]
        col = lambda k: [r[k] for r in rows if r[k] is not None]
        lp = [f[1] for f in self.frames if f[1] is not None]
        lt = [f[2] for f in self.frames if f[2] is not None]
        dur = (self.frames[-1][0] - self.frames[0][0]) if len(self.frames) > 1 else 0
        fps_in = mean(col('fps_in'))
        fps_out = (len(self.frames) - 1) / dur if dur > 0 else None
        return {
            'label': self.label or 'sem_nome', 'commit': self.commit,
            'data': datetime.now().isoformat(timespec='seconds'),
            'duracao_medida_s': round(dur, 1), 'frames_respondidos': len(self.frames),
            'fps_in': fps_in, 'fps_out': fps_out,
            'perdidos_pct': (100 * (1 - fps_out / fps_in)) if fps_in and fps_out else None,
            'lat_proc_media_ms': mean(lp), 'lat_proc_p50_ms': pct(lp, 50), 'lat_proc_p95_ms': pct(lp, 95),
            'lat_proc_max_ms': max(lp) if lp else None,
            'lat_total_p50_ms': pct(lt, 50), 'lat_total_p95_ms': pct(lt, 95),
            'cpu_visao_media_pct': mean(col('cpu_visao_pct')), 'cpu_visao_max_pct': max(col('cpu_visao_pct'), default=None),
            'cpu_resto_launch_media_pct': mean(col('cpu_resto_launch_pct')),
            'cpu_sistema_media_pct': mean(col('cpu_sistema_pct')),
            'ram_visao_max_mb': max(col('ram_visao_mb'), default=None),
            'ram_sistema_max_pct': max(col('ram_sistema_pct'), default=None),
            'temp_cpu_max_c': max(col('temp_cpu_c'), default=None),
            'freq_media_mhz': mean(col('freq_mhz')),
            'gpu_media_pct': mean(col('gpu_pct')),
            'cpu_medidor_media_pct': mean(col('cpu_medidor_pct')),
            'deteccoes_por_frame': {t: (statistics.mean(d) if d else 0) for t, d in self.dets.items()},
            'nucleos': psutil.cpu_count(),
        }

    def salvar(self):
        r = self.resumo()
        if not self.frames:
            print('\nnada medido (nenhuma saída após o aquecimento) — nada salvo.', flush=True)
            return
        self.saida.mkdir(parents=True, exist_ok=True)
        base = self.saida / f'{r["label"]}_{datetime.now():%Y%m%d-%H%M%S}'
        base.with_suffix('.json').write_text(json.dumps(r, indent=2, ensure_ascii=False))
        with open(base.with_suffix('.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.linhas[0]))
            w.writeheader()
            w.writerows(self.linhas)
        print('\n==== RESUMO', r['label'], r['commit'], '====', flush=True)
        for k, v in r.items():
            if isinstance(v, float):
                v = round(v, 1)
            print(f'  {k:28s} {v}', flush=True)
        print(f'\nsalvo em {base}.json / .csv   (compare com: ros2 run vision_bench comparar)', flush=True)


def main():
    rclpy.init()
    node = Medidor()
    try:
        while rclpy.ok() and not node.fim:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.salvar()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
