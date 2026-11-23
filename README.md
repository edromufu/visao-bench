# Medidor de desempenho da visão (`vision_bench`)

Workspace separado do repositório: o medidor é o mesmo para todas as branches.

## Uso

```bash
# uma vez (ou quando mudar o medidor)
cd ~/visao-bench/ws && colcon build --symlink-install

# recomendado, para os números não variarem com a frequência da CPU
sudo cpupower frequency-set -g performance
```

Para cada branch:

```bash
# terminal 1: a branch, do jeito normal
cd ~/visao-cbr26 && git checkout <branch>
cd visao-cbr26 && rm -rf build install log && colcon build --symlink-install
source install/setup.bash
ros2 launch object_finder vision.launch.py        # + o que desligar a janela, ex. show_window:=false

# terminal 2: o medidor
source /opt/ros/humble/setup.bash && source ~/visao-bench/ws/install/setup.bash
ros2 launch vision_bench medir.launch.py duracao:=60
```

Depois de medir todas: `ros2 run vision_bench comparar` (também grava `resultados/resumo.csv`).

## Argumentos do `medir.launch.py`

| arg | padrão | |
|---|---|---|
| `duracao` | `0` | segundos medidos depois do aquecimento; `0` = até Ctrl+C |
| `ignorar_s` | `5` | segundos após a 1ª detecção que ficam fora do resumo (linhas com `*`) |
| `label` | branch git | nome no resultado; por padrão é a branch de onde o `finder` foi instalado |
| `processo` | `object_finder/finder` | executável do nó de visão |
| `image_topic` | `/camera/image_raw` | |
| `saida` | `~/visao-bench/resultados` | |

## O que é medido

- **fps in / fps out**: imagens da câmera e `balls_in_image` por segundo; **perd%** = frames sem resposta.
- **lat_proc**: chegada da imagem → chegada da saída de mesmo `header.stamp` (custo da visão).
- **lat_total**: `header.stamp` da imagem → chegada da saída (inclui driver da câmera).
- **CPU visão / RAM visão**: processo `finder`. **CPU resto**: os outros nós do mesmo launch (usb_cam, ipm...).
  CPU% é a soma dos núcleos (100% = 1 núcleo cheio).
- **Máquina**: CPU total, núcleo mais carregado, RAM %, temperatura, frequência, GPU% (AMD/Intel via sysfs, NVIDIA via nvidia-smi).
- **det/frame**: detecções médias por frame, para ver se a branch está detectando de fato.

Cada medição gera `<label>_<data>.json` (resumo) e `.csv` (uma linha por segundo).
