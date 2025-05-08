import matplotlib.pyplot as plt
import os
import re
from glob import glob

log_dir = os.path.expanduser('~/prueba_qos/Resultados')

def parse_file(filepath, mode='hz'):
    values = []
    if not os.path.exists(filepath):
        print(f"⚠️  No existe el archivo {filepath}")
        return values
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if mode == 'hz':
                m = re.search(r'average rate:\s+([0-9.]+)', line)
                if m:
                    values.append(float(m.group(1)))
                elif 'no new messages' in line:
                    values.append(0.0)
            elif mode == 'delay':
                m = re.search(r'average delay:\s+(-?[0-9.]+)', line)
                if m:
                    values.append(float(m.group(1)))
                elif 'no new messages' in line:
                    values.append(0.0)
            elif mode == 'bw':
                m = re.search(r'([0-9.]+)\s*(B|KB|MB)/s', line, re.IGNORECASE)
                if m:
                    value = float(m.group(1))
                    unit = m.group(2).upper()
                    if unit == 'B':
                        value = value / (1024.0 * 1024.0)
                    elif unit == 'KB':
                        value = value / 1024.0
                    values.append(value)
                elif 'no new messages' in line:
                    values.append(0.0)
    print(f"{os.path.basename(filepath)} → {len(values)} muestras")
    return values

# Recopilar archivos por tipo
data_hz = {}
data_delay = {}
data_bw = {}

hz_files = sorted(glob(os.path.join(log_dir, '*_hz.txt')))
delay_files = sorted(glob(os.path.join(log_dir, '*_delay.txt')))
bw_files = sorted(glob(os.path.join(log_dir, '*_bw.txt')))

for file in hz_files:
    name = os.path.basename(file).split('_')[0]
    data_hz[name] = parse_file(file, mode='hz')

for file in delay_files:
    name = os.path.basename(file).split('_')[0]
    data_delay[name] = parse_file(file, mode='delay')

for file in bw_files:
    name = os.path.basename(file).split('_')[0]
    data_bw[name] = parse_file(file, mode='bw')

def plot_metric(data, title, ylabel, filename, marker='o', linestyle='--'):
    plt.figure(figsize=(10, 5))
    all_vals = [v for vals in data.values() for v in vals]
    if all_vals:
        mn, mx = min(all_vals), max(all_vals)
        margin = abs(0.1 * max(abs(mn), abs(mx)))
        plt.ylim(mn - margin, mx + margin)
    for label, vals in data.items():
        if not vals:
            continue
        x = list(range(len(vals)))
        plt.plot(x, vals, label=f"Perfil {label}", marker=marker, linestyle=linestyle, markersize=3)
        for i, v in enumerate(vals):
            if v == 0:
                plt.plot(i, 0, 'ro')
    plt.title(title)
    plt.xlabel("Muestra")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(log_dir, filename))
    plt.show()

# Generar gráficas
plot_metric(data_hz, "Frecuencia", "Hz", "frecuencia.png")
plot_metric(data_delay, "Delay", "s", "delay.png", marker='x')
plot_metric(data_bw, "Ancho de banda", "MB/s", "ancho_banda.png", marker='x')
