import os
import pyshark
import numpy as np
import matplotlib.pyplot as plt

# Directorio con capturas pcap
LOG_DIR = os.path.expanduser('~/prueba_qos/Resultados')

# Lista de nombres de archivos pcap generados automáticamente
pcap_files = [f'{i}.pcap' for i in range(1, 26)]

# Métricas a recolectar
pcap_labels = []
total_packets = []
avg_packet_sizes = []

def process_pcap(pcap_path):
    """
    Procesa un archivo pcap y devuelve:
    - count: número total de paquetes UDP o TCP capturados
    - avg_size: tamaño promedio de esos paquetes (bytes)
    """
    try:
        cap = pyshark.FileCapture(pcap_path, display_filter='', keep_packets=False)
        sizes = []
        for pkt in cap:
            try:
                sizes.append(int(pkt.length))
            except (AttributeError, ValueError):
                continue
        cap.close()
        count = len(sizes)
        avg_size = float(np.mean(sizes)) if sizes else 0.0
        return count, avg_size
    except Exception as e:
        print(f"❌ Error procesando {pcap_path}: {e}")
        return 0, 0.0

# Procesar cada archivo pcap
def process_all_pcaps():
    for filename in pcap_files:
        pcap_path = os.path.join(LOG_DIR, filename)
        if not os.path.exists(pcap_path):
            print(f"⚠️ No existe el archivo {pcap_path}")
            continue

        count, avg_size = process_pcap(pcap_path)
        label = os.path.splitext(filename)[0]
        pcap_labels.append(label)
        total_packets.append(count)
        avg_packet_sizes.append(avg_size)
        print(f"{label}: paquetes_totales={count}, tamaño_medio={avg_size:.2f} bytes")

# Función para graficar
def plot_bar(values, labels, title, ylabel, filename=None, fmt='int'):
    plt.figure(figsize=(10, 6))
    bars = plt.bar(labels, values, color='skyblue', edgecolor='black')
    for bar, val in zip(bars, values):
        text = f"{int(val)}" if fmt == 'int' else f"{val:.2f}"
        plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + max(values) * 0.01, text, ha='center')
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel('Captura')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    if filename:
        plt.savefig(filename)
    plt.show()

# Ejecutar el procesamiento y graficar los resultados
process_all_pcaps()

# Graficar número total de paquetes
plot_bar(total_packets, pcap_labels, 'Total de paquetes TCP/UDP por Captura', 'Número de paquetes', filename='total_packets.png')

# Graficar tamaño medio de los paquetes
plot_bar(avg_packet_sizes, pcap_labels, 'Tamaño medio de paquetes TCP/UDP por Captura', 'Tamaño medio (bytes)', filename='avg_packet_size.png')
