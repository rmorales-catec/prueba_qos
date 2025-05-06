#!/bin/zsh

# Lista de archivos XML de QoS a probar
QOS_PROFILES=(
    "$HOME/prueba_qos/Config/1.xml"
    "$HOME/prueba_qos/Config/2.xml"
    "$HOME/prueba_qos/Config/3.xml"
)

DURATION=20  # segundos de medición
LOG_DIR="$HOME/prueba_qos/Resultados"
mkdir -p "$LOG_DIR"

BASE_DOMAIN_ID=90
DOMAIN_ID=$BASE_DOMAIN_ID

cd ~/ros2_ws
source install/setup.zsh

# Función que espera a que el topic esté disponible
wait_for_topic_image() {
    echo "🔎 Esperando a que el topic /image_compressed esté disponible..."

    for i in {1..60}; do
        HAS_IMAGE=$(ros2 topic list 2>/dev/null | grep -q "/image_compressed" && echo "1" || echo "0")

        if [[ "$HAS_IMAGE" == "1" ]]; then
            echo "✅ Topic de imagen disponible"
            sleep 5
            return 0
        fi

        echo "⏳ Esperando ($i)..."
        sleep 1
    done

    echo "❌ Topic /image_compressed no detectado tras 60 segundos"
    return 1
}

mover_pcaps() {
    echo "📦 Moviendo archivos .pcap desde /tmp a $LOG_DIR"

    local files=(/tmp/*.pcap)

    if [[ ${#files[@]} -eq 0 ]]; then
        echo "⚠️ No se encontraron archivos .pcap en /tmp"
        return
    fi

    for file in "${files[@]}"; do
        echo "➡️  Moviendo $(basename "$file")"
        sudo mv "$file" "$LOG_DIR/"
    done

    # Aseguramos que el usuario tenga permisos sobre los archivos
    sudo chown $USER:$USER "$LOG_DIR"/*.pcap

    echo "✅ Todos los archivos .pcap fueron movidos"
}

for PROFILE in "${QOS_PROFILES[@]}"; do
    echo "\n🔧 Probando perfil QoS: $PROFILE"

    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=$PROFILE
    export ROS_DOMAIN_ID=$DOMAIN_ID 

    echo "➡️  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "➡️  FASTRTPS_DEFAULT_PROFILES_FILE: $FASTRTPS_DEFAULT_PROFILES_FILE"


    # Reiniciar el daemon
    ros2 daemon stop
    sleep 2

    if ! wait_for_topic_image; then
        echo "⚠️ Saltando perfil $PROFILE por timeout en topics"
        continue
    fi

    PROFILE_NAME=$(basename "$PROFILE" .xml)

    echo "🚀 Lanzando nodo de suscripción de imagen..."
    ros2 run prueba_rmw image_subscriber_compressed &
    SUB_PID=$!
    sleep 2


    # Captura de paquetes
    (
        sleep 7
        echo "📡 Iniciando captura de paquetes con tcpdump..."
        TCPDUMP_TMP="/tmp/${PROFILE}.pcap"
        TCPDUMP_PID_FILE="/tmp/tcpdump.pid"  # Archivo para guardar el PID

        # Ejecutar tcpdump en segundo plano y guardar su PID
        sudo tcpdump -i enxc84d44228958 -w "$TCPDUMP_TMP" &
        TCPDUMP_PID=$!
        echo "$TCPDUMP_PID" > "$TCPDUMP_PID_FILE"
        echo "Captura guardada en: $TCPDUMP_TMP (PID: $TCPDUMP_PID)"
        sleep 2
    ) &


    (
        sleep 13
        echo "🛑 Deteniendo captura de paquetes (auto)"
        # Leer el PID desde el archivo
        TCPDUMP_PID_FILE="/tmp/tcpdump.pid"
        if [ -f "$TCPDUMP_PID_FILE" ]; then
            TCPDUMP_PID=$(cat "$TCPDUMP_PID_FILE")
            if ps -p "$TCPDUMP_PID" > /dev/null; then
                sudo kill -SIGINT "$TCPDUMP_PID"
                echo "✅ Captura de paquetes cerrada (PID: $TCPDUMP_PID)"
            else
                echo "⚠️ tcpdump ya no está corriendo"
            fi
            rm "$TCPDUMP_PID_FILE"
            echo "PID eliminado"
        else
            echo "❌ Archivo PID no encontrado"
        fi
    ) &

    # Medir frecuencia, delay, bandwidth
    timeout ${DURATION}s ros2 topic hz /image_compressed -w 30 | tee "$LOG_DIR/${PROFILE_NAME}_image_hz.txt" &
    HZ_PID=$!

    timeout ${DURATION}s ros2 topic delay /image_compressed -w 30 | tee "$LOG_DIR/${PROFILE_NAME}_image_delay.txt" &
    DELAY_PID=$!

    timeout ${DURATION}s ros2 topic bw /image_compressed | tee "$LOG_DIR/${PROFILE_NAME}_image_bw.txt" &
    BW_PID=$!

    wait $HZ_PID
    wait $DELAY_PID
    wait $BW_PID

    echo "🛑 Deteniendo nodos..."
    # Detenemos el subscriber de imágenes
    pkill -f image_subscriber_compressed
    sleep 1

    # Espera hasta que el proceso se cierre completamente
    while pgrep -f image_subscriber_compressed > /dev/null; do
        echo "⏳ Esperando que image_subscriber termine..."
        sleep 1
    done
    echo "✅ Nodo image_subscriber cerrado"

    echo "✅ Pruebas completadas para $PROFILE.xml"

    # Incrementar para la siguiente iteración
    ((DOMAIN_ID++))
    sleep 5
done

echo "\n📁 Todas las pruebas de QoS completadas. Resultados en: $LOG_DIR"

# Generar gráficos
echo "📊 Generando gráficos..."
cd $HOME/qos_logs
python3 graficos_qos.py
