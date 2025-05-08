#!/bin/zsh

TOPIC="/image_compressed"

# Lista de archivos XML de QoS a probar
QOS_PROFILES=(
        "$HOME/prueba_qos/Config"$TOPIC"/1.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/2.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/3.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/4.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/5.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/6.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/7.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/8.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/9.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/10.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/11.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/12.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/13.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/14.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/15.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/16.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/17.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/18.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/19.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/20.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/21.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/22.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/23.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/24.xml"
        "$HOME/prueba_qos/Config"$TOPIC"/25.xml"
)

DURATION=20  # segundos de medición
LOG_DIR="$HOME/prueba_qos/Resultados"
mkdir -p "$LOG_DIR"

BASE_DOMAIN_ID=90
DOMAIN_ID=$BASE_DOMAIN_ID

cd ~/ros2_ws
source install/setup.zsh


wait_for_topic() {
    echo "🔎 Esperando a que el topic '$TOPIC' esté disponible..."

    for i in {1..60}; do
        HAS_TOPIC=$(ros2 topic list 2>/dev/null | grep -q "$TOPIC" && echo "1" || echo "0")

        if [[ "$HAS_TOPIC" == "1" ]]; then
            echo "✅ Topic disponible: $TOPIC"
            sleep 5
            return 0
        fi

        echo "⏳ Esperando ($i)..."
        sleep 1
    done

    echo "❌ Topic $TOPIC no detectado tras 60 segundos"
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

    ros2 daemon stop
    sleep 2

    if ! wait_for_topic; then
        echo "⚠️ Saltando perfil $PROFILE por timeout en topics"
        continue
    fi

    PROFILE_NAME=$(basename "$PROFILE" .xml)

    # echo "🚀 Lanzando nodo de suscripción de imagen..."
    # cd ~/ros2_ws
    # source install/setup.zsh
    # if [[ "$TOPIC" == "/image_compressed" ]]; then
    #     ros2 run prueba_rmw image_publisher_compressed &
    # elif [[ "$TOPIC" == "/chatter" ]]; then
    #     ros2 run demo_nodes_cpp talker &
    # elif [[ "$TOPIC" == "/image" ]]; then
    #     ros2 run image_tools cam2image &
    # fi
    # SUB_PID=$!
    sleep 2

    (
        sleep 7
        echo "📡 Iniciando captura de paquetes con tcpdump..."
        TCPDUMP_TMP="/tmp/${PROFILE_NAME}.pcap"
        TCPDUMP_PID_FILE="/tmp/tcpdump.pid"

        sudo tcpdump -i enxc84d44228958 -w "$TCPDUMP_TMP" &
        TCPDUMP_PID=$!
        echo "$TCPDUMP_PID" > "$TCPDUMP_PID_FILE"
        echo "Captura guardada en: $TCPDUMP_TMP (PID: $TCPDUMP_PID)"
        sleep 2
    ) &

    (
        sleep 13
        echo "🛑 Deteniendo captura de paquetes (auto)"
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
    timeout ${DURATION}s ros2 topic hz "$TOPIC" -w 30 | tee "$LOG_DIR/${PROFILE_NAME}_hz.txt" &
    HZ_PID=$!

    timeout ${DURATION}s ros2 topic delay "$TOPIC" -w 30 | tee "$LOG_DIR/${PROFILE_NAME}_delay.txt" &
    DELAY_PID=$!

    timeout ${DURATION}s ros2 topic bw "$TOPIC" | tee "$LOG_DIR/${PROFILE_NAME}_bw.txt" &
    BW_PID=$!

    wait $HZ_PID
    wait $DELAY_PID
    wait $BW_PID


    echo "🛑 Deteniendo nodos..."

    if [[ "$TOPIC" == "/image_compressed" ]]; then
        pkill -f image_subscriber
        sleep 1
        while pgrep -f image_subscriber > /dev/null; do
            echo "⏳ Esperando que image_subscriber termine..."
            sleep 1
        done
        echo "✅ Nodo image_subscriber cerrado"
    elif [[ "$TOPIC" == "/chatter" ]]; then
        pkill -f showimage
        sleep 1
        while pgrep -f showimage > /dev/null; do
            echo "⏳ Esperando que showimage termine..."
            sleep 1
        done
        echo "✅ Nodo showimage cerrado"
    elif [[ "$TOPIC" == "/image" ]]; then
        pkill -f cam2image
        sleep 1
        while pgrep -f cam2image > /dev/null; do
            echo "⏳ Esperando que cam2image termine..."
            sleep 1
        done
        echo "✅ Nodo cam2image"
    fi

    echo "✅ Pruebas completadas para $PROFILE_NAME.xml"

    ((DOMAIN_ID++))
    sleep 5
done

echo "\n📁 Todas las pruebas de QoS completadas. Resultados en: $LOG_DIR"

# Generar gráficos
echo "📊 Generando gráficos..."
cd $HOME/prueba_qos
python3 graficos_qos.py
