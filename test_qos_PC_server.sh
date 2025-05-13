#!/bin/zsh

PACKAGE="prueba_rmw"
EXECUTABLE_SUB="image_subscriber_compressed"
TOPIC="/image_compressed"
INTERFACE="enxc84d44228958"

echo "Paquete: $PACKAGE"
echo "Ejecutable: $EXECUTABLE_SUB"
echo "Interfaz de Red: $INTERFACE"


# Lista de archivos XML de QoS a probar
QOS_PROFILES=(
        "$HOME/prueba_qos/Config"$TOPIC"_server/1.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/2.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/3.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/4.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/5.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/6.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/7.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/8.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/9.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/10.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/11.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/12.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/13.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/14.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/15.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/16.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/17.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/18.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/19.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/20.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/21.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/22.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/23.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/24.xml"
        "$HOME/prueba_qos/Config"$TOPIC"_server/25.xml"
)

DURATION=20  # segundos de medición
LOG_DIR="$HOME/prueba_qos/Resultados"
mkdir -p "$LOG_DIR"

BASE_DOMAIN_ID=90
DOMAIN_ID=$BASE_DOMAIN_ID

cd ~/ros2_ws
source install/setup.zsh

# Limpiamos puertos: 
sudo fuser -k :11811/udp
sudo fuser -k :11888/udp
#Lanzamos servers
fastdds discovery -i 0 -l 10.1.0.191 -p 11811 &
SERVER1_PID=$!
fastdds discovery -i 1 -l 10.1.0.191 -p 11888 &
SERVER2_PID=$!

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
    export ROS_DISCOVERY_SERVER="10.1.0.191:11811;10.1.0.191:11888"

    echo "➡️  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "➡️  FASTRTPS_DEFAULT_PROFILES_FILE: $FASTRTPS_DEFAULT_PROFILES_FILE"
    echo "➡️  ROS_DISCOVERY_SERVER: $ROS_DISCOVERY_SERVER"

    ros2 daemon stop
    sleep 2

    if ! wait_for_topic; then
        echo "⚠️ Saltando perfil $PROFILE por timeout en topics"
        continue
    fi

    PROFILE_NAME=$(basename "$PROFILE" .xml)

    echo "🚀 Lanzando nodo de suscripción..."
    cd ~/ros2_ws
    source install/setup.zsh
    ros2 run "$PACKAGE" "$EXECUTABLE_SUB" &
    SUB_PID=$!
    sleep 2

    (
        sleep 7
        echo "📡 Iniciando captura de paquetes con tcpdump..."
        TCPDUMP_TMP="/tmp/${PROFILE_NAME}.pcap"
        TCPDUMP_PID_FILE="/tmp/tcpdump.pid"

        sudo tcpdump -i "$INTERFACE" -w "$TCPDUMP_TMP" &
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

    pkill -f "$EXECUTABLE_SUB"
    sleep 1
    while pgrep -f "$EXECUTABLE_SUB" > /dev/null; do
        echo "⏳ Esperando que "$EXECUTABLE_SUB" termine..."
        sleep 1
    done
    echo "✅ $EXECUTABLE_SUB cerrado"

    echo "✅ Pruebas completadas para $PROFILE_NAME.xml"

    ((DOMAIN_ID++))
    sleep 5
done

echo "\n📁 Todas las pruebas de QoS completadas. Resultados en: $LOG_DIR"

mover_pcaps

# Limpiamos puertos: 
sudo fuser -k :11811/udp
sudo fuser -k :11888/udp

# Generar gráficos
echo "📊 Generando gráficos..."
cd $HOME/prueba_qos
python3 graficos_qos.py