#!/bin/zsh

PACKAGE="prueba_rmw"
EXECUTABLE_PUB="image_publisher_compressed"
TOPIC="/image_compressed"
NODE_NAME_SUB="image_subscriber_qos"

echo "Paquete: $PACKAGE"
echo "Ejecutable: $EXECUTABLE_PUB"


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

wait_for_node() {
    echo "🔎 Esperando a que el nodo se detenga..."

    for i in {1..60}; do
        HAS_TOPIC=$(ros2 node list 2>/dev/null | grep -q "$NODE_NAME_SUB" && echo "0" || echo "1")

        if [[ "$HAS_TOPIC" == "1" ]]; then
            echo "✅ Nodo eliminado"
            sleep 1
            return 0
        fi

        echo "⏳ Esperando ($i)..."
        sleep 1
    done

    echo "❌ Nodo detectado tras 60 segundos"
    return 1
}

DURATION=30  # segundos de publicación
BASE_DOMAIN_ID=90
DOMAIN_ID=$BASE_DOMAIN_ID

cd ~/ros2_ws
source install/setup.zsh

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

    echo "🚀 Lanzando nodo de publicación..."
    cd ~/ros2_ws
    source install/setup.zsh
    ros2 run "$PACKAGE" "$EXECUTABLE_PUB" &
    PUB_PID=$!

    if ! wait_for_node; then
        echo "⚠️ Saltando perfil por timeout en nodos"
        continue
    fi

    sleep $DURATION

  # Finalizar nodos
    echo "🛑 Matando nodos..."

    pkill -f "$EXECUTABLE_PUB"
    sleep 1
    while pgrep -f "$EXECUTABLE_PUB" > /dev/null; do
        echo "⏳ Esperando que "$EXECUTABLE_PUB" termine..."
        sleep 1
    done
    echo "✅ $EXECUTABLE_PUB cerrado"

    echo "✅ Publicación finalizada para perfil $(basename "$PROFILE")"

    # Incrementar DOMAIN_ID para evitar conflictos
    ((DOMAIN_ID++))
    sleep 2
done

echo "\n🎉 Publicación con todos los perfiles completada."