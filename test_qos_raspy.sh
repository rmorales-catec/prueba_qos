#!/bin/zsh

# Lista de archivos XML de QoS a probar
QOS_PROFILES=(
    "$HOME/prueba_qos/Config/1.xml"
    "$HOME/prueba_qos/Config/2.xml"
    "$HOME/prueba_qos/Config/3.xml"
)

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

    echo "🚀 Lanzando nodo de publicación de imagen..."
    ros2 run prueba_rmw image_publisher_compressed &
    PUB_PID=$!

    sleep $DURATION

  # Finalizar nodos
    echo "🛑 Matando nodos..."

    # Cerrar el subscriber (usa cámara) completamente
    pkill -f image_publisher_compressed
    sleep 1
    while pgrep -f image_publisher_compressed > /dev/null; do
        echo "⏳ Esperando que image_publisher termine..."
        sleep 1
    done
    echo "✅ image_publisher cerrado"

    echo "✅ Publicación finalizada para perfil $(basename "$PROFILE")"

    # Incrementar DOMAIN_ID para evitar conflictos
    ((DOMAIN_ID++))
    sleep 5
done

echo "\n🎉 Publicación con todos los perfiles completada."