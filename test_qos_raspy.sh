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
    cd ~/ros2_ws
    source install/setup.zsh
    if [[ "$TOPIC" == "/image_compressed" ]]; then
        ros2 run prueba_rmw image_publisher_compressed &
        PUB_PID=$!
    elif [[ "$TOPIC" == "/chatter" ]]; then
        ros2 run demo_nodes_cpp talker &
        PUB_PID=$!
    elif [[ "$TOPIC" == "/image" ]]; then
        ros2 run image_tools cam2image &
        PUB_PID=$!
    fi


    sleep $DURATION

  # Finalizar nodos
    echo "🛑 Matando nodos..."

    # Cerrar el subscriber (usa cámara) completamente
    if [[ "$TOPIC" == "/image_compressed" ]]; then
        pkill -f image_publisher
        sleep 1
        while pgrep -f image_publisher > /dev/null; do
            echo "⏳ Esperando que image_publisher termine..."
            sleep 1
        done
        echo "✅ image_publisher cerrado"
    elif [[ "$TOPIC" == "/chatter" ]]; then
        pkill -f talker
        sleep 1
        while pgrep -f talker > /dev/null; do
            echo "⏳ Esperando que talker termine..."
            sleep 1
        done
        echo "✅ talker cerrado"
    elif [[ "$TOPIC" == "/image" ]]; then
        pkill -f cam2image
        sleep 1
        while pgrep -f cam2image > /dev/null; do
            echo "⏳ Esperando que cam2image termine..."
            sleep 1
        done
        echo "✅ cam2image cerrado"
    fi

    echo "✅ Publicación finalizada para perfil $(basename "$PROFILE")"

    # Incrementar DOMAIN_ID para evitar conflictos
    ((DOMAIN_ID++))
    sleep 5
done

echo "\n🎉 Publicación con todos los perfiles completada."