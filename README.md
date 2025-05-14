# prueba_qos
Comparación de diferentes perfiles del QoS de ROS2 para cualquier paquete ROS2.

## Replicar experimentos
Para utilizarlo con otro paquete que no sea el de *prueba_rmw*, se deben cambiar de las primeras líneas: 
- test_qos_raspy.sh: nombre del paquete ROS2, nombre del ejecutable del publisher, nombre del topic que queremos capturar y nombre del nodo que se ejecuta como subscriber.
- test_qos_PC.sh: nombre del paquete ROS2, nombre del ejecutable del subscriber, nombre del topic que queremos capturar y nombre de la interfaz de red que estamos utilizando para comunicarnos con la Raspberry.
- test_qos_raspy_server.sh: nombre del paquete ROS2, nombre del ejecutable del publisher, nombre del topic que queremos capturar y nombre del nodo que se ejecuta como subscriber.
- test_qos_PC_server: nombre del paquete ROS2, nombre del ejecutable del subscriber, nombre del topic que queremos capturar, nombre de la interfaz de red que estamos utilizando para comunicarnos con la Raspberry e IP del Discovery Server (debe coincidir con la IP del PC de la red común con la Raspberry).

