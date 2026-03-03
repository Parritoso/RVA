# RVA

## 1. Inicio del Entorno (Docker)
- Ejecutar el contenedor (con GPU NVIDIA):
  
    ```bash
  ./run_container.gpu.bash
    ```
  
- Ejecutar el contenedor en otra consola:

    ```bash
  docker exec -it rva_container_gpu bash
    ```

> [!IMPORTANT]
> Ejecutar ```source install/local_setup.bash``` en las 2 consolas
  
## 2. Compilación del Workspace
  Una vez dentro del contenedor, es necesario compilar el espacio de trabajo (rva_ws) antes de ejecutar cualquier nodo:
  
  **1. Navegar al workspace:**
  
  ```bash
  cd ~/rva_ws
```
  
  **2. Compilar con colcon:**
  
  ```bash
  colcon build --symlink-install
```
  
  **3. Cargar las variables de entorno del workspace:**
  
  ```bash
  source install/local_setup.bash
```

  ## 3. Ejecución de la Simulación
  Para iniciar el simulador Gazebo con el modelo del TurtleBot3:

  ```bash
  ros2 launch turtlebot3_gazebo empty_world.launch.py
```
  Ejecutar un nodo:
  ```bash
    ros2 run epd1 controlGoal_node
  ```

## 4. Herramientas de Inspección y Control
Comandos útiles para depurar y visualizar el estado del sistema:

* **Visualización en RViz:**
    ```bash
    ros2 run rviz2 rviz2
    ```
* **Listar nodos activos:**
    ```bash
    ros2 node list
    ```
* **Listar parámetros de los nodos:**
    ```bash
    ros2 param list
    ```
* **Publicar un objetivo (Goal) manualmente por consola:**
    ```bash
    ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}}}"
    ```
