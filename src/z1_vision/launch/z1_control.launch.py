#!/usr/bin/env python3
"""
Z1 Control Launch
=================
Avvia il pipeline di controllo del braccio Z1:

  Ordine di avvio:
  t=0s  safe_controller_switch     : servizi /safe_switch/to_torque e /safe_switch/to_jtc
  t=0s  z1_ik_to_jtc               : IK (Pinocchio) → JointTrajectoryController
  t=0s  impedance_controller       : safe startup interno (3s), poi standby fino a enable
  t=5s  z1_FSM                     : orchestra tutto (primo stato: HOMING → WAITING)

Nota: il nodo z1_keyboard_safety NON viene avviato qui perché richiede
un terminale interattivo dedicato. Avviarlo separatamente:
    ros2 run z1_vision z1_keyboard_safety

Da lanciare DOPO:
  1. z1_realsense.launch.py   (robot hw + JTC attivo + camera + TF)
  2. z1_perception.launch.py  (YOLO tracker + surface node)

Uso:
    ros2 launch z1_vision z1_control.launch.py
    ros2 launch z1_vision z1_control.launch.py fsm_delay:=8.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('z1_vision')

    # ── Config files ───────────────────────────────────────────────────
    ik_params        = PathJoinSubstitution([pkg, 'config', 'z1_ik_jtc_params.yaml'])
    impedance_params = PathJoinSubstitution([pkg, 'config', 'impedance_control_params.yaml'])
    fsm_params       = PathJoinSubstitution([pkg, 'config', 'z1_fsm_params.yaml'])

    # ── Launch arguments ───────────────────────────────────────────────
    fsm_delay_arg = DeclareLaunchArgument(
        'fsm_delay',
        default_value='5.0',
        description='Secondi di attesa prima di avviare la FSM '
                    '(lascia tempo al safe startup di impedance: 3s)'
    )
    use_ik_arg = DeclareLaunchArgument(
        'use_ik',
        default_value='true',
        description='Avvia il nodo IK → JTC'
    )
    use_impedance_arg = DeclareLaunchArgument(
        'use_impedance',
        default_value='true',
        description='Avvia il nodo impedance controller'
    )
    use_fsm_arg = DeclareLaunchArgument(
        'use_fsm',
        default_value='true',
        description='Avvia la FSM (primo stato: HOMING)'
    )

    fsm_delay     = LaunchConfiguration('fsm_delay')
    use_ik        = LaunchConfiguration('use_ik')
    use_impedance = LaunchConfiguration('use_impedance')
    use_fsm       = LaunchConfiguration('use_fsm')

    # ── Nodi ──────────────────────────────────────────────────────────

    # NODO 1 — Safe Controller Switch  (t = 0s)
    # Espone i servizi /safe_switch/to_torque e /safe_switch/to_jtc.
    # Nessun parametro YAML: opera tramite /controller_manager.
    switch_node = Node(
        package    = 'z1_vision',
        executable = 'safe_controller_switch',
        name       = 'safe_controller_switch',
        output     = 'screen',
    )

    # NODO 2 — IK → JointTrajectoryController  (t = 0s)
    # Risolve l'IK con Pinocchio (damped Jacobian) e invia la traiettoria
    # al JTC tramite action. Attende /ik_enable prima di attivarsi.
    ik_to_jtc_node = Node(
        package    = 'z1_vision',
        executable = 'z1_ik_to_jtc',
        name       = 'z1_ik_to_jtc',
        parameters = [ik_params],
        output     = 'screen',
        condition  = IfCondition(use_ik),
    )

    # NODO 3 — Impedance Controller  (t = 0s)
    # Safe startup interno di 3s (pubblica zero torques, torque_controller
    # ancora inattivo → nessun effetto). Poi entra in standby finché la FSM
    # non pubblica /impedance_enable=True (dopo lo switch JTC→torque).
    # Quando disabilitato: traccia x_desired = x_current → ripartenza sicura.
    impedance_node = Node(
        package    = 'z1_vision',
        executable = 'impedance_controller_realsense',
        name       = 'impedance_controller_realsense',
        parameters = [impedance_params],
        output     = 'screen',
        condition  = IfCondition(use_impedance),
    )

    # NODO 4 — FSM  (t = fsm_delay secondi, default 5s)
    # Il delay garantisce che impedance controller abbia completato il
    # safe startup (3s) prima che la FSM tenti di attivarlo.
    # Primo stato: HOMING → porta il braccio in home_position → WAITING.
    fsm_node = Node(
        package    = 'z1_vision',
        executable = 'z1_FSM',
        name       = 'z1_FSM',
        parameters = [fsm_params],
        output     = 'screen',
        condition  = IfCondition(use_fsm),
    )

    return LaunchDescription([
        # Argomenti
        fsm_delay_arg,
        use_ik_arg,
        use_impedance_arg,
        use_fsm_arg,

        # t = 0s: switch + IK + impedance partono subito
        switch_node,
        ik_to_jtc_node,
        impedance_node,

        # t = fsm_delay: FSM parte dopo il delay
        TimerAction(
            period  = fsm_delay,
            actions = [fsm_node],
        ),
    ])
