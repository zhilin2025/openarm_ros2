### 一个硬件接口插件和一个控制器
ZeroTorqueController 决定策略（重力补偿），OpenArm_v10HW 负责执行（CAN 发包 + 限位兜底 + 阻尼配置）
ZeroTorqueController用来在切换到effort模式时，由controller_manager spawner加载，用Pinocchio计算重力补偿力矩。数据流方向：读state → 算力矩 → 写command 
OpenArm_v10HW是ros2_control加载的硬件接口插件，随硬件始终运行，数据流方向：读command → CAN→电机

### 切换零力矩模式时的完整时序：
1. 用户调用 controller_manager 服务：
   ros2 control switch_controllers --activate zero_torque_controller
                                  --deactivate joint_trajectory_controller

2. controller_manager 调用 ZeroTorqueController::on_activate()
   → 日志: "Zero-torque mode ACTIVATED (controller_kd=0.300, gravity_scale=1.00)"

3. controller_manager 检测到 effort 接口被激活
   → 调用 OpenArm_v10HW::prepare_command_mode_switch()
   → 调用 OpenArm_v10HW::perform_command_mode_switch()
   → effort_mode_ = true
   → 日志: "Switched to effort mode (Kp=0, Kd=0.300)"

4. 之后的每个控制周期 (100Hz):
   ┌─ read(): 从 CAN 总线读取电机实际位置 → pos_states_[i]
   │
   ├─ ZeroTorqueController::update(): 
   │  读取 pos_states_ → Pinocchio RNEA → 重力力矩 → 写入 tau_commands_
   │
   └─ write():
       cmd_kp = 0.0         ← effort_mode_=true
       cmd_kd = 0.3         ← zero_torque_kd_
       cmd_pos = pos_states ← 当前位置（hold）
       cmd_tau = tau_commands_[i] ← 重力补偿力矩（由控制器写入）
       → CAN发送: {Kp=0, Kd=0.3, pos=当前位置, tau=重力力矩}

### 力矩补偿的两个地方实现 
   → motion模式下 controllers 是 JTC + forward_effort，teach模式下 controllers 是 zero_torque
   → 注意：forward_effort_controller 是可选的，根据是否需要重力补偿添加。
   → 即motion模式下，由gravity_comp_node节点计算位置控制时所需的重力补偿前馈，发布到forward_effort_controller，然后到硬件层的tau_commands_，配合JTC的输入，最终形成完整MIT控制指令
   → 然后切换到teach模式下的时候，停掉了JTC和FEC控制器，由zero_torque_controller插件直接做示教模式下的完整力矩计算，发布到tau_commands_，然后配合其他预设参数，最终形成完整MIT控制指令(详情见 v10_simple_hardware.cpp的写入模块)
