package frc.robot.states.global;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotStateManager {
    private static RobotState state = RobotState.INTAKE;
    
    public static RobotState getState() { return state; }
    public static void setState(RobotState _state) { state = _state; Logger.recordOutput("Other/States/RobotState", state); }
    public static boolean is(RobotState _state) { return state.equals(_state); }
    public static Command setStateFactory(RobotState _state ) { return Commands.runOnce(() -> {setState(_state); }); }
}