package frc.robot.states.aim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AimStateManager {
    private static AimState state = AimState.SPEAKER;
    
    public static AimState getState() { return state; }
    public static void setState(AimState _state) { state = _state; Logger.recordOutput("Other/States/AimState", state); }
    public static boolean is(AimState _state) { return state.equals(_state); }
    public static Command setStateFactory(AimState _state ) { return Commands.runOnce(() -> {setState(_state); }); }
}