package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterLED;

public class Climb extends SubsystemBase {
    DoubleSolenoid solenoid;

    public Climb() {
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void set(boolean on) {
        solenoid.set(on ? Value.kForward : Value.kReverse);
    }

    public boolean get() {
        if(solenoid.get().equals(Value.kForward)) return true;
        return false;
    }

    public boolean toggle() {
        set(!get());
        return get();
    }

    public Command raise() {
        return Commands.runOnce(() -> {
            set(false);
            ShooterLED.getInstance().pistonClimb = true;
        });
    }

    public Command lower() {
        return Commands.runOnce(() -> {
            set(true);
            ShooterLED.getInstance().pistonClimb = false;
        });
    }
    
}
