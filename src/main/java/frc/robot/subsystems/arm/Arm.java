package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ControlConstants;

public class Arm extends SubsystemBase {

    ProfiledPIDController profiledPIDController;
    PIDController pidController;
    Constraints profile;
    ArmFeedforward feedforward;

    Mechanism2d mechanism;
    MechanismLigament2d armLigament;
    MechanismLigament2d setpointLigament;

    ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    ArmIO io;

    double setpoint = 0;

    boolean manual = false;

    boolean stow = false;

    public Arm(ArmIO io) {
        this.io = io;

        pidController = new PIDController(
            ControlConstants.kArmP,
            ControlConstants.kArmI,
            ControlConstants.kArmD);

            profile = new TrapezoidProfile.Constraints(
                ControlConstants.kArmDegPerSec, ControlConstants.kArmAccel);

        profiledPIDController = new ProfiledPIDController(
            ControlConstants.kArmP,
            ControlConstants.kArmI,
            ControlConstants.kArmD, 
            new TrapezoidProfile.Constraints(
                ControlConstants.kArmDegPerSec, ControlConstants.kArmAccel));

        feedforward = new ArmFeedforward(0.0, 0.4, 0.0);

        mechanism = new Mechanism2d(0, 0);
        MechanismRoot2d root = mechanism.getRoot("shooter", -HardwareConstants.kYOriginToArm, HardwareConstants.kZOriginToArm);
        armLigament = root.append(new MechanismLigament2d("arm", HardwareConstants.kArmLength, 90));
        armLigament.setLineWeight(5);
        armLigament.setColor(new Color8Bit(128, 0, 128));
        setpointLigament = root.append(new MechanismLigament2d("armSetpoint", HardwareConstants.kArmLength, 90));
        setpointLigament.setLineWeight(2);
        setpointLigament.setColor(new Color8Bit(0, 255, 0));

        new Trigger(DriverStation::isDisabled).whileTrue(new InstantCommand(new Runnable() {
            @Override
            public void run() {
                setSetpoint(io.getAngle());
            }
        }).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystem/Arm", inputs);

        armLigament.setAngle(Math.toRadians(io.getAngle()) * (180.0 / Math.PI));
        setpointLigament.setAngle(Math.toRadians(setpoint) * (180.0 / Math.PI));
        Logger.recordOutput("Subsystem/Arm/Mechanism", mechanism);
        Logger.recordOutput("Subsystem/Arm/Setpoint", profiledPIDController.getSetpoint().position);
        Logger.recordOutput("Subsystem/Arm/Pose3d", new Pose3d(-HardwareConstants.kYOriginToArm, 0, HardwareConstants.kZOriginToArm, new Rotation3d((Math.PI / 2.0) - Rotation2d.fromDegrees(io.getAngle()).getRadians(), 0.0, (Math.PI / 2.0))));
        Logger.recordOutput("Subsystem/Arm/SetpointPose3d", new Pose3d(-HardwareConstants.kYOriginToArm, 0, HardwareConstants.kZOriginToArm, new Rotation3d((Math.PI / 2.0) - Math.toRadians(setpoint), 0.0, (Math.PI / 2.0))));

        if(ControlConstants.kArmPid) {
            double output = 0.0;
            if(stow) {
                if(!DriverStation.isDisabled())
                profiledPIDController.setGoal(10);
            } else {
                profiledPIDController.setGoal(setpoint);
            }
            output = -feedforward.calculate(
                    Units.degreesToRadians(profiledPIDController.getSetpoint().position), 0)
                - profiledPIDController.calculate(io.getAngle());


            Logger.recordOutput("Subsystem/Arm/output", output);
            setVoltage(output);
        }

        if(DriverStation.isDisabled()) {
            setSetpoint(getAngle());
        }
    }

    @Override
    public void simulationPeriodic() {
        io.updateSim();
    }
    
    public void setSetpoint(double angle) {
        setpoint = angle;
        
        if(setpoint < HardwareConstants.kArmRotPhysicalMin) {
            setpoint = HardwareConstants.kArmRotPhysicalMin;
        }
        if(HardwareConstants.kArmRotPhysicalMax < setpoint) {
            setpoint = HardwareConstants.kArmRotPhysicalMax;
        }
    }

    public void setMotor(double value) {
        setVoltage(value * RobotController.getBatteryVoltage());
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void adjustSetpoint(double adjustment) {
        setSetpoint(setpoint += (adjustment * (ControlConstants.kArmDegPerSec / 50)));
    }

    public void setInput(double input) {
        if(ControlConstants.kArmPid) {
            adjustSetpoint(input);
        } else {
            setMotor(input);
        }
    }

    public double getAngle() {
        return io.getAngle();
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getAngleRad() {
        return io.getAngle();
    }

    public void setManualControl(boolean manualControl) {
        manual = manualControl;
    }

    public void setStow(boolean stow) {
        this.stow = stow;
    }

    public void setCoast(boolean coast) {
        io.setCoast(coast);
    }

    public boolean getCoast(int motor) {
        return io.getCoast(motor);
    }

    public Command setpointFactory(double setpoint) {
        return Commands.runOnce(() -> {
            setSetpoint(setpoint);
        }, this);
    }
}
