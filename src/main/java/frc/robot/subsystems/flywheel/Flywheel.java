// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Minutes;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.Constants.ControlConstants;

public class Flywheel extends SubsystemBase {
    
    FlywheelIO io;
    FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    double setpoint;

    PIDController topController;
    PIDController bottomController;

    SimpleMotorFeedforward feedforward;
    SimpleMotorFeedforward feedforwardBottom;

    GenericHID driver;

    double rpm = 0;

    public SysIdRoutine sysIdRoutine;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        topController = new PIDController(
            0.0016201,
            0,
            0);
        bottomController = new PIDController(
            0.0015057,
            0,
            0);

        feedforward = new SimpleMotorFeedforward(0, 0.0021186);
        feedforwardBottom = new SimpleMotorFeedforward(0, 0.0021371);

        driver = new GenericHID(0);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                    io.setTopVoltage(volts.in(Volts));
                    io.setBottomVoltage(volts.in(Volts));
                },
                log -> {
                log.motor("flywheel-top").voltage(Volts.of(inputs.topVoltage))
                    .angularPosition(Rotations.of(inputs.topPosition))
                    .angularVelocity(Rotations.of(inputs.topVelocity).per(Minutes.of(1)));

                log.motor("flywheel-bottom").voltage(Volts.of(inputs.bottomVoltage))
                    .angularPosition(Rotations.of(inputs.bottomPosition))
                    .angularVelocity(Rotations.of(inputs.bottomVelocity).per(Minutes.of(1)));
        }, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Subsystem/Flywheel", inputs);
        Logger.recordOutput("Subsystem/Flywheel/Top Setpoint Reached", topController.atSetpoint());
        Logger.recordOutput("Subsystem/Flywheel/Bottom Setpoint Reached", bottomController.atSetpoint());
        Logger.recordOutput("Subsystem/Flywheel/atSetpoint", this.atSetpoint());
        
        if(Robot.isSimulation()) {
            io.updateSimulation();
        }

        topController.setSetpoint(rpm);
        bottomController.setSetpoint(rpm);
        
        if(ControlConstants.kFlywheelPID) {
            double topOutput = topController.calculate(inputs.topVelocity);
            double bottomOutput = bottomController.calculate(inputs.bottomVelocity);

            if(topOutput < -3) topOutput = -3;
            if(bottomOutput < -3) bottomOutput = -3;

            Logger.recordOutput("Subsystem/Flywheel/TopOutput", topOutput + feedforward.calculate(rpm));
            Logger.recordOutput("Subsystem/Flywheel/BottomOutput", bottomOutput + feedforward.calculate(rpm));
            io.setTopVoltage(topOutput + feedforward.calculate(rpm));
            io.setBottomVoltage(bottomOutput + feedforwardBottom.calculate(rpm));
        }
    }

    public void setSpeed(double speed) {
        setTopSpeed(speed);
        setBottomSpeed(speed);
    }

    public void setTopSpeed(double speed) {
        io.setTopSpeed(speed);
    }

    public void setBottomSpeed(double speed) {
        io.setBottomSpeed(speed);
    }

    public void setRPM(double rpm) {
        this.rpm = rpm;
        Logger.recordOutput("Subsystem/Flywheel/Setpoint", this.rpm);
    }

    public double getVelocity() {
        return io.getVelocity();
    }

    public boolean atSetpoint() {
        return (Math.abs(
            inputs.topVelocity - rpm)
                < 100.0) &&
            (Math.abs(
                inputs.bottomVelocity - rpm)
                < 100.0);
    }
}
