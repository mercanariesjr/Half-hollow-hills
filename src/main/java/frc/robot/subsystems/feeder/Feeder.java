// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlConstants;

/** Add your docs here. */
public class Feeder extends SubsystemBase {

    FeederIO io;
    FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
    PIDController feederController;
    boolean pidControl = false;

    GenericHID driver = new GenericHID(0);

    FeederState state = FeederState.STOP;

    public Feeder(FeederIO io) {
        this.io = io;
        feederController = new PIDController(ControlConstants.kFeederP, 0, 0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
        Logger.recordOutput("Feeder/Setpoint", feederController.getSetpoint());
        
        switch(state) {
            case PID:
                double output = feederController.calculate(getPosition());
                io.setVoltage(output);
                break;
            case FORWARD:
                io.setVoltage(ControlConstants.kFeederMagnitude * RobotController.getBatteryVoltage());
                break;
            case BACKWARD:
                io.setVoltage(-ControlConstants.kFeederMagnitude * RobotController.getBatteryVoltage());
                break;
            case STOP:
                io.setVoltage(0);
                break;
        }
    }
    
    public void setSetpoint(double setpoint) {
        feederController.setSetpoint(setpoint);
    }

    public double getPosition() {
        return inputs.position;
    }

    public boolean getTop() {
        return inputs.topSensor;
    }

    public boolean getBottom() {
        return inputs.bottomSensor;
    }

    public boolean atSetpoint() {
        return feederController.atSetpoint();
    }

    public void setCoast(boolean coast) {
        io.setCoast(coast);
    }

    public FeederState getState() {
        return state;
    }

    public void setState(FeederState state) {
        this.state = state;
    }

    public enum FeederState {
        PID,
        FORWARD,
        BACKWARD,
        STOP
    }

    public Command setStateFactory(FeederState state) {
        return Commands.runOnce(() -> { this.setState(state); }, this);
    }

    public Command setpointCommand(double setpoint) {
        return Commands.runOnce(() -> { setSetpoint(setpoint); });
    }

    public Command setpointIncCommand(double setpointInc) {
        return Commands.runOnce(() -> { setSetpoint(this.getPosition() + setpointInc); });
    }
}
