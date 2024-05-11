package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.HardwareConstants;

public class FeederIOSparkMax implements FeederIO {
    CANSparkMax feeder;
    RelativeEncoder encoder;

    DigitalInput topSwitch;
    DigitalInput bottomSwitch;

    public FeederIOSparkMax() {
        feeder = new CANSparkMax(HardwareConstants.kFeeder, MotorType.kBrushless);
        encoder = feeder.getEncoder();

        topSwitch = new DigitalInput(HardwareConstants.kFeederTopSensor);
        bottomSwitch = new DigitalInput(HardwareConstants.kFeederBottomSensor);

        feeder.restoreFactoryDefaults();
        feeder.setSmartCurrentLimit(20);
        encoder.setPositionConversionFactor(1.0/16.0);
        feeder.setOpenLoopRampRate(0);
        feeder.setClosedLoopRampRate(0);

        feeder.setInverted(true);

        if(Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(feeder, DCMotor.getNeo550(1));
            SmartDashboard.putBoolean("Top Switch Sim", false);
            SmartDashboard.putBoolean("Bottom Switch Sim", false);
        }
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.voltage = feeder.getAppliedOutput() * feeder.getBusVoltage();
        inputs.value = feeder.get();
        inputs.current = feeder.getOutputCurrent();
        inputs.rpm = encoder.getVelocity();
        inputs.position = encoder.getPosition();
        inputs.topSensor = !topSwitch.get();
        inputs.bottomSensor = !bottomSwitch.get();

        if(Robot.isSimulation()) {
            inputs.topSensor = SmartDashboard.getBoolean("Top Switch Sim", false);
            inputs.bottomSensor = SmartDashboard.getBoolean("Bottom Switch Sim", false);
        }
    }

    @Override
    public void setVoltage(double volts) {
        if(volts < -12) volts = -12;
        if(12 < volts) volts = 12;
        feeder.setVoltage(volts);
    }

    @Override
    public void set(double value) {
        feeder.set(value);
    }

    @Override
    public void setCoast(boolean coast) {
        feeder.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
    }
}
