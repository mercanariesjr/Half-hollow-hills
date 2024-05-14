// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PositionConstants;
import frc.ballistics.BallisticsCalculator;
import frc.ballistics.Note;
import frc.robot.Constants.ControlConstants;
import frc.robot.commands.NamedWait;
import frc.robot.commands.auto.AlignCommand;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.LobAlign;
import frc.robot.commands.auto.NoteLock;
import frc.robot.states.aim.AimState;
import frc.robot.states.aim.AimStateManager;
import frc.robot.states.global.RobotState;
import frc.robot.states.global.RobotStateManager;
import frc.robot.subsystems.ShooterLED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemIOSparkMax;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSparkMax;
import frc.robot.subsystems.feeder.Feeder.FeederState;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final CommandJoystick operatorJoystick
    = new CommandJoystick(OperatorConstants.kOperartorControllerPort);
  private final CommandGenericHID commandGeneric = new CommandGenericHID(0);

  private final DigitalInput dio;
  
  private DriveSubsystem driveSubsystem;
  private Arm arm;
  private Flywheel flywheel;
  private Feeder feeder;
  private Climb climb;

  private VisionSubsystem visionSubsystem;

  private SendableChooser<String> autoChooser = new SendableChooser<String>();

  public RobotContainer() {

    driveSubsystem = new DriveSubsystem(new DriveSubsystemIOSparkMax());
    if(Robot.isReal()) {
      arm = new Arm(new ArmIOSparkMax());
      flywheel = new Flywheel(new FlywheelIOSparkMax());
    } else {
      arm = new Arm(new ArmIOSim());
      flywheel = new Flywheel(new FlywheelIOSim());
    }

    feeder = new Feeder(new FeederIOSparkMax());
    climb = new Climb();
    visionSubsystem = new VisionSubsystem();

    String[] autos = new String[]{
      "4 Note Middle",
      "Blue 4 Note",

      "Source Side",
      "CenterSub",
      "AmpSub",
      "SourceSub",
      "keebler"
    };

    autoChooser.setDefaultOption("Nothing", "Nothing");
    for(String auto : autos) {
      autoChooser.addOption(auto, auto);
    }

    SmartDashboard.putData("Autonomous Chooser", autoChooser);


    dio = new DigitalInput(3);

    configureBindings();
    Autos.constructAutoBuilder(driveSubsystem);

    driveSubsystem.setPose(0, 0, 0);

    // Interpolation Sliders
    SmartDashboard.putNumber("Flywheel RPM", 0);
    SmartDashboard.putNumber("Arm Angle", 0);

    // SOTF Sliders
    SmartDashboard.putNumber("Note Velocity", 1.0);
    SmartDashboard.putNumber("Lob RPM", 3000.0);
    SmartDashboard.putNumber("Lob Angle", 25.0);
    SmartDashboard.putBoolean("SOTF", false);
    SmartDashboard.putBoolean("Strict Shooting", true);
    
    // Initialize LED Subsystem
    ShooterLED.getInstance();


    SmartDashboard.putNumber("Shooting Distance Offset", 0.173); // NYRO value (its 0 in the library)

    // This command makes some autos easier to work out with other teams
    SmartDashboard.putNumber("Named Wait Duration", 0.0);
    NamedCommands.registerCommand("Named Wait", new NamedWait());
  }



  private void configureBindings() {

    SmartDashboard.putData(CommandScheduler.getInstance());

    // NUCLEAR BUTTON
    SmartDashboard.putData("Nuclear Button", Commands.runOnce(() -> {
      CommandScheduler.getInstance().cancelAll();
    }));

    // Position resets
    SmartDashboard.putData("Blue Subwoofer Reset", Commands.runOnce(() -> {
      driveSubsystem.setPose(new Pose2d(new Translation2d(1.35, 5.5), Rotation2d.fromDegrees(0)));
    }));
    
    SmartDashboard.putData("Red Subwoofer Reset", Commands.runOnce(() -> {
      driveSubsystem.setPose(new Pose2d(GeometryUtil.flipFieldPosition(new Translation2d(1.35, 5.5)), Rotation2d.fromDegrees(180)));
    }));

    commandGeneric.button(9).onTrue(Commands.runOnce(() -> {
      driveSubsystem.resetGyroOffset();
    }));

    // Coast Button
    new Trigger(() -> {return dio.get();}).onTrue(Commands.runOnce(() -> {
      arm.setCoast(!dio.get());
      feeder.setCoast(!dio.get());
      ShooterLED.getInstance().coastbutton = false;
    }).ignoringDisable(true)).onFalse(Commands.runOnce(() -> {
      arm.setCoast(!dio.get());
      feeder.setCoast(!dio.get());
      ShooterLED.getInstance().coastbutton = true;
    }).ignoringDisable(true));

    // Driving

    driveSubsystem.setDefaultCommand(driveSubsystem.teleopCommand().withName("Teleop Command"));

    // Intaking

    // Yaw Note Lock
    operatorJoystick.button(XboxController.Button.kA.value).whileTrue(
      new NoteLock(driveSubsystem, visionSubsystem, RobotContainer::isRed,
      0.1, 0.0, 0.0)
    );

    // Manual Arm
    arm.setDefaultCommand(Commands.run(() -> {
      arm.setInput(MathUtil.applyDeadband(operatorJoystick.getRawAxis(0), 0.1));
    }, arm).withName("Arm Manual Control"));


    // Intake
    NamedCommands.registerCommand("Intake", intakeFactory());
    operatorJoystick.button(2).onTrue(intakeFactory());

    new Trigger(() -> { return feeder.getBottom(); }).and(()-> { return RobotStateManager.is(RobotState.INTAKE); }).onTrue(arm.setpointFactory(10));
    new Trigger(() -> { return feeder.getTop(); }).and(()-> { return RobotStateManager.is(RobotState.INTAKE); }).onTrue(Commands.sequence(
      feeder.setStateFactory(FeederState.STOP),
      Commands.waitSeconds(0.75),
      feeder.setpointIncCommand(0.75),
      feeder.setStateFactory(FeederState.PID),
      RobotStateManager.setStateFactory(RobotState.IDLE)
    ));

    // Aim
    NamedCommands.registerCommand("Aim", aimFactory());
    operatorJoystick.button(XboxController.Button.kX.value).onTrue(aimFactory()); // button #3

    operatorJoystick.button(XboxController.Button.kX.value).and(new Trigger(() -> { return AimStateManager.is(AimState.SPEAKER); })).whileTrue(yawFactory());
    operatorJoystick.button(XboxController.Button.kX.value).and(new Trigger(() -> { return AimStateManager.is(AimState.LOB); })).whileTrue(lobFactory());

    // Shooting
    NamedCommands.registerCommand("Shoot", shootFactory());
    commandGeneric.button(10).onTrue(shootFactory());
    operatorJoystick.button(16).onTrue(shootFactory());

    new Trigger(() -> {
      return ((Math.abs(arm.getSetpoint() - arm.getAngle())  < 1.0) && flywheel.atSetpoint());
    }).and(() -> {
      return RobotStateManager.is(RobotState.SHOOT);
    }).onTrue(Commands.sequence(
      feeder.setStateFactory(FeederState.FORWARD),
      this.simulateNote(),
      Commands.waitSeconds(0.2),
      feeder.setStateFactory(FeederState.STOP),
      Commands.runOnce(() -> {
        flywheel.setRPM(ControlConstants.kFlywheelIdle);
        RobotStateManager.setState(RobotState.IDLE);
      }),
      Commands.waitSeconds(0.04),
      Commands.runOnce(() -> {
        intakeFactory().schedule();
      })
    ).withName("Real Shoot Handler"));

    // Mode Switching
    operatorJoystick.pov(0).onTrue(AimStateManager.setStateFactory(AimState.SPEAKER));
    operatorJoystick.pov(180).onTrue(AimStateManager.setStateFactory(AimState.AMP));
    operatorJoystick.pov(90).or(operatorJoystick.pov(270)).onTrue(AimStateManager.setStateFactory(AimState.LOB));

    // Climbing
    operatorJoystick.button(XboxController.Button.kLeftBumper.value).onTrue(climb.raise());
    operatorJoystick.button(XboxController.Button.kRightBumper.value).onTrue(climb.lower());

    new Trigger(() -> {
     return DriverStation.isTeleopEnabled(); 
    }).onTrue(intakeFactory());

    if(Robot.isSimulation()) {
      SmartDashboard.putNumber("Time Factor", 1.0);
      configureSimDebug();
    }
  }

  public Command getAutonomousCommand() {
    System.out.println("Auto Selected: " + autoChooser.getSelected());
    driveSubsystem.driveChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    if(autoChooser.getSelected().equalsIgnoreCase("Quasistatic Forward")) {
      return flywheel.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    } else if(autoChooser.getSelected().equalsIgnoreCase("Quasistatic Backward")) {
      return flywheel.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    } else if(autoChooser.getSelected().equalsIgnoreCase("Dynamic Forward")) {
      return flywheel.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    } else if(autoChooser.getSelected().equalsIgnoreCase("Dynamic Backward")) {
      return flywheel.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    } else if(autoChooser.getSelected().equalsIgnoreCase("Nothing")) {
      return Commands.none();
    } else {  
      return new PathPlannerAuto(autoChooser.getSelected());
      // return AutoBuilder.buildAuto(autoChooser.getSelected()).andThen(Commands.runOnce(() -> { driveSubsystem.arcadeDrive(0, 0, 0); }));
    }
  }

  /**
   * Defaults to blue
   * @return Alliance
   */
  public static Alliance getAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if(alliance.isPresent()) {
      return alliance.get();
    }
    else return Alliance.Blue;
  }

  public static boolean isRed() {
    return getAlliance() == Alliance.Red;
  }

  public static boolean isBlue() {
    return getAlliance() == Alliance.Blue;
  }

  public static boolean isInvalid() {
    return DriverStation.getAlliance().isEmpty();
  }

  public Command intakeFactory() {
    return Commands.parallel(
      RobotStateManager.setStateFactory(RobotState.INTAKE),
      feeder.setStateFactory(FeederState.FORWARD),
      arm.setpointFactory(-1)
    ).withName("Intake Command");
  }

  public Command aimFactory() {
    return Commands.sequence(
      RobotStateManager.setStateFactory(RobotState.AIM),
      Commands.sequence(
        Commands.runOnce(() -> { driveSubsystem.setVisionOverride(true); }),
        aimLoopFactory()
      ).finallyDo(() -> {
        driveSubsystem.setVisionOverride(false);
        RobotStateManager.setState(RobotState.IDLE);
      }).onlyWhile(() -> { return RobotStateManager.is(RobotState.AIM) || RobotStateManager.is(RobotState.SHOOT); })).withName("Aim Command");
  }

  public Command aimLoopFactory() {
    return Commands.run(() -> {

        switch(AimStateManager.getState()) {
          case SPEAKER:
            Translation2d speaker = PositionConstants.kSpeakerPosition.plus(new Translation2d(0.5, 0));
            Translation2d robot = driveSubsystem.getPose().getTranslation();
            if(isRed()) speaker = GeometryUtil.flipFieldPosition(speaker);

            double dist = robot.getDistance(speaker);

            // SOTF
            if(SmartDashboard.getBoolean("SOTF", false)) {}

            dist += SmartDashboard.getNumber("Shooting Distance Offset", 0.0); // Account for field inaccuracy (get number during calibration period)
            flywheel.setRPM(Interpolation.getRPM(dist));
            arm.setSetpoint(Interpolation.getAngle(dist));
            break;
          case AMP:
            arm.setSetpoint(100);
            flywheel.setRPM(1000);
            break;
          case LOB:
            arm.setSetpoint(SmartDashboard.getNumber("Lob Angle", 25));
            flywheel.setRPM(SmartDashboard.getNumber("Lob RPM", 3000));
            break; 
        }

      }, arm, flywheel).withName("Aim Loop");
  }

  public Command shootFactory() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        if(RobotStateManager.is(RobotState.IDLE)) {
          aimFactory().schedule();
        }
      }),
      Commands.waitSeconds(0.04),
      RobotStateManager.setStateFactory(RobotState.SHOOT)
    ).withName("Shoot Command");
  }

  public Command yawFactory() {
    return new AlignCommand(driveSubsystem, visionSubsystem, RobotContainer::isRed, ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD);
  }
  public Command lobFactory() {
    return new LobAlign(driveSubsystem, visionSubsystem, RobotContainer::isRed, ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD);
  }

  public void configureSimDebug() {
    Commands.run(() -> {
        Note.handleNotes(0.02);
    }).ignoringDisable(true).withName("Ballistics Simulator").schedule();

    // Simulation Shooting Debug
    operatorJoystick.button(15).onTrue(Commands.sequence(
      feeder.setStateFactory(FeederState.FORWARD),
      this.simulateNote(),
      Commands.waitSeconds(0.2),
      feeder.setStateFactory(FeederState.STOP),
      Commands.runOnce(() -> {
        flywheel.setRPM(ControlConstants.kFlywheelIdle);
        RobotStateManager.setState(RobotState.IDLE);
      }),
      Commands.waitSeconds(0.04),
      Commands.runOnce(() -> {
        intakeFactory().schedule();
      })
    ).withName("Simulated Shoot Handler (No Aim)"));

    operatorJoystick.button(14).onTrue(Commands.runOnce(() -> {
      SmartDashboard.putBoolean("Top Switch Sim", true);
    }).withName("Simulation Note Intake Button"));

    operatorJoystick.button(14).onFalse(Commands.runOnce(() -> {
      SmartDashboard.putBoolean("Top Switch Sim", false);
    }).withName("Simulation Note Intake Button"));
  }

  public Command simulateNote() {
    return Commands.runOnce(() -> {
        double angle = arm.getAngle();
        Pose2d drivePose = driveSubsystem.getPose();
        new Note(Math.toRadians(angle) + Math.toRadians(113.6), drivePose.getRotation().getRadians(),
        8.0, // shot velocity (needs to be interpolated)
        drivePose.getX() + BallisticsCalculator.thetaHoodAngleToX(angle, drivePose.getRotation().getRadians()),
        drivePose.getY() + BallisticsCalculator.thetaHoodAngleToY(angle, drivePose.getRotation().getRadians()),
        BallisticsCalculator.hoodAngleToZ(Math.toRadians(angle)),
        0, 0);
      });
  }
}