// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PIDControlConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.PIDAlign;
import frc.robot.commands.auto.SpeakerAlign;
import frc.robot.commands.auto.SpeakerAlignCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystemIOSparkMax;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOInputsAutoLogged;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final CommandXboxController driverController
    = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick operatorJoystick
    = new CommandJoystick(OperatorConstants.kOperartorControllerPort);
  private DriveSubsystem driveSubsystem;
  private Arm arm;

  private VisionSubsystem visionSubsystem;

  private TeleopCommand teleopCommand;

  private SendableChooser<Integer> driveChooser = new SendableChooser<Integer>();
  private SendableChooser<Integer> tempAutoChooser = new SendableChooser<Integer>();

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem(new DriveSubsystemIOSparkMax());
    if(Robot.isReal()) arm = new Arm(new ArmIOSparkMax());
    else arm = new Arm(new ArmIOSim());
    visionSubsystem = new VisionSubsystem();

    Autos.constructAutoBuilder(driveSubsystem);

    teleopCommand = new TeleopCommand(driveSubsystem);

    driveChooser.setDefaultOption("Arcade", 0);

    tempAutoChooser.setDefaultOption("Nothing", 0);
    tempAutoChooser.setDefaultOption("Test Trajectory Auto", 1);

    configureBindings();

    driveSubsystem.setPose(0, 0, 0);
  }

  private void configureBindings() {
    // SmartDashboard.putData("DriveChooser", driveChooser);
    driveSubsystem.setDefaultCommand(teleopCommand);

    buttonTriggerPIDAlign(driverController.a(), PositionConstants.kAmpPose, RobotContainer::isRed,
    PIDControlConstants.kP, PIDControlConstants.kI, PIDControlConstants.kD,
    PIDControlConstants.kAP, PIDControlConstants.kAI, PIDControlConstants.kAD
    );
    
    // driverController.a().whileTrue(
    //   new PIDAlign(
    //     driveSubsystem,
    //     PositionConstants.kAmpPose, RobotContainer::isRed,
    //     10.0, 0, 0.0,
    //     10.0, 0, 0.0
    //   )
    // );

    driverController.x().whileTrue(
      new PIDAlign(
        driveSubsystem,
        PositionConstants.kSource1Pose, RobotContainer::isBlue,
        PIDControlConstants.kP, PIDControlConstants.kI, PIDControlConstants.kD,
        PIDControlConstants.kAP, PIDControlConstants.kAI, PIDControlConstants.kAD
      )
    );

    driverController.y().whileTrue(
      new PIDAlign(
        driveSubsystem,
        PositionConstants.kSource2Pose, RobotContainer::isBlue,
        PIDControlConstants.kP, PIDControlConstants.kI, PIDControlConstants.kD,
        PIDControlConstants.kAP, PIDControlConstants.kAI, PIDControlConstants.kAD
      )
    );

    driverController.b().whileTrue(
      new PIDAlign(
        driveSubsystem,
        PositionConstants.kSource3Pose, RobotContainer::isBlue,
        PIDControlConstants.kP, PIDControlConstants.kI, PIDControlConstants.kD,
        PIDControlConstants.kAP, PIDControlConstants.kAI, PIDControlConstants.kAD
      )
    );

    driverController.rightBumper().whileTrue(
      new SpeakerAlign(
        driveSubsystem, visionSubsystem, RobotContainer::isRed,
        PIDControlConstants.kAP, PIDControlConstants.kAI, PIDControlConstants.kAD
      )
    );

    // Reset Gyro
    driverController.pov(0).onTrue(new InstantCommand(new Runnable() {
      @Override
      public void run() {
        driveSubsystem.setPose(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY(), 0);
      }
    }) );

    new Trigger(() -> {
      return Math.abs(operatorJoystick.getRawAxis(0)) > 0.1;
    }).whileTrue(Commands.run(new Runnable() {
      @Override
      public void run() {
        arm.setInput(operatorJoystick.getRawAxis(0));
      }
    }, arm)).onFalse(Commands.runOnce(new Runnable() {
      @Override
      public void run() {
        arm.setInput(0);
      }
    }));

  }

  public Command getAutonomousCommand() {
    return Autos.B1R3_2Cube();
  }

  public int getDriveConfig() {
    return (int) driveChooser.getSelected();
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

  public void buttonTriggerPIDAlign(Trigger trigger, Pose2d target, BooleanSupplier flip, double p, double i, double d, double ap, double ai, double ad) {
    trigger.whileTrue(new PIDAlign(driveSubsystem, target, flip, p, i, d, ap, ai, ad));
  }
}
