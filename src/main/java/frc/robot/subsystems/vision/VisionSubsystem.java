// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonVersion;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/** Add your docs here. */
public class VisionSubsystem extends SubsystemBase {

  PhotonCamera frontCamera;
  PhotonCamera backCamera;
  PhotonCamera noteCamera;
  PhotonCamera[] poseCameras;

  PhotonCameraSim frontCameraSim;
  PhotonCameraSim backCameraSim;
  PhotonCameraSim intakeCameraSim;

  PhotonPoseEstimator frontPoseEstimator;
  PhotonPoseEstimator backPoseEstimator;
  PhotonPoseEstimator[] poseEstimators;

  PhotonPipelineResult result;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget bestTarget;

  Optional<PhotonTrackedTarget> note = Optional.empty();

  VisionSystemSim sim;

  public static Optional<EstimatedRobotPose> estimatedPoseFront = Optional.empty();
  public static Optional<EstimatedRobotPose> estimatedPoseBack = Optional.empty();
  private static Optional<EstimatedRobotPose>[] poseOptionals = new Optional[]{estimatedPoseFront, estimatedPoseBack};

  Transform3d robotToFrontCamera;
  Transform3d robotToBackCamera;
  Transform3d[] robotToCameras;

  boolean rotationOverride = false;

  public static double MIN_TARGET_SIZE = 0.06;

  AprilTagFieldLayout fieldLayout;

  public VisionSubsystem() {
    Logger.recordOutput("PV", PhotonVersion.versionString);
    robotToFrontCamera = new Transform3d(-0.225948,
      Units.inchesToMeters(0.125), 0.451592, new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(0)));

    robotToBackCamera = new Transform3d(-0.278422,
      Units.inchesToMeters(0.125), 0.450133, new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(180)));

    robotToCameras = new Transform3d[]{robotToFrontCamera, robotToBackCamera};

    frontCamera = new PhotonCamera("Front");
    backCamera = new PhotonCamera("Back");
    noteCamera = new PhotonCamera("Intake");
    poseCameras = new PhotonCamera[]{frontCamera, backCamera};

    if(Robot.isSimulation()) {
      sim = new VisionSystemSim("main");

      SimCameraProperties properties = new SimCameraProperties();
      properties.setCalibration(800, 600, Rotation2d.fromDegrees(70));

      frontCameraSim = new PhotonCameraSim(frontCamera, properties);
      backCameraSim = new PhotonCameraSim(backCamera, properties);
      intakeCameraSim = new PhotonCameraSim(noteCamera, properties);
  
      
      frontCameraSim.enableDrawWireframe(true);
      backCameraSim.enableDrawWireframe(true);


      sim.addCamera(frontCameraSim, robotToFrontCamera);
      sim.addCamera(backCameraSim, robotToBackCamera);
      sim.addCamera(intakeCameraSim, new Transform3d());
    }

    try {
      fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      frontPoseEstimator = new PhotonPoseEstimator(
              fieldLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              frontCamera,
              robotToFrontCamera);
      frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      backPoseEstimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              backCamera,
              robotToBackCamera);
      backPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      poseEstimators = new PhotonPoseEstimator[]{frontPoseEstimator, backPoseEstimator};

      sim.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
    } catch (Exception e) {
      e.printStackTrace();
    }

    PPHolonomicDriveController.setRotationTargetOverride(() -> {
      if(rotationOverride) return Optional.of(getNoteRotation());
      return Optional.empty();
    });

    NamedCommands.registerCommand("Note Lock", Commands.runOnce(() -> {
      rotationOverride = true;
    }));

    NamedCommands.registerCommand("Note Unlock", Commands.runOnce(() -> {
      rotationOverride = false;
    }));
  }

  @Override
  public void periodic() {

    for(int i = 0; i < poseCameras.length; i++) {
      boolean pnp = true;
      if(!poseCameras[i].getLatestResult().hasTargets()) continue;
      var list = poseCameras[i].getLatestResult().getTargets();
      for(PhotonTrackedTarget target : list) {
        if(target.getArea() < MIN_TARGET_SIZE) { // If any target is under minimum target size, switch to single target pose estimation
          pnp = false;
          Logger.recordOutput("Vision/" + i + "/PNP", false);
          // Single target pose estimation
          var bestTarget = poseCameras[i].getLatestResult().getBestTarget();
          var tagPoseOpt = fieldLayout.getTagPose(bestTarget.getFiducialId());
          if(tagPoseOpt.isPresent()) {
            var tagPose = tagPoseOpt.get();
            poseOptionals[i] = Optional.of(new EstimatedRobotPose(tagPose.transformBy(bestTarget.getBestCameraToTarget().inverse()).transformBy(robotToCameras[i].inverse()), poseCameras[i].getLatestResult().getTimestampSeconds(), List.of(bestTarget), null));
          }
        }
      }
      if(pnp) {
        Logger.recordOutput("Vision/" + i + "/PNP", true);
        poseOptionals[i] = poseEstimators[i].update();
      }
    }


    estimatedPoseFront = poseOptionals[0];
    estimatedPoseBack = poseOptionals[1];

    // estimatedPoseFront = frontPoseEstimator.update();
    // estimatedPoseBack = backPoseEstimator.update();

    result = noteCamera.getLatestResult();


    if(result.hasTargets()) note = Optional.of(result.getBestTarget());
    else note = Optional.empty();

    if(estimatedPoseFront.isPresent()) {
      Logger.recordOutput("Vision/0/EstimatedPoseFront", estimatedPoseFront.get().estimatedPose.toPose2d());
    }

    if(estimatedPoseBack.isPresent()) {
      Logger.recordOutput("Vision/1/EstimatedPoseBack", estimatedPoseBack.get().estimatedPose.toPose2d());
    }

  }

  @Override
  public void simulationPeriodic() {
    sim.update(DriveSubsystem.robotPose);
  }

  public List<PhotonTrackedTarget> getTargets() {
    List<PhotonTrackedTarget> list = frontCamera.getLatestResult().getTargets();
    list.addAll(backCamera.getLatestResult().getTargets());
    return list;
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    return note;
  }

  public boolean hasTargets() {
    return backCamera.getLatestResult().hasTargets() || frontCamera.getLatestResult().hasTargets();
  }

  public Optional<PhotonTrackedTarget> getTag(int id) {
    for(PhotonTrackedTarget target : getTargets()) {
      if(target.getFiducialId() == id) {
        return Optional.of(target);
      }
    }
    return Optional.empty();
  }

  public double distanceToTarget(PhotonTrackedTarget target) {
    return Math.sqrt(Math.pow(target.getBestCameraToTarget().getX(), 2) + Math.pow(target.getBestCameraToTarget().getY(), 2)); 
  }

  public void setRotationTargetOverride(boolean val) {
    rotationOverride = val;
  }

  public boolean getRotationTargetOVerride() {
    return rotationOverride;
  }

  public Rotation2d getNoteRotation() {
    if(getBestTarget().isEmpty()) return DriveSubsystem.robotPose.getRotation();
    return DriveSubsystem.robotPose.getRotation().plus(Rotation2d.fromDegrees(getBestTarget().get().getYaw()));
  }
}
  