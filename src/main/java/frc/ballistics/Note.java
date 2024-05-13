// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.ballistics;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class Note {
    public static ArrayList<Note> notes = new ArrayList<Note>();

    private static int idTracker = 0;

    Pose3d lastPose;

    double angle;
    double theta;
    double shotvel;
    double x;
    double y;
    double z;
    double vx;
    double vy;

    double id;
    double time;
    /**
     * Object that handles ballistics for one note
     * @param angle Angle of shot
     * @param theta Heading
     * @param shotvel Velocity of note exiting from shooter
     * @param x X position when note was shot
     * @param y Y position when note was shot
     * @param z Z position when note was shot
     * @param vx Drivetrain x velocity
     * @param vx Drivetrainy velocity
     */
    public Note(double angle, double theta, double shotvel, double x, double y, double z, double vx, double vy) {
        this.angle = angle;
        this.theta = theta;
        this.shotvel = shotvel;
        this.x = x;
        this.y = y;
        this.z = z;
        this.vx = vx;
        this.vy = vy;

        this.id = idTracker++;
        this.time = System.currentTimeMillis();

        notes.add(this);
    }

    /**
     * 
     * @param t Time since shot
     * @return
     */
    public Pose3d getPose(double t) {
        return new Pose3d(BallisticsCalculator.x(shotvel, t, angle, theta) + x, BallisticsCalculator.y(shotvel, t, angle, theta) + y, BallisticsCalculator.z(shotvel, t, angle) + z, new Rotation3d());
    }

    public static void handleNotes() {
        double time = System.currentTimeMillis();
        Pose3d[] poses = new Pose3d[notes.size()];
        for(int i = 0; i < notes.size(); i++) {
            poses[i] = notes.get(i).getPose((time - notes.get(i).time) / 1000.0);
        }
        for(int i = notes.size()-1; 0 <= i; i--) {
            if(poses[i].getZ() <= -10) notes.remove(i);
        }
        Logger.recordOutput("Other/Notes", poses);
    }
}
