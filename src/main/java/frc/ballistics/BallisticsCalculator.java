// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.ballistics;

/** Add your docs here. */
/**
 * @param vel Starting velocity of note (guess or interpolate lol)
 * @param t Time since shot
 * @param hoodangle Angle of shot (rad)
 * @param theta Facing of shot (rad)
 */
public class BallisticsCalculator {
    public static double G = 9.81;

    public static double x(double vel, double t, double hoodangle, double theta) {
        return dist(vel, t, hoodangle) * Math.cos(theta);
    }

    public static double y(double vel, double t, double hoodangle, double theta) {
        return dist(vel, t, hoodangle) * Math.sin(theta);
    }

    public static double z(double vel, double t, double hoodangle) {
        return vel * Math.sin(hoodangle) * t - (0.5 * G * Math.pow(t, 2));
    }

    public static double dist(double vel, double t, double hoodangle) {
        return vel * Math.cos(hoodangle) * t;
    }
    
    // return Math.tan(hoodangle) * t - ((G) / (2 * Math.pow(vel, 2) * Math.pow(Math.cos(hoodangle), 2))) * Math.pow(t, 2);

}
