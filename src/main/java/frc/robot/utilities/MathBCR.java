// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

public class MathBCR {

  /**
   * Converts input angle to a number between -179.999 and +180.0.
   * @return Normalized angle, in degrees
   */
  public static double normalizeAngle(double angle) {
    angle = angle % 360;
    angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
    return angle;
  }

  /**
   * Translates a pose by a given X and Y offset.
   * [Xnew Ynew ThetaNew] = [Xinit Yinit ThetaInit] + [xOffset yOffset 0]
   * @param initialPose Pose2d to apply the translation to
   * @param xOffset X component of the translation
   * @param yOffset Y component of the translation
   * @return Nnew Pose2d based on the initial pose and translations
   */
  public static Pose2d translate(Pose2d initialPose, double xOffset, double yOffset) {
    return new Pose2d(initialPose.getTranslation().plus(new Translation2d(xOffset, yOffset)), initialPose.getRotation());
  }

  /**
   * Calculates the number of degrees between two normalized angles [-180, 180].
   * @param a Normalized angle a
   * @param b Normalized angle b
   * @return Difference between the angles, in degrees
   */
  public static double angleMinus(double a, double b) {
    double d = a - b;
    if (d > 180) d -= 360; 
    if (d < -180) d += 360;
    return d;
  }

  /**
   * Returns value clamped between low and high boundaries.
   *
   * @param value Value to clamp.
   * @param low The lower boundary to which to clamp value.
   * @param high The higher boundary to which to clamp value.
   * @return The clamped value.
   */
  public static int clamp(int value, int low, int high) {
    return Math.max(low, Math.min(value, high));
  }

  /**
   * Returns value clamped between low and high boundaries.
   *
   * @param value Value to clamp.
   * @param low The lower boundary to which to clamp value.
   * @param high The higher boundary to which to clamp value.
   * @return The clamped value.
   */
  public static double clamp(double value, double low, double high) {
    return Math.max(low, Math.min(value, high));
  }

}