/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.command2.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.MathBCR;

public class DriveResetPose extends Command {
  private final DriveTrain driveTrain;
  
  private final boolean fromElastic;
  private final boolean onlyAngle;      // true = resent angle but not X-Y position
  private final boolean usingLambda;
  private final boolean tolerance;      // true = do not reset if within 0.5m or 15 degrees of location, false = always reset
  private double curX, curY, curAngle;  // in meters and degrees
  private Supplier<Pose2d> poseSupplier;

  private TunableDouble resetXInput = Tunables.addDouble("DriveResetPose X", 0.0);
  private TunableDouble resetYInput = Tunables.addDouble("DriveResetPose Y", 0.0);
  private TunableDouble resetAngleInput = Tunables.addDouble("DriveResetPose Angle", 0.0);
  
  /**
	 * Resets the pose, gyro, and encoders on the driveTrain.
   * <p><b>NOTE:</b>  This command can run while the robot is disabled.
   * @param curXinMeters robot X location in the field, in meters (0 = field edge in front of the Blue driver station, positive = away from the Blue drivestation)
   * @param curYinMeters Robot Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, positive = left when looking from the Blue drivestation)
   * @param curAngleinDegrees robot angle on the field, in degrees (0 = facing away from the Blue drivestation, positive = to the left, negative = to the right)
   * @param tolerance true = do not reset if within 0.5m or 15 degrees of location, false = always reset
   * @param driveTrain DriveTrain subsystem
	 */
  public DriveResetPose(double curXinMeters, double curYinMeters, double curAngleinDegrees, boolean tolerance, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    curX = curXinMeters;
    curY = curYinMeters;
    curAngle = curAngleinDegrees;
    fromElastic = false;
    onlyAngle = false;
    usingLambda = false;
    this.tolerance = tolerance;

    addRequirements(driveTrain);
  }

  /**
	 * Resets the pose, gyro, and encoders on the driveTrain.
   * <p><b>NOTE:</b>  This command can run while the robot is disabled.
   * @param curPose robot current pose on the field, components include:
   *  <p> Robot X location in the field, in meters (0 = field edge in front of the Blue driver station, positive = away from the Blue drivestation)
   *  <p> Robot Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, positive = left when looking from the Blue drivestation)
   *  <p> Robot angle on the field (0 = facing away from the Blue drivestation, positive = to the left, negative = to the right)
   * @param tolerance true = do not reset if within 0.5m or 15 degrees of location, false = always reset
   * @param driveTrain DriveTrain subsystem
	 */
  public DriveResetPose(Pose2d curPose, boolean tolerance, DriveTrain driveTrain) {
    this(curPose.getX(), curPose.getY(), curPose.getRotation().getDegrees(), tolerance,
        driveTrain);
  }

  /**
	 * Resets the pose and gyro (not the encoders) on the driveTrain.
   * Reset the angle but keep the current position (use the current measured position as the new position).
   * <p><b>NOTE:</b>  This command can run while the robot is disabled.
   * @param curAngleinDegrees robot angle on the field, in degrees (0 = facing away from the Blue drivestation)
   * @param tolerance true = do not reset if within 0.5m or 15 degrees of location, false = always reset
   * @param driveTrain DriveTrain subsystem
	 */
  public DriveResetPose(double curAngleinDegrees, boolean tolerance, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    curAngle = curAngleinDegrees;
    fromElastic = false;
    onlyAngle = true;
    usingLambda = false;
    this.tolerance = tolerance;

    addRequirements(driveTrain);
  }

  /**
	 * Resets the pose, gyro, and encoders on the driveTrain.
   * <p><b>NOTE:</b>  This command can run while the robot is disabled.
   * @param curPose A Pose2D supplier to return the current robot pose on the field.  Pose components include
   *  <p> Robot X location in the field, in meters (0 = field edge in front of the Blue driver station, positive = away from the Blue drivestation)
   *  <p> Robot Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, positive = left when looking from the Blue drivestation)
   *  <p> Robot angle on the field (0 = facing away from the Blue drivestation, positive = to the left, negative = to the right)
   * @param tolerance true = do not reset if within 0.5m or 15 degrees of location, false = always reset
   * @param driveTrain DriveTrain subsytem
	 */
  public DriveResetPose(Supplier<Pose2d> curPose, boolean tolerance, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    poseSupplier = curPose;
    fromElastic = false;
    onlyAngle = false;
    usingLambda = true;
    this.tolerance = tolerance;

    addRequirements(driveTrain);
  }

  /**
   * Resets the pose, gyro, and encoders on the driveRrain, getting values from Shuffleboard.
   * <p><b>NOTE:</b>  This command can run while the robot is disabled.
   * @param driveTrain DriveTrain subystem
   */
  public DriveResetPose(DriveTrain driveTrain){
    this.driveTrain = driveTrain;
    
    fromElastic = true;
    onlyAngle = false;
    usingLambda = false;
    tolerance = false;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (usingLambda) {
      Pose2d curPose = poseSupplier.get();
      curX = curPose.getX();
      curY = curPose.getY();
      curAngle = curPose.getRotation().getDegrees();
    }

    if (fromElastic) {
      curX = resetXInput.getAsDouble();
      curY = resetYInput.getAsDouble();
      curAngle = resetAngleInput.getAsDouble();
    }

    if (onlyAngle) {
      curX = driveTrain.getPose().getX();
      curY = driveTrain.getPose().getY();
    }
    
    DataLogUtil.writeMessage("DriveResetPose: Init, x =", curX, ", y =", curY, ", angle =", curAngle);

    if (!tolerance || Math.abs(curX - driveTrain.getPose().getX()) > 0.5
        || Math.abs(curY - driveTrain.getPose().getY()) > 0.5
        || Math.abs(MathBCR.normalizeAngle(curAngle - driveTrain.getPoseAngle())) > 15.0
        || Math.abs(MathBCR.normalizeAngle(curAngle - driveTrain.getGyroRotation())) > 15.0) {
            driveTrain.resetPose(new Pose2d(curX, curY, Rotation2d.fromDegrees(curAngle)));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  // Returns true if the command should run when the robot is disabled.
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
