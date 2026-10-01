// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.Command;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeDeployAngle;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class IntakeDeploySetAngle extends Command {
  private final Intake intake;
  private double targetAngle;
  private boolean fromElastic;

  private static final TunableDouble angleInput = Tunables.addDouble(INTAKE_DEPLOY_CMD_SET_ANGLE_INPUT.value, IntakeDeployAngle.START_CONFIG.value);

  /**
   * Sets the target angle for the Intake Deploy and attempts to move it to that angle. It is constrained by
   * calibration and interlocks. This command ends when the position is within the tolerance degrees of the target,
   * or immediately if uncalibrated.
   * @param angle target angle, in degrees (0 = orange arrow straight up, positive = in, negative = out)
   * @param intake Intake subsystem
   */
  public IntakeDeploySetAngle(double angle, Intake intake) {
    this.intake = intake;
    this.targetAngle = angle;
    this.fromElastic = false;
    addRequirements(intake);
  }

  /**
   * Sets the target angle for the Intake Deploy and attempts to move it to that angle. It is constrained by
   * calibration and interlocks. This command ends when the position is within the tolerance degrees of the target,
   * or immediately if uncalibrated.
   * @param angle target angle, in degrees (see IntakeConstants.IntakeDeployAngle)
   * @param intake Intake subsystem
   */
  public IntakeDeploySetAngle(IntakeDeployAngle angle, Intake intake) {
    this.intake = intake;
    this.targetAngle = angle.value;
    this.fromElastic = false;
    addRequirements(intake);
  }

  /**
   * Sets the target angle for the Intake Deploy FROM ELASTIC and attempts to move it to that angle. It is constrained by
   * calibration and interlocks. This command ends when the position is within the tolerance degrees of the target,
   * or immediately if uncalibrated.
   * @param intake Intake subsystem
   */
  public IntakeDeploySetAngle(Intake intake) {
    this.intake = intake;
    this.fromElastic = true;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) this.targetAngle = angleInput.getAsDouble();
    intake.setIntakeDeployAngle(targetAngle);

    intake.enableFastLogging(true);
    DataLogUtil.writeMessage("IntakeDeploySetAngle: Init, Target =", targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.enableFastLogging(false);
    if (interrupted) DataLogUtil.writeMessage("IntakeDeploySetAngle: Interrupted, Target =", targetAngle, ", Current =", intake.getIntakeDeployAngle());
    else {
      DataLogUtil.writeMessage("IntakeDeploySetAngle: End, Target =", targetAngle, ", Current =", intake.getIntakeDeployAngle());
      if (targetAngle == IntakeDeployAngle.INTAKE.value) intake.setIntakeDeployPercentOutput(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intake.isIntakeDeployCalibrated() || Math.abs(intake.getIntakeDeployAngle() - intake.getIntakeDeployTargetAngle()) < IntakeConstants.deploySetAngleTolerance;
  }
}
