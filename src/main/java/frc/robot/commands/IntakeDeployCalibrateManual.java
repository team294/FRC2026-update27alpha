// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import frc.robot.Constants.IntakeConstants.IntakeDeployAngle;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class IntakeDeployCalibrateManual extends InstantCommand {
  private final Intake intake;
  private double angle;
  private boolean fromElastic;

  private static final TunableDouble angleInput = Tunables.addDouble(INTAKE_DEPLOY_CMD_CALIBRATE_MANUAL_INPUT.value, IntakeDeployAngle.INTAKE.value);

  /**
   * Manually calibrate the intakeDeployMotor with a known current angle.
   * @param angle the known current angle, in degrees
   * @param intake Intake subsystem
   */
  public IntakeDeployCalibrateManual(double angle, Intake intake) {
    this.intake = intake;
    this.angle = angle;
    this.fromElastic = false;
    addRequirements(intake);
  }

  /**
   * Manually calibrate the intakeDeployMotor with a known current angle FROM ELASTIC.
   * @param intake Intake subsystem
   */
  public IntakeDeployCalibrateManual(Intake intake) {
    this.intake = intake;
    this.fromElastic = true;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) this.angle = angleInput.getAsDouble();
    intake.calibrateIntakeDeployEncoder(angle);
    DataLogUtil.writeMessage("IntakeDeployCalibrateManual: Init, Angle =", angle);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
