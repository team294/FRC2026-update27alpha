// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.Command;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class HoodSetPercent extends Command {
  private final Shooter shooter;

  private final boolean endImmediately, fromElastic;
  private double percent;
  private int motorNumber;

  private TunableDouble percentInput = Tunables.addDouble(HOOD_CMD_SET_PERCENT_INPUT.value, 0.0);

  /**
   * Set percent for hood motors to move at [0.0, 1.0] with input from Elastic
   * @param endImmediately boolean.  True = command ends immediately, and leaves motors running.  False = command runs forever, and turns off motors when interrupted.
   * @param shooter Shooter subsystem
   * @param motorNumber int motorNumber to set, use 0 for both motors
   */
  public HoodSetPercent(boolean endImmediately, Shooter shooter, int motorNumber) {
    this.shooter = shooter;
    this.endImmediately = endImmediately;
    this.motorNumber = motorNumber;
    fromElastic = true;

    addRequirements(shooter);
  }

  /**
   * Set percent for hood motors to move at [0.0, 1.0] with input
   * @param endImmediately boolean.  True = command ends immediately, and leaves motors running.  False = command runs forever, and turns off motors when interrupted.
   * @param shooter Shooter subsystem
   * @param percent percent of max speed to move at [-1.0, 1.0]
   * @param motorNumber int motorNumber to set, use 0 for both motors
   */
  public HoodSetPercent(boolean endImmediately, double percent, Shooter shooter, int motorNumber) {
    this.shooter = shooter;
    this.endImmediately = endImmediately;
    this.percent = percent;
    this.motorNumber = motorNumber;
    fromElastic = false;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) percent = percentInput.getAsDouble();
    
    if (motorNumber == 0) {
      shooter.setHoodVoltageBothMotors(percent * ShooterConstants.hoodVoltage);
    } else {
      shooter.setHoodVoltageOneMotor(percent * ShooterConstants.hoodVoltage, motorNumber);
    }

    DataLogUtil.writeMessage("Hood Set Percent: Init, Percent =", percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (endImmediately) {
      DataLogUtil.writeMessage("IntakeDeploySetPercent: End");
    } else {
      if (motorNumber == 0) {
        shooter.setHoodVoltageBothMotors(0.0);
      } else {
        shooter.setHoodVoltageOneMotor(0.0, motorNumber);
      }
      DataLogUtil.writeMessage("IntakeDeploySetPercent: End, Percent =", 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endImmediately;
  }
}
