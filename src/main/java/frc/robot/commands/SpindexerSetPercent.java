// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.Command;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import frc.robot.subsystems.Feeder;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class SpindexerSetPercent extends Command {
  private final Feeder feeder;
  private double percent = 0.0;
  private boolean endImmediately, fromElastic;

  private TunableDouble percentInput = Tunables.addDouble(SPINDEXER_CMD_SET_PERCENT_INPUT.value, 0.0);

  /**
   * Sets the percent output of the spindexerMotor FROM ELASTIC.
   * @param endImmediately true = end command immediately, false = end on interrupt and stop the motor
   * @param feeder Feeder subsystem
   */
  public SpindexerSetPercent(boolean endImmediately, Feeder feeder) {
    this.feeder = feeder;
    this.endImmediately = endImmediately;
    fromElastic = true;
    addRequirements(feeder);
  }

  /**
   * Sets the percent output of the spindexerMotor.
   * @param percent output percent, -1.0 to 1.0 (positive = forward, negative = reverse)
   * @param endImmediately true = end command immediately, false = end on interrupt and stop the motor
   * @param feeder Feeder subsystem
   */  
  public SpindexerSetPercent(double percent, boolean endImmediately, Feeder feeder) {
    this.feeder = feeder;
    this.percent = percent;
    this.endImmediately = endImmediately;
    fromElastic = false;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) percent = percentInput.getAsDouble();
    feeder.setSpindexerPercentOutput(percent);
    DataLogUtil.writeMessage("SpindexerSetPercent: Init, Percent =", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (endImmediately) {
      DataLogUtil.writeMessage("SpindexerSetPercent: End, Percent =", percent);
    } else {
      feeder.stopSpindexer();
      DataLogUtil.writeMessage("SpindexerSetPercent: End, Percent =", 0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endImmediately;
  }
}

