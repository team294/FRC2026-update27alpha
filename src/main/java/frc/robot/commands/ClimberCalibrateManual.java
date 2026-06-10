// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import frc.robot.subsystems.Climber;

public class ClimberCalibrateManual extends InstantCommand {
  private final Climber climber;

  /**
   * Recalibrates Climber subsystem. Both sides of Climber must be at lower limit.
   * @param climber Climber subsystem
   */
  public ClimberCalibrateManual(Climber climber) {
    this.climber = climber;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.calibrateEncoderLeft();
    climber.calibrateEncoderRight();
  }

  // Returns whether the command should be able to run while the robot is disabled (default = false).
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
