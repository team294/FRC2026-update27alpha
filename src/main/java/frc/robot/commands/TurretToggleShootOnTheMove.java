// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import frc.robot.subsystems.Turret;


public class TurretToggleShootOnTheMove extends InstantCommand {
  private final Turret turret;

  /**
   * Toggle auto-aim functionality on Turret. Shoot on the move will only be enabled if auto aim is also enabled.
   * @param turret Turret subsystem
   */
  public TurretToggleShootOnTheMove(Turret turret) {
    this.turret = turret;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.toggleShootOnTheMove();
  }

  // Returns whether the command should be able to run while the robot is disabled (default = false). 
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
