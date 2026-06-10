// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import frc.robot.subsystems.Turret;

public class TurretToggleAutoAim extends InstantCommand {
  private final Turret turret;

  /**
   * Toggle auto-aim functionality on Turret
   * @param turret Turret subsystem
   */
  public TurretToggleAutoAim(Turret turret) {
    this.turret = turret;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.toggleAutoAim();
  }
}
