// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.datalog.DataLog;
import org.wpilib.datalog.DoubleLogEntry;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.RobotController;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandGamepad;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.DataLogUtil;

public class IntakeDeployManualControl extends Command {
  private final Intake intake;
  private final CommandGamepad xboxController;

  // Variables for DataLogging
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogPct = new DoubleLogEntry(log, "/IntakeDeployManualControl/IntakeDeployPercent");

  /**
   * Controls the intake using the Xbox controller left joystick.
   * @param xboxController Xbox controller
   * @param intake Intake subsystem
   */
  public IntakeDeployManualControl(CommandGamepad xboxController, Intake intake) {
    this.intake = intake;
    this.xboxController = xboxController;

    addRequirements(intake);

    // Prime data logging at boot time
    long timeNow = RobotController.getTime();
    dLogPct.append(-1, timeNow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogUtil.writeMessage("IntakeDeployManualControl: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakePercent = -xboxController.getLeftY();
    if ((Math.abs(intakePercent) < OIConstants.joystickDeadband)) intakePercent = 0;

    // Slow the control down
    intakePercent *= IntakeConstants.deployMaxManualPercentOutput;
    intake.setIntakeDeployPercentOutput(intakePercent);

    // Log data
    long timeNow = RobotController.getTime();
    dLogPct.append(intakePercent, timeNow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeDeploy();

    DataLogUtil.writeMessage("IntakeDeployManualControl: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
