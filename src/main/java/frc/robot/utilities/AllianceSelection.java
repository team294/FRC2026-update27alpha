// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.driverstation.Alliance;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

import frc.robot.Robot;
import frc.robot.utilities.DataLogUtil.RotationWhenDisabled;

public class AllianceSelection {
  
  private int logRotationKey;
  private Alliance alliance;  // Red or Blue
  private List<Consumer<Alliance>> newAllianceCalls;

  private enum AllianceChoice {
    Red,
    Blue,
    Auto
  }

  private Selectable<AllianceChoice> allianceChooser = new Selectable<>();

  public AllianceSelection() {
    
    newAllianceCalls = new ArrayList<Consumer<Alliance>>();
    logRotationKey = DataLogUtil.allocateLogRotation();

		allianceChooser.addDefault("Automatic", AllianceChoice.Auto);
		allianceChooser.add("Red", AllianceChoice.Red);
		allianceChooser.add("Blue", AllianceChoice.Blue);
	
		// Show alliance selection widget on Elastic
    Tunables.publish("Alliance Selection", allianceChooser);

    // Set Alliance on Elastic
    Optional<Alliance> ally = MatchState.getAlliance();
    if (ally.isPresent()) {
      setAlliance(ally.get());
      DataLogUtil.writeMessageEcho("Alliance Selection: Init, Alliance from DriverStation =", alliance.name());
    } else {
      setAlliance(Alliance.BLUE);
      DataLogUtil.writeMessageEcho("Alliance Selection: Init, DriverStation not present - default to Blue. ", alliance.name());
    }
  }

  /**
   * Sets the current alliance.
   * @param alliance Red, Blue, or Invalid
   */
  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
    DataLogUtil.writeMessageEcho("Alliance Selection: SetAlliance, Alliance Chooser =", allianceChooser.getSelected().name(), ", Alliance =", alliance.name());

    Telemetry.getTable("AllianceSelection").log("Alliance Blue", alliance != Alliance.RED);
    Telemetry.getTable("AllianceSelection").log("Alliance Red", alliance != Alliance.BLUE);
    Telemetry.getTable("AllianceSelection").log("Alliance", alliance.name());
  }

  /**
   * Adds a consumer to the newAllianceCalls list that is called whenever the alliance of the robot is changed.
   * @param newConsumer new Alliance consumer that will be added to the list
   */
  public void addAllianceChangeNotification(Consumer<Alliance> newConsumer) {
    newAllianceCalls.add(newConsumer);
  }

  /**
   * Gets the current alliance.
   * @return Red or Blue
   */
  public Alliance getAlliance() {
    return alliance;
  }

  /**
   * Runs once per scheduler cycle.
   */
  public void periodic() {
    if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNormal)) {
      Alliance newAlliance = alliance;

      switch (allianceChooser.getSelected()) {
        case Auto:
          // Do not auto change alliance in Auto or Teleop mode
          if (RobotState.isDisabled()) {
            Optional<Alliance> ally = MatchState.getAlliance();
            if (ally.isPresent()) newAlliance = ally.get();
          }
          break;
        case Red:
          newAlliance = Alliance.RED;
          break;
        case Blue:
          newAlliance = Alliance.BLUE;
          break;
      }

      if (alliance != newAlliance) {
        setAlliance(newAlliance);
        for (Consumer<Alliance> setters : newAllianceCalls) {
          setters.accept(newAlliance);
        }
      }
    }
  }
}