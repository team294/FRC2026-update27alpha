// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static org.wpilib.command2.Commands.*;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.system.Timer;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitUntilCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Constants.StopType;
import frc.robot.Constants.AutoConstants.AutoEndType;
import frc.robot.Constants.AutoConstants.BallPickupTrajType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.Constants.AutoConstants.RoutineSideType;
import frc.robot.Constants.AutoConstants.ShootPreloadType;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.autos.components.AutonCollectBalls;
import frc.robot.commands.autos.components.AutonDriveToNeutralAndCollectBalls;
import frc.robot.commands.autos.components.AutonReturnToAllianceAndShoot;
import frc.robot.commands.autos.components.AutonScore;
import frc.robot.commands.sequences.IntakeSequence;
import frc.robot.commands.autos.components.AutonDriveToNeutralZone;
import frc.robot.commands.autos.components.AutonOutpostOrDepotAndScore;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.FieldUtil;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BuildLeftBumpDepotAuton extends SequentialCommandGroup {
  /** Creates a new BuildAutoRoutine. */
  public BuildLeftBumpDepotAuton(ShootPreloadType shootPreload, Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache, FieldUtil field) { 
    // Initial pose: if shooting preload = inital pose of first trajectory - robot length + front bumper thickness
    //           if not shooting preload = inital pose of first trajectory
    Pose2d initialTrajectoryPose = cache.getTrajectory(true, TrajectoryName.StartTrenchToCollectStart).getInitialPose(alliance.getAlliance() == Alliance.RED).get();
    Pose2d initalRobotStartPose = shootPreload == ShootPreloadType.kShootPreload ?  
                                initialTrajectoryPose.transformBy(new Transform2d(-RobotDimensions.robotLength + RobotDimensions.frontBumperThickness, 0, Rotation2d.ZERO)) : 
                                initialTrajectoryPose;

    addCommands(
      parallel(
        new DataLogMessage(false, "BuildAutonRoutine: shootPreload = " + shootPreload + ", ballPickupPath = " + BallPickupTrajType.kDriveThroughSide + ", returnToAlliancePath = " + ReturnToAllianceType.kOverBump + ", autoEnd = " + AutoEndType.kNone + ", routineSide = " + RoutineSideType.kLeftSide),
        new DriveResetPose(initalRobotStartPose, false, driveTrain)
      ),
      new AutonDriveToNeutralAndCollectBalls(shootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kOverBump, true, intake, feeder, turret, shooter, climber, driveTrain, alliance, cache, field),
      new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(true, TrajectoryName.CollectEndToShootBump), driveTrain, alliance),
      parallel(
          new AutonScore(EndType.runForever, true, intake, feeder, turret, shooter, climber, driveTrain, field),
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(true, TrajectoryName.ShootBumpToDepot), driveTrain, alliance)
      )
    );
  }
}
