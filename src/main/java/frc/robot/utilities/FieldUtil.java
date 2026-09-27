// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.io.UncheckedIOException;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.driverstation.Alliance;
import frc.robot.Constants.FieldConstants;

/** BCR Field class (implementation in progress) */
public class FieldUtil {
    private AllianceSelection allianceSelection;
    private Field fieldLayout;

    /**
     * Constructor for Field objects.
     * @param allianceSelection
     */
    public FieldUtil(AllianceSelection allianceSelection) {
        this.allianceSelection = allianceSelection;

        // Create aprilTagFieldLayout for pose estimators and photonvision
        try {
            fieldLayout = Fields.FRC_2026_REBUILT_WELDED.loadField();
            fieldLayout.setOrigin(Field.OriginPosition.BLUE_ALLIANCE_WALL_RIGHT_SIDE);
            DataLogUtil.writeMessage("Field Constructor:  Loaded april tags from file");
        } catch (UncheckedIOException e) {
            RobotPreferences.recordStickyFaults("Field");
            DataLogUtil.writeMessage("Field Constructor:  Error loading april tags from file");
            e.printStackTrace();
        }
    }

    /**
     * Returns the field layout (including AprilTags) for the current season and field
     * @return
     */
    public Field getField() {
        return fieldLayout;
    }

    /**
     * Get target to aim at based on current robot location. <b> Does NOT account for robot moving, use getBallTarget() instead if needed. </b>
     * @param robotTranslation2d Translation2d object of robot position.
     * @return Translation2d object of target.
     */
    public Translation2d getStaticShotTarget(Translation2d robotTranslation2d) {
        if (allianceSelection.getAlliance() == Alliance.BLUE) {
            if (robotTranslation2d.getX() < FieldConstants.hubPositionBlue.getX()) {return FieldConstants.hubPositionBlue;}
            else {
                if (robotTranslation2d.getY() < FieldConstants.width/2.0) {return FieldConstants.rightPassBlue;}
                else {return FieldConstants.leftPassBlue;}
            }
        } else {
            if (robotTranslation2d.getX() > FieldConstants.hubPositionRed.getX()) {return FieldConstants.hubPositionRed;}
            else {
                if (robotTranslation2d.getY() < FieldConstants.width/2.0) {return FieldConstants.rightPassRed;}
                else {return FieldConstants.leftPassRed;}
            }
        }
    }

    /**
     * Get "ghost" target to use for aiming shots to account for robot acceleration and velocity
     * @param staticTargetTranslation2d Translation2d object of shot target
     * @param turretTranslation2d Translation2d object of turret (shooter)
     * @param robotFieldRelativeVelocities <i>Field-relative</i> robot velocities
     * @return Translation2d object of "ghost" target to aim at
     */
    public Translation2d getGhostShotTarget(Translation2d staticTargetTranslation2d, Translation2d turretTranslation2d, ChassisVelocities robotFieldRelativeVelocities) {
        double estTimeOfFlight = turretTranslation2d.getDistance(staticTargetTranslation2d) * FieldConstants.timeOfFlightSlope + FieldConstants.timeOfFlightIntercept;

        // We want to aim to counteract the offset caused by the added velocity that the robot moving has on the ball.
        // During the time of flight, the ball deflects from where we want it to be by the formula: velocity*time
        // Acceleration does not matter since there is no forces acting on the ball in the horizontal plane other than negligible? air resistance 
        // This displacement is subtracted from the static target position because if the robot moves
        // +1m left during flight, the target appears to shift -1m from the robot's POV so we have to aim 1m short
        double offsetX = (robotFieldRelativeVelocities.vx * estTimeOfFlight);
        double offsetY = (robotFieldRelativeVelocities.vy * estTimeOfFlight);

        // ghost target = static target + displacement
        return new Translation2d(staticTargetTranslation2d.getX() + offsetX, staticTargetTranslation2d.getY() + offsetY);
    }
}