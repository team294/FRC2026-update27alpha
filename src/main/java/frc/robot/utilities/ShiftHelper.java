package frc.robot.utilities;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

import org.wpilib.driverstation.MatchState;
import org.wpilib.system.Timer;

import frc.robot.Robot;
import frc.robot.utilities.DataLogUtil.RotationWhenDisabled;

import org.wpilib.driverstation.Alliance;

public class ShiftHelper {
    private final Alliance alliance;
    private final Timer teleopTimer;
    private int logRotationKey;           // Key to identify the log rotation

    /**
     * Keeps track of shifts, publishing data to Elastic regarding whether or not shift is active.
     * @param alliance Current Alliance (Alliance.BLUE, Alliance.RED)
     * @param teleopTimer Timer object tracking match time
     */
    public ShiftHelper(Alliance alliance, Timer teleopTimer) {
        this.alliance = alliance;
        this.teleopTimer = teleopTimer;
        logRotationKey = DataLogUtil.allocateLogRotation();
    }

    /**
     * Returns whether or not Blue Alliance scored more points in autonomous based on FMS message.
     * @return true = blue won, false = red won  
     */
    public boolean blueWonAuto() {
        // return MatchState.getGameData().get().equals("B"); // TODO Test Blue won auto function
        return MatchState.getGameData().orElse("").equals("B"); // TODO Test Blue won auto function.   Replaced get() with orElse() so simulation does not crash.
    }

    /**
     * Returns time left in the current scoring shift, regardless of which Alliance's it is.
     * @return time left in the current shift, in seconds
     */
    public double timeLeftInShiftSeconds() {
        if (teleopTimer.get() <= 0.0) {return 0.0;}

        if (teleopTimer.get() <= 10.0) {return 10.0 - teleopTimer.get();}
        else if (teleopTimer.get() <= 35.0) {return 35.0 - teleopTimer.get();}
        else if (teleopTimer.get() <= 60.0) {return 60.0 - teleopTimer.get();}
        else if (teleopTimer.get() <= 85.0) {return 85.0 - teleopTimer.get();}
        else if (teleopTimer.get() <= 110.0) {return 110.0 - teleopTimer.get();}
        else {return 140.0 - teleopTimer.get();}
    }

    /**
     * Returns whether or not it is Blue Alliance's shift
     * @return true = Blue Alliance scoring shift, false = Red Alliance scoring shift. ignores transition and endgame
     */
    public boolean isCurrentShiftBlue() {
        boolean firstOrThirdShift = (teleopTimer.get() > 10.0 && teleopTimer.get() <= 35.0) || (teleopTimer.get() > 85.0);
        return blueWonAuto() ? !firstOrThirdShift : firstOrThirdShift;
    }

    /**
     * Returns whether or not your hub is active.
     * @return true = your scoring shift (transition, 2 alliance-specific, endgame), false = not your shift (other 2 alliance-specific)
     */
    public boolean isCurrentShiftYours() {
        if (teleopTimer.get() <= 0) {return false;}

        if (teleopTimer.get() <= 10 || teleopTimer.get() >= 130) {
            return true;
        } else if (alliance == Alliance.BLUE) {
            return isCurrentShiftBlue();
        } else {
            return !isCurrentShiftBlue();
        }
    }

    public void periodic() {
        if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNormal)) {
            Robot.dashboard.log(TELEOP_TIMER.value, Math.max(140.0 - teleopTimer.get(), 0));
            Robot.dashboard.log(SHIFT_TIMER.value, timeLeftInShiftSeconds());
            Robot.dashboard.log(IS_BLUE_SHIFT.value, isCurrentShiftBlue());
            Robot.dashboard.log(IS_YOUR_SHIFT.value, isCurrentShiftYours());
        }
    }
}