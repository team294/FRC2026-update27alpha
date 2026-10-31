package frc.robot.commands;

import org.wpilib.command2.Command;

import frc.robot.subsystems.Shooter;

public class HoodLockInPlace extends Command {
    private final Shooter shooter;
    private final boolean lock;

    public HoodLockInPlace(Shooter shooter, boolean lock) {
        this.shooter = shooter;
        this.lock = lock;
    }

    @Override
    public void initialize() {
        shooter.lockHood(lock);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
