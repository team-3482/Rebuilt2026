// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ShooterConstants;

/** Feed fuel into shooter with Feeder and Sterilizer motors */
public class FeedShooterCommand extends Command {
    private final Timer timer = new Timer();
    private boolean shouldFeed = true;
    private final boolean doChecks;
    private final boolean reversed;

    public FeedShooterCommand(boolean doChecks, boolean reversed) {
        setName("FeedShooterCommand");

        this.doChecks = doChecks;
        this.reversed = reversed;
    }

    public FeedShooterCommand() {
        this(true, false);
    }

    @Override
    public void initialize() {
        timer.restart();

        double direction = reversed ? -1.0 : 1.0;

        ShooterSubsystem.getInstance().setSterilizerSpeed(ShooterConstants.STERILIZER_SPEED * direction);
        ShooterSubsystem.getInstance().setFeederSpeed(ShooterConstants.FEEDER_SPEED * direction);
    }

    // @Override
    // public void execute() {
    //     if (
    //         !doChecks
    //         || (ShooterSubsystem.getInstance().isShooterVelocityWithinTolerance()
    //         && HoodSubsystem.getInstance().isPositionWithinTolerance()
    //         && SwerveSubsystem.getInstance().angleWithinToleranceToTarget())
    //     ) {
    //         if (timer.advanceIfElapsed(shouldFeed ? ShooterConstants.FEEDER_FEED_DURATION : ShooterConstants.FEEDER_PAUSE_DURATION)) {
    //             shouldFeed = !shouldFeed;
    //         }
    //
    //         if (shouldFeed) {
    //             ShooterSubsystem.getInstance().setFeederSpeed(ShooterConstants.FEEDER_SPEED);
    //         } else {
    //             ShooterSubsystem.getInstance().setFeederSpeed(0);
    //         }
    //     }
    // }

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setFeederSpeed(0);
        ShooterSubsystem.getInstance().setSterilizerSpeed(0);
    }
}