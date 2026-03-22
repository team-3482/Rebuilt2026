// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.hood.HoodSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/** Feed fuel into shooter with Feeder and Sterilizer motors */
public class FeedShooterCommand extends Command {
    private final Timer timer = new Timer();
    private boolean shouldFeed = true;
    private boolean doChecks;

    public FeedShooterCommand(boolean doChecks) {
        setName("FeedShooterCommand");

        this.doChecks = doChecks;

        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(ShooterSubsystem.getInstance());
        // addRequirements(HoodSubsystem.getInstance());
        // addRequirements(SwerveSubsystem.getInstance());
    }

    public FeedShooterCommand() {
        this(true);
    }

    @Override
    public void initialize() {
        timer.restart();

        ShooterSubsystem.getInstance().setSterilizerSpeed(ShooterConstants.STERILIZER_SPEED);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Shooter/AtShootingVelocityThreshold", ShooterSubsystem.getInstance().isShooterVelocityWithinTolerance());
        Logger.recordOutput("Shooter/AtShootingVelocityThreshold", ShooterSubsystem.getInstance().isShooterVelocityWithinTolerance());

        if(
            !doChecks
            || (ShooterSubsystem.getInstance().isShooterVelocityWithinTolerance()
            && HoodSubsystem.getInstance().isPositionWithinTolerance()
            && SwerveSubsystem.getInstance().angleWithinToleranceToTarget())
        ) {
            if (timer.advanceIfElapsed(shouldFeed ? ShooterConstants.FEEDER_FEED_DURATION : ShooterConstants.FEEDER_PAUSE_DURATION)) {
                shouldFeed = !shouldFeed;
            }

            if (shouldFeed) {
                ShooterSubsystem.getInstance().setFeederSpeed(ShooterConstants.FEEDER_SPEED);
            } else {
                ShooterSubsystem.getInstance().setFeederSpeed(0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setFeederSpeed(0);
        ShooterSubsystem.getInstance().setSterilizerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}