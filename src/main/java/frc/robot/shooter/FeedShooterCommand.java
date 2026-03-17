// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ShooterConstants;

/** Feed fuel into shooter with Feeder and Sterilizer motors */
public class FeedShooterCommand extends Command {
    public FeedShooterCommand() {
        setName("FeedShooterCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setFeederSpeed(ShooterConstants.FEEDER_SPEED);
        ShooterSubsystem.getInstance().setSterilizerSpeed(ShooterConstants.STERILIZER_SPEED);
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