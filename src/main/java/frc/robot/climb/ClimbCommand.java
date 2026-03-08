// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimbConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.shooter.ShooterSubsystem;

/** Climb from floor to tower */
public class ClimbCommand extends Command {
    public ClimbCommand() {
        setName("ClimbCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ClimbSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ClimbSubsystem.getInstance().setClimbSpeed(ClimbConstants.CLIMB_SPEED);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        ClimbSubsystem.getInstance().setClimbSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return ClimbSubsystem.getInstance().getClimbPosition().in(Units.Revolutions) >= ClimbConstants.FULL_RETRACTION_REVOLUTIONS.in(Units.Revolutions);
    }
}