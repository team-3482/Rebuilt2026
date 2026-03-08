// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.VirtualConstants;
import frc.robot.constants.VirtualConstants.Shooter;
import frc.robot.template.ExampleSubsystem;

/** Rev up shooter and start feeding when up to speed */
public class ShootCommand extends Command {
    public ShootCommand() {
        setName("ShootCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setShooterSpeed(VirtualConstants.Shooter.SHOOTING_SPEED);
    }

    @Override
    public void execute() {
        if(ShooterSubsystem.getInstance().atShootingVelocityThreshold()) {
            ShooterSubsystem.getInstance().setFeederSpeed(VirtualConstants.Shooter.FEEDER_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setShooterSpeed(0);
        ShooterSubsystem.getInstance().setFeederSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}