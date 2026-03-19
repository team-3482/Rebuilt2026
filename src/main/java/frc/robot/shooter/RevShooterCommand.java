// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveSubsystem;

/** Rev up shooter to speed based on distance from target */
public class RevShooterCommand extends Command {
    Pose2d target;
    public RevShooterCommand(Pose2d target) {
        setName("RevShooterCommand");

        this.target = target;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());

        Distance distance = SwerveSubsystem.getInstance().getDistance(target);
        AngularVelocity velocity = ShooterSubsystem.getInstance().calculateShooterAngularVelocity(distance);
        System.out.println(velocity);
        ShooterSubsystem.getInstance().motionMagicAngularVelocity(velocity);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setShooterSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}