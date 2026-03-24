// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

/** Rev up shooter to speed based on distance from target */
public class RevShooterCommand extends Command {
    Pose2d target;
    public RevShooterCommand(Pose2d target) {
        setName("RevShooterCommand");

        this.target = target;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());

    }

    @Override
    public void initialize() {
        Distance distance = SwerveSubsystem.getInstance().getDistance(target);
        Logger.recordOutput("Shooter/DistanceToTarget", distance.in(Meters));
        AngularVelocity velocity = ShooterSubsystem.getInstance().calculateShooterAngularVelocity(distance);
        Logger.recordOutput("Shooter/SetVelocity", velocity.in(RPM));
        ShooterSubsystem.getInstance().setShooterAngularVelocity(velocity);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setShooterSpeed(0);
        ShooterSubsystem.getInstance().setLastTargetVelocity(RPM.of(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}