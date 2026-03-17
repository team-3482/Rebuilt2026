package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.hood.HoodSubsystem;
import frc.robot.hood.MoveHoodCommand;
import frc.robot.shooter.FeedShooterCommand;
import frc.robot.shooter.RevShooterCommand;
import frc.robot.swerve.LookAtPositionCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.ResetPoseCommand;

/** Class that holds commands that don't need to clutter RobotContainer */
public class CommandGenerators {
    /**
     * A command that cancels all running commands.
     * @return The command.
     */
    public static Command CancelAllCommands() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll());
    }

    /**
     * A command that resets the odometry to an empty Pose2d.
     * @return The command.
     */
    public static Command ResetOdometryToOriginCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> SwerveSubsystem.getInstance().resetPose(Pose2d.kZero)),
            new ResetPoseCommand().withTimeout(0.25)
        );
    }

    /**
     * A command that takes the current orientation of the robot
     * and makes it X forward for field-relative maneuvers.
     * @return The command.
     */
    public static Command SetForwardDirectionCommand() {
        return Commands.runOnce(() -> SwerveSubsystem.getInstance().seedFieldCentric());
    }

    /**
     * Continuously move the hood angle to meet a target's distance.
     * @param target The target.
     * @return The command.
     */
    public static Command ContinuousMoveHoodCommand(Pose2d target) {
        return Commands.run(() -> {
            Distance distance = SwerveSubsystem.getInstance().getDistance(target);
            new MoveHoodCommand(HoodSubsystem.getInstance().getShootingHoodAngle(distance, false));
        });
    }

    /**
     * Aligns to target, Aims the Hood, and then revs the Shooter.
     * @param target The target.
     * @return The command.
     */
    public static Command AimAndRevShooter(Pose2d target) {
        return Commands.parallel(
            new LookAtPositionCommand(target),
            ContinuousMoveHoodCommand(target),
            new RevShooterCommand(target)
        );
    }

    /**
     * Feeds Fuel into Shooter if at correct velocity and Hood is at correct position.
     * @return The command.
     */
    public static Command FeedShooter() {
        return Commands.run(() -> { // TODO: rewrite atShootingVelocityThreshold
            if (/*ShooterSubsystem.getInstance().atShootingVelocityThreshold() &&*/HoodSubsystem.getInstance().isPositionWithinTolerance()) {
                new FeedShooterCommand();
            }
        });
    }
}