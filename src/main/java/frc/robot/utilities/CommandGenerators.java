package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants.PositionConstants;
import frc.robot.hood.HoodSubsystem;
import frc.robot.hood.MoveHoodCommand;
import frc.robot.shooter.FeedShooterCommand;
import frc.robot.shooter.RevShooterCommand;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.LookAtPositionCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.ResetPoseCommand;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.constants.Constants.CalculationConstants.MIN_SHOOTING_DISTANCE;

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
    public static Command ContinuousMoveHoodCommand(Pose2d target, boolean hub) {
        return Commands.runOnce(() -> {
            Distance distance = SwerveSubsystem.getInstance().getDistance(target);
            Angle angle = HoodSubsystem.getInstance().getShootingHoodAngle(distance, hub);
            System.out.println(angle.in(Degrees));
            CommandScheduler.getInstance().schedule(new MoveHoodCommand(angle));
        });
    }

    /**
     * Aligns to target, Aims the Hood, and then revs the Shooter.
     * @param target The target.
     * @return The command.
     */
    public static Command AimAndRevShooter(Pose2d target, boolean hub) {
        if (SwerveSubsystem.getInstance().getDistance(target).gt(MIN_SHOOTING_DISTANCE)) {
            return Commands.parallel(
                new LookAtPositionCommand(target),
                // ContinuousMoveHoodCommand(target, hub),
                new RevShooterCommand(target)
            );
        } else {
            System.out.println("Too close to target!!!");
            return Commands.none();
        }
    }

    /**
     * Aims to our alliance side and revs shooter
     * @return The command.
     */
    public static Command PrepareFerry() {
        return Commands.runOnce(() -> {
            boolean redAlliance = DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
            boolean rightSide = SwerveSubsystem.getInstance().getState().Pose.getY() > PositionConstants.HALF_FIELD_Y;

            Pose2d position = redAlliance ?
                (rightSide ? PositionConstants.RED_RIGHT_FERRY : PositionConstants.RED_LEFT_FERRY)  :
                (rightSide ? PositionConstants.BLUE_RIGHT_FERRY : PositionConstants.BLUE_LEFT_FERRY);

            CommandScheduler.getInstance().schedule(CommandGenerators.AimAndRevShooter(position, false));
        });
    }

    /**
     * Aims to the Hub and revs the Shooter
     * @return The command.
     */
    public static Command PrepareHub() {
        return Commands.runOnce(() -> {
            boolean redAlliance = DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
            CommandScheduler.getInstance().schedule(
                CommandGenerators.AimAndRevShooter(
                    redAlliance ? PositionConstants.RED_HUB : PositionConstants.BLUE_HUB,
                    true
                )
            );
        });
    }


    /**
     * Feeds Fuel into Shooter if at correct velocity and Hood is at correct position.
     * @return The command.
     */
    public static Command FeedShooter() {
        return Commands.run(() -> {
            if (
                ShooterSubsystem.getInstance().atShootingVelocityThreshold()
                && HoodSubsystem.getInstance().isPositionWithinTolerance()
                && SwerveSubsystem.getInstance().angleWithinToleranceToTarget()
            ) { // TODO: move intake pivot up and down
                new FeedShooterCommand();
            }
        });
    }
}