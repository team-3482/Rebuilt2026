package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants.CalculationConstants;
import frc.robot.constants.Constants.Positions;
import frc.robot.hood.HoodSubsystem;
import frc.robot.hood.MoveHoodCommand;
import frc.robot.shooter.RevShooterCommand;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.LookAtPositionCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.ResetPoseCommand;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
            LinearVelocity velocity = ShooterSubsystem.getInstance().getFuelLinearVelocity(distance);
            Logger.recordOutput("Shooter/FuelLinearVelocity", velocity.in(MetersPerSecond), MetersPerSecond);
            Angle angle = HoodSubsystem.getInstance().getShootingHoodAngle(distance, velocity, hub);
            CommandScheduler.getInstance().schedule(new MoveHoodCommand(angle));
        });
    }

    /**
     * Aligns to target, Aims the Hood, and then revs the Shooter.
     * @param target The target.
     * @return The command.
     */
    public static Command AimAndRevShooter(Pose2d target, boolean hub) {
        Distance distance = SwerveSubsystem.getInstance().getDistance(target);
        Logger.recordOutput("Shooter/Target", target);
        if (
            distance.gt(CalculationConstants.MIN_SHOOTING_DISTANCE)
                && distance.lt(CalculationConstants.MAX_SHOOTING_DISTANCE)
        ) {
            return Commands.parallel(
                new LookAtPositionCommand(target, DriverStation.getAlliance().orElse(Alliance.Blue)),
                ContinuousMoveHoodCommand(target, hub),
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
        // TODO: make sure this works
        boolean redAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(DriverStation.Alliance.Red);
        boolean topHalf = SwerveSubsystem.getInstance().getState().Pose.getY() > Positions.HALF_FIELD_Y;

        Pose2d position = redAlliance
            ? (topHalf ? Positions.RED_TOP_FERRY : Positions.RED_BOTTOM_FERRY)
            : (topHalf ? Positions.BLUE_TOP_FERRY : Positions.BLUE_BOTTOM_FERRY);

        return CommandGenerators.AimAndRevShooter(position, false);
    }

    /**
     * Aims to the Hub and revs the Shooter
     * @return The command.
     */
    public static Command PrepareHub() {
        boolean redAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(DriverStation.Alliance.Red);
        return CommandGenerators.AimAndRevShooter(
            redAlliance ? Positions.RED_HUB : Positions.BLUE_HUB,
            true
        );
    }
}