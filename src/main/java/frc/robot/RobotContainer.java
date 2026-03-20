// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.climb.ClimbCommand;
import frc.robot.climb.ClimbSubsystem;
import frc.robot.climb.LeaveClimbCommand;
import frc.robot.constants.Constants.DriverStationConstants;
import frc.robot.constants.Constants.HoodConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.hood.HoodSubsystem;
import frc.robot.hood.MoveHoodCommand;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.MovePivotCommand;
import frc.robot.shooter.FeedShooterCommand;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.SwerveTelemetry;
import frc.robot.utilities.CommandGenerators;
import frc.robot.vision.CalibrateQuestCommand;
import frc.robot.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Map;
import java.util.function.Supplier;

public class RobotContainer {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class RobotContainerHolder {
        private static final RobotContainer INSTANCE = new RobotContainer();
    }

    public static RobotContainer getInstance() {
        return RobotContainerHolder.INSTANCE;
    }

    private final SendableChooser<Command> autoChooser;
    private Command auton = null;

    // Instance of the controllers used to drive the robot
    private final CommandXboxController driverController;
    private final XboxController driverController_HID;
    private final CommandXboxController operatorController;

    public RobotContainer() {
        // Initialize controllers
        this.driverController = new CommandXboxController(DriverStationConstants.DRIVER_CONTROLLER_ID);
        this.driverController_HID = this.driverController.getHID();
        this.operatorController = new CommandXboxController(DriverStationConstants.OPERATOR_CONTROLLER_ID);

        configureDrivetrain();
        initializeSubsystems();
        registerNamedCommands();

        this.autoChooser = AutoBuilder.buildAutoChooser(); // The default auto will be Commands.none()
        this.autoChooser.onChange((Command autoCommand) -> this.auton = autoCommand); // Reloads the stored auto

        // SmartDashboard.putData("Auto Chooser", this.autoChooser);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    /**
     * Creates instances of each subsystem so periodic runs on startup.
     */
    @SuppressWarnings("ResultOfMethodCallIgnored")
    private void initializeSubsystems() {
        ClimbSubsystem.getInstance();
        HoodSubsystem.getInstance();
        IntakeSubsystem.getInstance();
        ShooterSubsystem.getInstance();
        VisionSubsystem.getInstance();
    }

    /**
     * This method initializes the swerve subsystem and configures its bindings with the driver controller.
     * This is based on the Phoenix6 Swerve example.
     */
    private void configureDrivetrain() {
        final SwerveSubsystem Drivetrain = SwerveSubsystem.getInstance();

        final double NormalSpeed = TunerConstants.kSpeedNormal.in(Units.MetersPerSecond);
        final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
        final double NormalAngularSpeed = TunerConstants.kAngularSpeedNormal.in(Units.RadiansPerSecond);
        final double FastAngularSpeed = TunerConstants.kAngularSpeedFast.in(Units.RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        final SwerveRequest.FieldCentric fieldCentricDrive_withDeadband = new SwerveRequest
            .FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

        Supplier<Boolean> leftTrigger = () -> this.driverController_HID.getLeftTriggerAxis() >= 0.5;
        Supplier<Boolean> rightTrigger = () -> this.driverController_HID.getRightTriggerAxis() >= 0.5;

        // Drivetrain will execute this command periodically
        Drivetrain.setDefaultCommand(
            Drivetrain.applyRequest(() -> {
                boolean topSpeed = leftTrigger.get();
                boolean fineControl = rightTrigger.get();

                double linearSpeed = topSpeed ? MaxSpeed : NormalSpeed;

                Logger.recordOutput("DriveState/MaxSpeed", linearSpeed);
                SmartDashboard.putNumber("DriveState/MaxSpeed", linearSpeed);

                double angularSpeed = topSpeed ? NormalAngularSpeed : FastAngularSpeed;

                double fineControlMult = fineControl ? DriverStationConstants.FINE_CONTROL_MULT : 1;

                return fieldCentricDrive_withDeadband
                    // Drive forward with negative Y (forward)
                    .withVelocityX(-driverController.getLeftY() * linearSpeed * fineControlMult)
                    // Drive left with negative X (left)
                    .withVelocityY(-driverController.getLeftX() * linearSpeed * fineControlMult)
                    // Drive counterclockwise with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * angularSpeed * fineControlMult)

                    .withDeadband(DriverStationConstants.DEADBAND * linearSpeed)
                    .withRotationalDeadband(DriverStationConstants.DEADBAND * angularSpeed);
            }).ignoringDisable(true)
        );

        // POV / D-PAD
        final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // POV angle: [X velocity, Y velocity] in m/s
        final Map<Integer, Integer[]> povSpeeds = Map.ofEntries(
            Map.entry(0, new Integer[]{1, 0}),
            Map.entry(45, new Integer[]{1, -1}),
            Map.entry(90, new Integer[]{0, -1}),
            Map.entry(135, new Integer[]{-1, -1}),
            Map.entry(180, new Integer[]{-1, 0}),
            Map.entry(225, new Integer[]{-1, 1}),
            Map.entry(270, new Integer[]{0, 1}),
            Map.entry(315, new Integer[]{1, 1})
        );

        povSpeeds.forEach(
            (Integer angle, Integer[] speeds) -> this.driverController.pov(angle).whileTrue(
                Drivetrain.applyRequest(() -> {
                    boolean faster = leftTrigger.get();
                    boolean robotCentric = rightTrigger.get();

                    return robotCentric
                        ? robotCentricDrive
                        .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                        .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25))
                        : fieldCentricDrive
                        .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                        .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25));
                })
            )
        );

        Drivetrain.registerTelemetry(logger::telemeterize);
    }

    /** Configures the button bindings of the driver controller. */
    public void configureDriverBindings() {
        // Double Rectangle (Left) -> Reset pose
        this.driverController.back().onTrue(CommandGenerators.ResetOdometryToOriginCommand());
        // Burger (Right) -> Reset rotation to zero
        this.driverController.start().onTrue(CommandGenerators.SetForwardDirectionCommand());

        // B -> Cancel all commands
        this.driverController.b().onTrue(CommandGenerators.CancelAllCommands());

        // TODO: make sure whileTrue works
        // Left Bumper -> Aim to home side to Ferry and start up Shooter
        this.driverController.leftBumper().whileTrue(CommandGenerators.PrepareFerry());
        // Right Bumper -> Aim to Hub and start up Shooter
        this.driverController.rightBumper().whileTrue(CommandGenerators.PrepareHub());
    }

    /** Configures the button bindings of the operator controller. */
    public void configureOperatorBindings() {
        // Double Rectangle (Left) -> Reset pose
        this.driverController.back().onTrue(CommandGenerators.ResetOdometryToOriginCommand());
        // Burger (Right) -> Calibrate QuestNav
        this.operatorController.start().onTrue(new CalibrateQuestCommand());  // TODO: comment this out before comp.

        // B -> Cancel all commands
        this.operatorController.b().onTrue(CommandGenerators.CancelAllCommands());

        // Left Bumper -> Intake
        this.operatorController.leftBumper().whileTrue(new IntakeCommand());

        // Right Bumper -> Feed Fuel into Shooter
        this.operatorController.rightBumper().whileTrue(new FeedShooterCommand());

        // Temporary
        // Left Trigger -> Shooter Hood Minimum
        this.operatorController.leftTrigger().onTrue(new MoveHoodCommand(HoodConstants.HOOD_ANGLE_MIN));
        // Right Trigger -> Shooter Hood Maximum
        this.operatorController.rightTrigger().onTrue(new MoveHoodCommand(HoodConstants.HOOD_ANGLE_MAX));

        // D-Pad Up -> Enter Climb
        this.operatorController.povUp().onTrue(new ClimbCommand());
        // D-Pad Down -> Leave Climb
        this.operatorController.povDown().onTrue(new LeaveClimbCommand());

        this.operatorController.a().onTrue(new MovePivotCommand(IntakeConstants.MAXIMUM_ANGLE));
        this.operatorController.y().onTrue(new MovePivotCommand(IntakeConstants.MINIMUM_ANGLE));
    }

    private void registerNamedCommands() {
        // Climb
        NamedCommands.registerCommand("Climb", new ClimbCommand());
        NamedCommands.registerCommand("LeaveClimb", new LeaveClimbCommand());

        // Intake
        NamedCommands.registerCommand("Intake", new IntakeCommand());
        NamedCommands.registerCommand("MovePivotDown", new MovePivotCommand(IntakeConstants.MAXIMUM_ANGLE));
        NamedCommands.registerCommand("MovePivotUp", new MovePivotCommand(IntakeConstants.MINIMUM_ANGLE));

        // Shooter
        NamedCommands.registerCommand("PrepareFerry", CommandGenerators.PrepareFerry());
        NamedCommands.registerCommand("PrepareHub", CommandGenerators.PrepareHub());
        NamedCommands.registerCommand("FeedShooter", CommandGenerators.FeedShooter());
    }

    public Command getAutonomousCommand() {
        if (this.auton == null) {
            this.auton = this.autoChooser.getSelected();
        }
        return this.auton;
    }
}