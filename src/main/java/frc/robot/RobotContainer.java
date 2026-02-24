// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.constants.TunerConstants;
import frc.robot.constants.VirtualConstants.ControllerConstants;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.SwerveTelemetry;
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

    // private final SendableChooser<Command> autoChooser; // TODO: pathplanner
    private Command auton = null;

    // Instance of the controllers used to drive the robot
    private final CommandXboxController driverController;
    private final XboxController driverController_HID;
    private final CommandXboxController operatorController;

    public RobotContainer() {
        // Initialize controllers
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.driverController_HID = this.driverController.getHID();
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

        configureDrivetrain();
        initializeSubsystems();

        // TODO: pathplanner
        // this.autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        // this.autoChooser.onChange((Command autoCommand) -> this.auton = autoCommand); // Reloads the stored auto

        // SmartDashboard.putData("Auto Chooser", this.autoChooser);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    /**
     * Creates instances of each subsystem so periodic runs on startup.
     */
    @SuppressWarnings("ResultOfMethodCallIgnored")
    private void initializeSubsystems() {
    } // TODO: add future subsystems here

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

                double fineControlMult = fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1;

                return fieldCentricDrive_withDeadband
                    // Drive forward with negative Y (forward)
                    .withVelocityX(-driverController.getLeftY() * linearSpeed * fineControlMult)
                    // Drive left with negative X (left)
                    .withVelocityY(-driverController.getLeftX() * linearSpeed * fineControlMult)
                    // Drive counterclockwise with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * angularSpeed * fineControlMult)

                    .withDeadband(ControllerConstants.DEADBAND * linearSpeed)
                    .withRotationalDeadband(ControllerConstants.DEADBAND * angularSpeed);
            }).ignoringDisable(true)
        );

        // POV / D-PAD
        final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        if (ControllerConstants.DPAD_DRIVE_INPUT) {
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
        }

        Drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Configures the button bindings of the driver controller.
     */
    public void configureDriverBindings() {
        // Double Rectangle -> Reset pose
        this.driverController.back().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().resetPose(Pose2d.kZero)));
        // Burger -> Reset rotation to zero
        this.driverController.start().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().seedFieldCentric()));
    }

    /**
     * Configures the button bindings of the operator controller.
     */
    public void configureOperatorBindings() {
        // B -> Cancel all commands
        this.operatorController.b().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
    }

    public Command getAutonomousCommand() {
        if (this.auton == null) {
            this.auton = null; //this.autoChooser.getSelected(); // TODO pathplanner
        }
        return this.auton;
    }
}