// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.AutoAngleConstants;
import frc.robot.constants.TunerConstants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

/** Takes a position on the field and automatically rotates to face it */
public class LookAtPositionCommand extends Command {
    Pose2d target;

    public LookAtPositionCommand(Pose2d target) {
        setName("LookAtPositionCommand");

        this.target = target;

        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(AutoAngleConstants.TOLERANCE.in(Radians));

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    SwerveDriveState state;
    Angle angleToTarget;
    Angle currentRobotAngle;
    Distance xDistance;
    Distance yDistance;

    private final SwerveRequest.FieldCentricFacingAngle facingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(TunerConstants.kSpeedAt12Volts.magnitude() * 0.035)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final PhoenixPIDController controller = new PhoenixPIDController(AutoAngleConstants.P, AutoAngleConstants.I, AutoAngleConstants.D);
    @Override
    public void initialize() {
        state = SwerveSubsystem.getInstance().getState();

        controller.reset();
        facingAngleDrive.HeadingController = controller;

        currentRobotAngle = SwerveSubsystem.getInstance().getState().Pose.getRotation().getMeasure();
        calculateAngle();
    }

    @Override
    public void execute() {
        SwerveSubsystem.getInstance().setControl(facingAngleDrive
            .withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(new Rotation2d(angleToTarget))
        );

        SwerveSubsystem.getInstance().setTargetAngle(angleToTarget);
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            System.out.println("Swerve Auto Angle command interrupted.");
        } else {
            System.out.println("Swerve angle within tolerance.");
        }
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    private void calculateAngle(){
        xDistance = Meters.of(target.getMeasureX().in(Meters) - state.Pose.getMeasureX().in(Meters));
        yDistance = Meters.of(target.getMeasureY().in(Meters) - state.Pose.getMeasureY().in(Meters));

        angleToTarget = Radians.of(Math.atan(yDistance.in(Meters) / xDistance.in(Meters)));
    }
}