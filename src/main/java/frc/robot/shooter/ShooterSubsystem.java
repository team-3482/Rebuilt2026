// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.constants.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

// Servo code from https://github.com/wcpllc/2026CompetitiveConcept/

/** Controls the Shooter */
public class ShooterSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class ShooterSubsystemHolder {
        private static final ShooterSubsystem INSTANCE = new ShooterSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static ShooterSubsystem getInstance() {
        return ShooterSubsystemHolder.INSTANCE;
    }

    private final TalonFX shooterMotor1 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_1, RobotConstants.CAN_BUS);
    private final TalonFX shooterMotor2 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_2, RobotConstants.CAN_BUS);
    private final TalonFX shooterMotor3 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_3, RobotConstants.CAN_BUS);
    private final TalonFX feederMotor = new TalonFX(ShooterConstants.FEEDER_MOTOR, RobotConstants.CAN_BUS);
    private final TalonFX sterilizerMotor = new TalonFX(ShooterConstants.STERILIZER_MOTOR, RobotConstants.CAN_BUS);
    private final Servo hoodAngleServo1 = new Servo(ShooterConstants.HOOD_SERVO_1);
    private final Servo hoodAngleServo2 = new Servo(ShooterConstants.HOOD_SERVO_2);

    private double currentPosition = 0.0;
    private double targetPosition = 0.0;
    private Time lastUpdateTime = Seconds.of(0);

    private ShooterSubsystem() {
        super("ShooterSubsystem");

        shooterMotor1.setControl(new Follower(shooterMotor3.getDeviceID(), MotorAlignmentValue.Opposed));
        shooterMotor2.setControl(new Follower(shooterMotor3.getDeviceID(), MotorAlignmentValue.Opposed));

        hoodAngleServo1.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        hoodAngleServo2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        setHoodPosition(currentPosition);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/ShooterVelocity", getShooterVelocity().in(Units.RPM));
        Logger.recordOutput("Shooter/FeederVelocity", feederMotor.getVelocity().getValue().in(Units.RPM));
        Logger.recordOutput("Shooter/SterilizerVelocity", sterilizerMotor.getVelocity().getValue().in(Units.RPM));

        SmartDashboard.putBoolean("Shooter/AtShootingVelocityThreshold", atShootingVelocityThreshold());
        Logger.recordOutput("Shooter/AtShootingVelocityThreshold", atShootingVelocityThreshold());

        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isHoodPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = ShooterConstants.MAX_SERVO_SPEED.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(ShooterConstants.SERVO_LENGTH).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);

        Logger.recordOutput("Shooter/HoodPosition", currentPosition);
        SmartDashboard.putNumber("Shooter/HoodPosition", currentPosition);

    }

    /**
     * Set the speed of the shooter motors
     * @param speed the speed from -1 to 1
     */
    public void setShooterSpeed(double speed) {
        shooterMotor3.set(speed);
    }

    /**
     * Set the speed of the feeder motor
     * @param speed the speed from -1 to 1
     */
    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }

    /**
     * Set the speed of the sterilizer motor
     * @param speed the speed from -1 to 1
     */
    public void setSterilizerSpeed(double speed) {
        sterilizerMotor.set(speed);
    }

    /**
     * Get the velocity of the shooter motors
     * @return the velocity
     */
    public AngularVelocity getShooterVelocity() {
        return shooterMotor3.getVelocity().getValue();
    }

    /**
     * Check for if shooter motors are revved up to shooting velocity
     * @return true if shooter velocity is within threshold
     */
    public boolean atShootingVelocityThreshold() {
        return getShooterVelocity().in(Units.RPM) >= ShooterConstants.VELOCITY_THRESHOLD.magnitude();
    }

    /**
     * Convert the angle to servo value from 0.0 to 1.0
     * @param angle within minimum and maximum set in {@link ShooterConstants}
     */
    public double hoodAngleToDouble(Angle angle) {
        return MathUtil.clamp((angle.in(Degrees) - ShooterConstants.HOOD_ANGLE_MIN.in(Degrees))
                / (ShooterConstants.HOOD_ANGLE_MAX.in(Degrees) - ShooterConstants.HOOD_ANGLE_MIN.in(Degrees)),
            0, 1);
    }

    /**
     *  Set the position of the Hood
     * @param position from 0.0 to 1.0
     */
    public void setHoodPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, 0, 1);
        hoodAngleServo1.set(clampedPosition);
        hoodAngleServo2.set(clampedPosition);
        targetPosition = clampedPosition;
    }

    /**
     * Set position of Hood using an angle
     * @param angle Between Minimum and Maximum from {@link ShooterConstants}
     */
    public void setHoodAngle(Angle angle) {
        setHoodPosition(hoodAngleToDouble(angle));
    }

    /**
     * Finds if Hood is actively within tolerance to target position
     * @return whether it's within tolerance
     */
    public boolean isHoodPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, 0.01);
    }
}