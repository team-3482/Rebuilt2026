// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.constants.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

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

    private ShooterSubsystem() {
        super("ShooterSubsystem");

        shooterMotor1.setControl(new Follower(shooterMotor3.getDeviceID(), MotorAlignmentValue.Opposed));
        shooterMotor2.setControl(new Follower(shooterMotor3.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/ShooterVelocity", getShooterVelocity().in(Units.RPM));
        Logger.recordOutput("Shooter/FeederVelocity", feederMotor.getVelocity().getValue().in(Units.RPM));
        Logger.recordOutput("Shooter/SterilizerVelocity", sterilizerMotor.getVelocity().getValue().in(Units.RPM));

        SmartDashboard.putBoolean("Shooter/AtShootingVelocityThreshold", atShootingVelocityThreshold());
        Logger.recordOutput("Shooter/AtShootingVelocityThreshold", atShootingVelocityThreshold());
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
}