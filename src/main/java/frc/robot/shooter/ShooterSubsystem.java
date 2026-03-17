// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CalculationConstants;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.constants.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

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
        Logger.recordOutput("Shooter/ShooterVelocity", getShooterVelocity().in(RPM));
        Logger.recordOutput("Shooter/FeederVelocity", feederMotor.getVelocity().getValue().in(RPM));
        Logger.recordOutput("Shooter/SterilizerVelocity", sterilizerMotor.getVelocity().getValue().in(RPM));

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
     * Calculate the desired fuel velocity based on distance from the hub
     * @param distance distance from the hub
     * @return the desired velocity of the fuel
     */
    public LinearVelocity calculateDesiredFuelVelocity(Distance distance){
        double minDist = CalculationConstants.MIN_SHOOTING_DISTANCE.in(Meters);
        double maxDist = CalculationConstants.MAX_SHOOTING_DISTANCE.in(Meters);
        double ratio = (distance.in(Meters) - minDist)/(maxDist - minDist);

        double minV = CalculationConstants.MIN_SHOOTING_VELOCITY.in(MetersPerSecond);
        double maxV = CalculationConstants.MAX_SHOOTING_VELOCITY.in(MetersPerSecond);

        return MetersPerSecond.of((ratio * (maxV - minV)) + minV);
    }

    /**
     * Get the velocity of the shooter motors
     * @return the velocity
     */
    public AngularVelocity getShooterVelocity() {
        return shooterMotor3.getVelocity().getValue();
    }

    /**
     * Sets the angular velocity of the shooter motors based on a desired fuel launch velocity
     * @param desiredFuelVelocity the desired velocity of the fuel projectile
     */
    public void setFuelLinearVelocity(LinearVelocity desiredFuelVelocity){
        double ratio = desiredFuelVelocity.in(MetersPerSecond) / getFuelLinearVelocity().in(MetersPerSecond);
        shooterMotor3.set(getShooterVelocity().in(RadiansPerSecond) * ratio);
    }

    /**
     * Check for if shooter motors are revved up to shooting velocity
     * @return true if shooter velocity is within threshold
     */
    public boolean atShootingVelocityThreshold() {
        return getShooterVelocity().in(RPM) >= ShooterConstants.VELOCITY_THRESHOLD.in(RPM);
    }

    /**
     * Calculate the Angular Velocity of the shot fuel using a planetary gearbox formula.
     * @return The Angular Velocity.
     */
    public AngularVelocity getFuelAngularVelocity() {
        double currentVelocity = getShooterVelocity().in(RPM);
        return RPM.of(
            // use threshold number by default, but use real velocity
            // if at threshold so it can correct the angle if necessary
            (atShootingVelocityThreshold() ? currentVelocity : ShooterConstants.VELOCITY_THRESHOLD.in(RPM))
            * CalculationConstants.VELOCITY_RATIO
        );
    }

    /**
     * Calculate the Linear Velocity of the shot fuel from the Angular Velocity.
     * @return The Linear Velocity.
     */
    public LinearVelocity getFuelLinearVelocity() {
        Distance radius = Inches.of(
            (CalculationConstants.FUEL_DIAMETER.in(Inches) / 2) + (CalculationConstants.WHEEL_DIAMETER.in(Inches) / 2)
        );

        return MetersPerSecond.of(getFuelAngularVelocity().in(RadiansPerSecond) * radius.in(Meters));
    }
}