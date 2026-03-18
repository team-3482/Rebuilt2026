// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CalculationConstants;
import frc.robot.constants.Constants.IntakeConstants;
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

    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0);

    private AngularVelocity lastTargetVelocity = RPM.of(0);

    private ShooterSubsystem() {
        super("ShooterSubsystem");

        this.configureMotors();

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
     * A helper method that configures MotionMagic on both motors.
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ratio from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = ShooterConstants.ROTOR_TO_MECHANISM_RATIO;

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.kG = ShooterConstants.Slot0Gains.kG;
        slot0Configs.kS = ShooterConstants.Slot0Gains.kS;
        slot0Configs.kV = ShooterConstants.Slot0Gains.kV;
        slot0Configs.kA = ShooterConstants.Slot0Gains.kA;
        slot0Configs.kP = ShooterConstants.Slot0Gains.kP;
        slot0Configs.kI = ShooterConstants.Slot0Gains.kI;
        slot0Configs.kD = ShooterConstants.Slot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.ACCELERATION;

        this.shooterMotor3.getConfigurator().apply(configuration);
    }

    /**
     * Set the speed of the shooter motors
     * @param speed the speed from -1 to 1
     */
    public void setShooterSpeed(double speed) {
        shooterMotor3.set(speed);
    }

    /**
     * Sets the angular velocity of the shooter motors with PID
     * @param targetAngularVelocity the target angular velocity for the shooter motors. 
     */
    public void motionMagicAngularVelocity(AngularVelocity targetAngularVelocity){
        lastTargetVelocity = targetAngularVelocity;
        shooterMotor3.setControl(motionMagicVelocityVoltage.withVelocity(targetAngularVelocity));
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
     * Calculate the desired fuel velocity based on distance from the target
     * @param distance distance from the target
     * @return the desired velocity of the fuel
     */
    public LinearVelocity desiredFuelVelocity(Distance distance){
        double minDist = CalculationConstants.MIN_SHOOTING_DISTANCE.in(Meters);
        double maxDist = CalculationConstants.MAX_SHOOTING_DISTANCE.in(Meters);
        double ratio = (distance.in(Meters) - minDist)/(maxDist - minDist);

        double minV = CalculationConstants.MIN_SHOOTING_VELOCITY.in(MetersPerSecond);
        double maxV = CalculationConstants.MAX_SHOOTING_VELOCITY.in(MetersPerSecond);

        return MetersPerSecond.of((ratio * (maxV - minV)) + minV);
    }

    /**
     * Returns the target angular velocity for the shooter motors given the distance from a target. 
     * @param distance the distance from the target.
     * @return The desired/target motor angular velocity.
     */
    public AngularVelocity desiredMotorAngularVelocity(Distance distance){
        AngularVelocity desiredRPM = 
            AngularVelocity.ofBaseUnits(
                (desiredFuelVelocity(distance).in(MetersPerSecond) * Constants.CalculationConstants.FUEL_LINEAR_TO_MOTOR_ANGULAR_VELOCITY_RATIO), RPM);
        return desiredRPM;
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
     * @param distance The distance of the bot from the target
     * @return true if shooter velocity is within threshold
     */
    public boolean atShootingVelocityThreshold() {
        double difference = (getShooterVelocity().in(RadiansPerSecond) - lastTargetVelocity.in(RPM));
        return Math.abs(difference) <= ShooterConstants.VELOCITY_TOLERANCE.in(RPM);
    }

    /**
     * Calculate the Angular Velocity of the shot fuel using a planetary gearbox formula.
     * @return The Angular Velocity.
     */
    public AngularVelocity getFuelAngularVelocity(Distance distance) {
        double currentVelocity = getShooterVelocity().in(RPM);
        return RPM.of(
            // use threshold number by default, but use real velocity
            // if at threshold so it can correct the angle if necessary
            (atShootingVelocityThreshold() ? currentVelocity : desiredMotorAngularVelocity(distance).in(RPM))
            * CalculationConstants.VELOCITY_RATIO
        );
    }

    /**
     * Calculate the Linear Velocity of the shot fuel from the Angular Velocity.
     * @return The Linear Velocity.
     */
    public LinearVelocity getFuelLinearVelocity(Distance distance) {
        Distance radius = Inches.of(
            (CalculationConstants.FUEL_DIAMETER.in(Inches) / 2) + (CalculationConstants.WHEEL_DIAMETER.in(Inches) / 2)
        );

        return MetersPerSecond.of(getFuelAngularVelocity(distance).in(RadiansPerSecond) * radius.in(Meters));
    }
}