// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
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

    @SuppressWarnings("FieldCanBeLocal")
    private final TalonFX shooterMotor1 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_1, RobotConstants.CAN_BUS);
    @SuppressWarnings("FieldCanBeLocal")
    private final TalonFX shooterMotor2 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_2, RobotConstants.CAN_BUS);
    private final TalonFX shooterMotor3 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_3, RobotConstants.CAN_BUS);
    private final TalonFX feederMotor = new TalonFX(ShooterConstants.FEEDER_MOTOR, RobotConstants.CAN_BUS);
    private final TalonFX sterilizerMotor = new TalonFX(ShooterConstants.STERILIZER_MOTOR, RobotConstants.CAN_BUS);

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    private AngularVelocity lastTargetVelocity = RPM.of(Double.MAX_VALUE);

    private ShooterSubsystem() {
        super("ShooterSubsystem");

        this.configureMotors();

        this.shooterMotor3.getPosition().setUpdateFrequency(50);
        this.shooterMotor3.getVelocity().setUpdateFrequency(50);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/ShooterVelocity", getShooterVelocity().in(RPM));
        Logger.recordOutput("Shooter/FeederVelocity", feederMotor.getVelocity().getValue().in(RPM));
        Logger.recordOutput("Shooter/SterilizerVelocity", sterilizerMotor.getVelocity().getValue().in(RPM));
        Logger.recordOutput("Shooter/TargetVelocity", lastTargetVelocity.in(RPM));

        SmartDashboard.putBoolean("Shooter/AtShootingVelocityThreshold", atShootingVelocityThreshold());
        Logger.recordOutput("Shooter/AtShootingVelocityThreshold", atShootingVelocityThreshold());
    }

    /**
     * A helper method that configures MotionMagic on both motors.
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.kS = ShooterConstants.Slot0Gains.kS;
        slot0Configs.kV = ShooterConstants.Slot0Gains.kV;
        slot0Configs.kP = ShooterConstants.Slot0Gains.kP;
        slot0Configs.kI = ShooterConstants.Slot0Gains.kI;
        slot0Configs.kD = ShooterConstants.Slot0Gains.kD;

        // this.shooterMotor1.getConfigurator().apply(configuration);
        // this.shooterMotor2.getConfigurator().apply(configuration);
        this.shooterMotor3.getConfigurator().apply(configuration);
    }

    /**
     * Set the speed of the shooter motors
     * @param speed the speed from -1 to 1
     */
    public void setShooterSpeed(double speed) {
        // shooterMotor1.set(-speed);
        // shooterMotor2.set(-speed);
        shooterMotor3.set(speed);
    }

    /**
     * Sets the angular velocity of the shooter motors with PID
     * @param targetAngularVelocity the target angular velocity for the shooter motors.
     */
    public void motionMagicAngularVelocity(AngularVelocity targetAngularVelocity){
        lastTargetVelocity = targetAngularVelocity;
        System.out.println(targetAngularVelocity);
        // shooterMotor1.setControl(velocityVoltage.withVelocity(targetAngularVelocity.times(-1)).withFeedForward(0.5));
        // shooterMotor2.setControl(velocityVoltage.withVelocity(targetAngularVelocity.times(-1)).withFeedForward(0.5));
        shooterMotor3.setControl(velocityVoltage.withVelocity(1));
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
    public AngularVelocity desiredMotorAngularVelocity(Distance distance) {
        return AngularVelocity.ofBaseUnits(
            (desiredFuelVelocity(distance).in(MetersPerSecond) * Constants.CalculationConstants.FUEL_LINEAR_TO_MOTOR_ANGULAR_VELOCITY_RATIO), RPM
        );
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
            (atShootingVelocityThreshold() && currentVelocity > 0 ? currentVelocity : desiredMotorAngularVelocity(distance).in(RPM))
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