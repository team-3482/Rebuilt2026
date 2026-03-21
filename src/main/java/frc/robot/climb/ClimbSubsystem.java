// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ClimbConstants;
import frc.robot.constants.Constants.RobotConstants;
import org.littletonrobotics.junction.Logger;

/** An example subsystem that does nothing. */
public class ClimbSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class ClimbSubsystemHolder {
        private static final ClimbSubsystem INSTANCE = new ClimbSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static ClimbSubsystem getInstance() {
        return ClimbSubsystemHolder.INSTANCE;
    }

    private final TalonFX climbMotor1 = new TalonFX(ClimbConstants.CLIMB_MOTOR_1, RobotConstants.CAN_BUS);
    private final TalonFX climbMotor2 = new TalonFX(ClimbConstants.CLIMB_MOTOR_2, RobotConstants.CAN_BUS);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private ClimbSubsystem() {
        super("ClimbSubsystem");

        configureMotors();

        climbMotor2.setControl(new Follower(climbMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureMotors(){
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ration from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = ClimbConstants.ROTOR_TO_MECHANISM_RATIO;

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.kG = ClimbConstants.Slot0Gains.kG;
        slot0Configs.kS = ClimbConstants.Slot0Gains.kS;
        slot0Configs.kV = ClimbConstants.Slot0Gains.kV;
        slot0Configs.kA = ClimbConstants.Slot0Gains.kA;
        slot0Configs.kP = ClimbConstants.Slot0Gains.kP;
        slot0Configs.kI = ClimbConstants.Slot0Gains.kI;
        slot0Configs.kD = ClimbConstants.Slot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = ClimbConstants.ACCELERATION;

        this.climbMotor1.getConfigurator().apply(configuration);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climb/Position", getClimbPosition().in(Units.Revolutions));
    }

    /**
     * Get the position of the climb
     * @return the position in revolutions
     */
    public Angle getClimbPosition(){
        return climbMotor1.getPosition().getValue();
    }

    /**
     * Set the speed of the climb motors
     * @param speed the speed from -1 to 1
     */
    public void setClimbSpeed(double speed){
        climbMotor1.set(speed);
    }

    /**
     * Move the climb to a set angle
     * @param angle the angle to move to
     */
    public void setClimbPosition(Angle angle){
        MotionMagicVoltage control = motionMagicVoltage
            .withSlot(0)
            .withPosition(angle);

        this.climbMotor1.setControl(control);
    }
}