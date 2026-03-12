// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.RobotConstants;

/** Controls Intake and Intake Pivot */
public class IntakeSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class IntakeSubsystemHolder {
        private static final IntakeSubsystem INSTANCE = new IntakeSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static IntakeSubsystem getInstance() {
        return IntakeSubsystemHolder.INSTANCE;
    }

    private TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR, RobotConstants.CAN_BUS);
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR, RobotConstants.CAN_BUS);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private IntakeSubsystem() {
        super("IntakeSubsystem");

        this.configureMotor();
        this.setPivotPosition(0);

        this.pivotMotor.getPosition().setUpdateFrequency(50);
    }

    // TODO: telemetry to logs and dashboard
    @Override
    public void periodic() {}

    /* Configures pivot motor since it is the only one using motion magic. */
    private void configureMotor() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ration from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = IntakeConstants.ROTOR_TO_MECHANISM_RATIO;

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Configs.kG = IntakeConstants.Slot0Gains.kG;
        slot0Configs.kS = IntakeConstants.Slot0Gains.kS;
        slot0Configs.kV = IntakeConstants.Slot0Gains.kV;
        slot0Configs.kA = IntakeConstants.Slot0Gains.kA;
        slot0Configs.kP = IntakeConstants.Slot0Gains.kP;
        slot0Configs.kI = IntakeConstants.Slot0Gains.kI;
        slot0Configs.kD = IntakeConstants.Slot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.ACCELERATION;

        this.pivotMotor.getConfigurator().apply(configuration);
    }
    
     /**
      * Goes to a position using Motion Magic slot 0.
      * @param position The position for the pivot in degrees.
      * @param clamp Whether to clamp with the soft limits.
      * @apiNote The soft limits in {@link IntakeConstants}.
      */
    public void motionMagicPosition(double position, boolean clamp) {
        if (clamp) {
            position = MathUtil.clamp(position, IntakeConstants.LOWER_ANGLE_LIMIT, IntakeConstants.UPPER_ANGLE_LIMIT);
        }

        MotionMagicVoltage control = motionMagicVoltage
            .withSlot(0)
            .withPosition(Units.degreesToRotations(position));

        this.pivotMotor.setControl(control);
    }

    /**
     * Goes to a position using Motion Magic slot 0.
     * @param position The position for the pivot in degrees.
     * @apiNote The position is clamped by the soft limits in {@link IntakeConstants}.
     */
    public void motionMagicPosition(double position) {
        motionMagicPosition(position, true);
    }

    /**
     * Gets the mechanism position of the motor.
     * @return The angle in degrees
     */
    public double getPosition() {
        return Units.rotationsToDegrees(this.pivotMotor.getPosition().getValueAsDouble());
    }

    /**
     * Checks if the current position is within
     * {@link IntakeConstants#POSITION_TOLERANCE} of an input position.
     * @param position The position to compare to in rot.
     */
    public boolean withinTolerance(double position) {
        return Math.abs(getPosition() - position) <= IntakeConstants.PIVOT_TOLERANCE;
    }

     /**
     * Sets the mechanism position of the pivot motor.
     * @param position The position in degrees.
     */
    public void setPivotPosition(double position) {
        position = Units.degreesToRotations(position);
        this.pivotMotor.setPosition(position);
    }

    /**
     * Sets the speed of the intake motor.
     * @param speed How fast the motor goes. Must be between -1 and 1.
     */
    public void setIntakeSpeed(double speed) {
        this.intakeMotor.set(-speed);
    }
}