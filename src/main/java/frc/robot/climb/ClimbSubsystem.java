// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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

    private ClimbSubsystem() {
        super("ClimbSubsystem");

        climbMotor2.setControl(new Follower(climbMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
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
}