// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CalculationConstants;
import frc.robot.constants.Constants.HoodConstants;
import frc.robot.constants.Constants.PositionConstants;
import frc.robot.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

// Servo code from https://github.com/wcpllc/2026CompetitiveConcept/

/** Controls the Shooter Hood */
public class HoodSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class HoodSubsystemHolder {
        private static final HoodSubsystem INSTANCE = new HoodSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static HoodSubsystem getInstance() {
        return HoodSubsystemHolder.INSTANCE;
    }

    private final Servo servo1 = new Servo(HoodConstants.HOOD_SERVO_1);
    private final Servo servo2 = new Servo(HoodConstants.HOOD_SERVO_2);

    private double currentPosition = 0.0;
    private double targetPosition = 0.0;
    private Time lastUpdateTime = Seconds.of(0);

    private HoodSubsystem() {
        super("HoodSubsystem");

        servo1.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        servo2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        setHoodPosition(currentPosition);
    }

    @Override
    public void periodic() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = HoodConstants.MAX_SERVO_SPEED.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(HoodConstants.SERVO_LENGTH).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);

        boolean inputToggled = SmartDashboard.getBoolean("Hood/ToggleInputSlider", false);

        if(!inputToggled) {
            SmartDashboard.putNumber("Hood/Position", currentPosition);
        }

        if (DriverStation.isEnabled()) {
            Command currentCommand = getCurrentCommand();

            if (currentCommand != null) {
                SmartDashboard.putBoolean("Hood/ToggleInputSlider", false);
            } else if (inputToggled && currentCommand == null) {
                setHoodAngle(Degrees.of(SmartDashboard.getNumber("Hood/Position", HoodConstants.HOOD_ANGLE_MIN.in(Degrees))));
            }
        } else {
            SmartDashboard.putBoolean("Hood/ToggleInputSlider", false);
        }

        Logger.recordOutput("Hood/Position", currentPosition);
    }

    /**
     * Convert the angle to servo value from 0.0 to 1.0
     * @param angle within minimum and maximum set in {@link HoodConstants}
     */
    public double hoodAngleToDouble(Angle angle) {
        return MathUtil.clamp((angle.in(Degrees) - HoodConstants.HOOD_ANGLE_MIN.in(Degrees))
                / (HoodConstants.HOOD_ANGLE_MAX.in(Degrees) - HoodConstants.HOOD_ANGLE_MIN.in(Degrees)),
            0, 1);
    }

    /**
     *  Set the position of the Hood
     * @param position from 0.0 to 1.0
     */
    public void setHoodPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, 0, 1);
        servo1.set(clampedPosition);
        servo2.set(clampedPosition);
        targetPosition = clampedPosition;
    }

    /**
     * Set position of Hood using an angle
     * @param angle Between Minimum and Maximum from {@link HoodConstants}
     */
    public void setHoodAngle(Angle angle) {
        setHoodPosition(hoodAngleToDouble(angle));
    }

    /**
     * Gives the angle at which the hood should be set to shoot a fuel element into the hub.
     * @param distance The distance from the bot to the center of the Hub.
     * @return The angle at which the hood should be set.
     */
    public Angle getShootingHoodAngle(Distance distance){
        double v = Math.abs((ShooterSubsystem.getInstance().getFuelLinearVelocity()).in(MetersPerSecond));
        double g = CalculationConstants.GRAV.in(MetersPerSecondPerSecond);
        double d = distance.in(Meters);

        return Radians.of(Math.atan(( // https://www.desmos.com/calculator/moewwoi4pa :)
            Math.pow(v, 2)
            + Math.sqrt(
                Math.pow(v, 4)
                - g * (
                    g * Math.pow(d, 2)
                    + 2 * PositionConstants.HUB_HEIGHT.in(Meters) * Math.pow(v, 2)
                )
            )) / (g * d)
        ));
    }

    /**
     * Finds if Hood is actively within tolerance to target position
     * @return whether it's within tolerance
     */
    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, 0.01);
    }
}