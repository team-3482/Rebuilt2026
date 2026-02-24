package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

/**
 * Physical attributes that are related to real-life things that don't change and don't need to be tuned.
 * For example, device IDs, ratios/multipliers, etc.
 * */
public class PhysicalConstants {
    public static final class RobotConstants {
        /** CAN Bus for all CTRE devices */
        public static final CANBus CAN_BUS = new CANBus("ctre");
    }

    // Future classes: PhysicalConstants.Shooter, PhysicalConstants.Intake, etc.
}