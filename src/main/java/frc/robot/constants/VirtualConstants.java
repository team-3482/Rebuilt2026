package frc.robot.constants;

/**
 * Tunable Constants that aren't based on physical attributes of the robot.
 * For example, motor speeds, PID gains, timings, positions, etc.
 * */
public class VirtualConstants {
    /** Constants for the controller and any controller-related assignments. */
    public static final class ControllerConstants {
        /** DriverStation ID of the driver controller. */
        public static final int DRIVER_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller. */
        public static final int OPERATOR_CONTROLLER_ID = 1;

        /** Removes input around the joystick's center (eliminates stick drift). */
        public static final double DEADBAND = 0.075;
        
        /** Speed multiplier when using fine-control movement. */
        public static final double FINE_CONTROL_MULT = 0.15;
    }

    // Future classes: VirtualConstants.Shooter, VirtualConstants.Intake, etc.

}