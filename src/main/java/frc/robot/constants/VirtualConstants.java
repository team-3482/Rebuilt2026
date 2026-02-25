package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

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

    /** Constants for QuestNav and Limelight */
    public static final class Vision {
        /** Standard Deviations for Trust with QuestNav */
        public static final Matrix<N3, N1> QUESTNAV_TRUST_STD_DEVS = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            Units.degreesToRadians(2) // Trust down to 2 degrees rotational
        );

        /** Standard Deviations for Trust with QuestNav */
        public static final Matrix<N3, N1> LIMELIGHT_TRUST_STD_DEVS = VecBuilder.fill(
            0.5, // Trust down to 50cm in X direction
            0.5, // Trust down to 50cm in Y direction
            9999999 // Trust all rotational data
        );
    }

}