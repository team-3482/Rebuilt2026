package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RPM;

/**
 * Tunable Constants that aren't based on physical attributes of the robot.
 * For example, motor speeds, PID gains, timings, positions, etc.
 * */
public class VirtualConstants {
    /** Contants relating to the driver dashboard (Elastic) */
    public static final class Dashboard {
        /** Tab to show while humans are driving */
        public static final String TELEOP_TAB = "Teleoperated";
        /** Tab to show during autonomous period at beginning of match */
        public static final String AUTON_TAB = "Autonomous";
        /** Extra tab for debugging/development */
        public static final String DEV_TAB = "Dev";
    }

    /** Constants for the controller and any controller-related assignments. */
    public static final class Controller {
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

        /** Maximum allowed AprilTag ambiguity value */
        public static final double MAX_APRILTAG_AMBIGUITY = 1.0; // TODO: tune
    }

    /** Constants for Intake & Intake Pivot */
    public static final class Intake {
        // TODO: fill in values
        /* Motion Magic Config */
        public static final double CRUISE_SPEED = 0;
        public static final double ACCELERATION = 0;

        /** Gains used for Motion Magic slot 0. */
        public static final class Slot0Gains {
            public static final double kG = 0;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    /** Constants for Shooter */
    public static final class Shooter {
        /** Shooter motor speed */
        public static final double SHOOTING_SPEED = 1.0;
        /** Feeder motor speed */
        public static final double FEEDER_SPEED = 1.0;

        /** Shooter velocity (RPM) threshold to start feeding fuel */
        public static final AngularVelocity VELOCITY_THRESHOLD = RPM.of(5000);
    }

}