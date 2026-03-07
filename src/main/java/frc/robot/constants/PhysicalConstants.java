package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

/**
 * Physical attributes that are related to real-life things that don't change and don't need to be tuned.
 * For example, device IDs, ratios/multipliers, etc.
 * */
public class PhysicalConstants {
    /** General specific constants about the robot. */
    public static final class Robot {
        /** CAN Bus for all CTRE devices */
        public static final CANBus CAN_BUS = new CANBus("ctre");
    }

    /** Constants for the controller and any controller-related assignments. */
    public static final class Controller {
        /** DriverStation ID of the driver controller. */
        public static final int DRIVER_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller. */
        public static final int OPERATOR_CONTROLLER_ID = 1;
    }

    /** Constants for QuestNav and Limelight */
    public static final class Vision {
        /** The name of the limelight */
        public static final String LIMELIGHT = "limelight-stheno";

        /** Quest mounting offsets */
        public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
            new Translation3d(
                Inches.of(0), // X -> Forward from robot center
                Inches.of(0), // Y -> Left from robot center
                Inches.of(0) // Z -> Up from robot center
            ),
            new Rotation3d(
                Degrees.of(0), // Yaw -> Counter-clockwise rotation on the Z axis
                Degrees.of(0), // Pitch -> Counter-clockwise rotation on the Y axis
                Degrees.of(0) // Roll -> Counter-clockwise rotation on the X axis
            )
        );
    }

    /** Constants for intake system (both the pivot and actual intake) */
    public static final class Intake {
        // TODO: fill in values
        /** The CAN ID for the Intake Pivot TalonFX */
        public static final int PIVOT_MOTOR_ID = 0;
        /** The CAN ID for the Intake Intake TalonFX */
        public static final int INTAKE_MOTOR_ID = 0;

        /** The lower and upper angle limit for the pivot */
        public static final double LOWER_ANGLE_LIMIT = 0;
        public static final double UPPER_ANGLE_LIMIT = 0;

        /** Gear ratio for mechanism */
        public static final double ROTOR_TO_MECHANISM_RATIO = 0;
    }
}