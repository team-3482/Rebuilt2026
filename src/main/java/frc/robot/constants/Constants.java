package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

public class Constants {
    /** General specific constants about the robot. */
    public static final class RobotConstants {
        /** CAN Bus for all CTRE devices */
        public static final CANBus CAN_BUS = new CANBus("ctre");
    }

    /** Constants for the controller and dashboard */
    public static final class DriverStationConstants {
        /** DriverStation ID of the driver controller. */
        public static final int DRIVER_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller. */
        public static final int OPERATOR_CONTROLLER_ID = 1;

        /** Removes input around the joystick's center (eliminates stick drift). */
        public static final double DEADBAND = 0.075;
        /** Speed multiplier when using fine-control movement. */
        public static final double FINE_CONTROL_MULT = 0.15;

        /** Tab to show while humans are driving */
        public static final String TELEOP_TAB = "Teleoperated";
        /** Tab to show during autonomous period at beginning of match */
        public static final String AUTON_TAB = "Autonomous";
        /** Extra tab for debugging/development */
        public static final String DEV_TAB = "Dev";
    }

    /** Constants for QuestNav and Limelight */
    public static final class VisionConstants {
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

    /** Constants for intake subsystem (both the pivot and actual intake) */
    public static final class IntakeConstants {
        // TODO: fill in values
        /** The CAN ID for the Intake Pivot TalonFX */
        public static final int PIVOT_MOTOR = 22;
        /** The CAN ID for the Intake TalonFX */
        public static final int INTAKE_MOTOR = 20;

        /** The lower and upper angle limit for the pivot */
        public static final double LOWER_ANGLE_LIMIT = 0;
        public static final double UPPER_ANGLE_LIMIT = 0;

        /** Gear ratio for mechanism */
        public static final double ROTOR_TO_MECHANISM_RATIO = 0;

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

        /** Speed to run the intake motor at */
        public static final double INTAKE_SPEED = 0.2;
    }

    /** Constants for the Shooter subsystem */
    public static final class ShooterConstants {
        /** CAN ID for Shooter motor 1 */
        public static final int SHOOTER_MOTOR_1 = 23;
        /** CAN ID for Shooter motor 2 */
        public static final int SHOOTER_MOTOR_2 = 21;
        /** CAN ID for Shooter motor 3 */
        public static final int SHOOTER_MOTOR_3 = 24;
        /** CAN ID for Feeder motor */
        public static final int FEEDER_MOTOR = 25;
        /** CAN ID for Sterilizer motor */
        public static final int STERILIZER_MOTOR = 30;
        /** PWM Channel for Hood Linear Actuator 1 */
        public static final int HOOD_SERVO_1 = 0;
        /** PWM Channel for Hood Linear Actuator 2 */
        public static final int HOOD_SERVO_2 = 1;

        /** Shooter motor speed */
        public static final double SHOOTING_SPEED = 1.0;
        /** Feeder motor speed */
        public static final double FEEDER_SPEED = 1.0;
        /** Sterilizer motor speed */
        public static final double STERILIZER_SPEED = 0.15;

        /** Shooter velocity (RPM) threshold to start feeding fuel */
        public static final AngularVelocity VELOCITY_THRESHOLD = RPM.of(5000);

        /** Full Servo Length */
        public static final Distance SERVO_LENGTH = Millimeters.of(50);
        /** Max Servo speed to run at */
        public static final LinearVelocity MAX_SERVO_SPEED = Millimeters.of(32).per(Second);
        // TODO: find real limits
        /** Hood angle minimum */
        public static final Angle HOOD_ANGLE_MIN = Degrees.of(30);
        /** Hood angle maximum */
        public static final Angle HOOD_ANGLE_MAX = Degrees.of(60);
    }

    /** Constants for climb */
    public static final class ClimbConstants {
        /** CAN ID for Climb motor 1 */
        public static final int CLIMB_MOTOR_1 = 31;
        /** CAN ID for Climb motor 2 */
        public static final int CLIMB_MOTOR_2 = 32;

        /** How many revolutions to completely retract the climb */
        public static final Angle FULL_RETRACTION_REVOLUTIONS = Revolutions.of(43);

        /** Speed to climb at */
        public static final double CLIMB_SPEED = 0.5;
    }
}