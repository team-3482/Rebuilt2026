package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

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
            new Translation3d( // values generated from https://www.desmos.com/calculator/ymzqtr2lip
                0.72010064, // X -> Forward from robot center
                -0.46832619, // Y -> Left from robot center
                0 // Z -> Up from robot center
            ),
            new Rotation3d(
                Degrees.of(180), // Yaw -> Counter-clockwise rotation on the Z axis
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
        /** The CAN ID for the Intake Pivot TalonFX */
        public static final int PIVOT_MOTOR = 22;
        /** The CAN ID for the Intake TalonFX */
        public static final int INTAKE_MOTOR = 20;

        /** Minimum angle (Up on hardstop inside robot) */
        public static final Angle MINIMUM_ANGLE = Degrees.of(0);
        /** Maximum angle (Intaking position on bumper) */
        public static final Angle MAXIMUM_ANGLE = Degrees.of(110); // TODO: real bumper angle

        /** Gear ratio for mechanism */
        public static final double ROTOR_TO_MECHANISM_RATIO = 6;

        // TODO: fill in values
        /* Motion Magic Config */
        public static final double CRUISE_SPEED = 2;
        public static final double ACCELERATION = 2;

        /** Gains used for Motion Magic slot 0. */
        public static final class Slot0Gains {
            public static final double kG = 0.075;
            public static final double kS = 0.09;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 256;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        /** The tolerance used for pivot in degrees  */
        public static final double PIVOT_TOLERANCE = 2; // TODO: make ts more accurate

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

        /** Shooter motor speed */
        public static final double SHOOTING_SPEED = 1.0;
        /** Feeder motor speed */
        public static final double FEEDER_SPEED = 1.0;
        /** Sterilizer motor speed */
        public static final double STERILIZER_SPEED = 0.15;

        /** Shooter velocity (RPM) threshold to start feeding fuel */
        public static final AngularVelocity VELOCITY_THRESHOLD = RPM.of(5000);

    }

    /** Constants for the Hood subsystem */
    public static final class HoodConstants {
        /** PWM Channel for Hood Linear Actuator 1 */
        public static final int HOOD_SERVO_1 = 0;
        /** PWM Channel for Hood Linear Actuator 2 */
        public static final int HOOD_SERVO_2 = 1;

        /** Full Servo Length */
        public static final Distance SERVO_LENGTH = Millimeters.of(50);
        /** Max Servo speed to run at */
        public static final LinearVelocity MAX_SERVO_SPEED = Millimeters.of(32).per(Second);
        /** Hood angle minimum */
        public static final Angle HOOD_ANGLE_MIN = Degrees.of(54.2);
        /** Hood angle maximum */
        public static final Angle HOOD_ANGLE_MAX = Degrees.of(69);
    }

    /** Constants for kinematics math */
    public static final class CalculationConstants {
        // https://engineerexcel.com/planetary-gear-calculation/
        /** Shooter wheel diameter */
        public static final Distance WHEEL_DIAMETER = Inches.of(3.965);
        /** Fuel ball diameter */
        public static final Distance FUEL_DIAMETER = Inches.of(5.91);
        /** Diameter of the hood (if it was a full circle) */
        public static final Distance RING_DIAMETER = Inches.of(15.155);

        /** Ratio of the diameter of the Ring to the diameter of the Wheel */
        public static final double RING_TO_WHEEL_RATIO = RING_DIAMETER.in(Inches) / WHEEL_DIAMETER.in(Inches) + 1;
        /** Ratio for converting Wheel angular velocity to Fuel angular velocity */
        public static final double VELOCITY_RATIO = 1 / RING_TO_WHEEL_RATIO;

        /** Acceleration of gravity */
        public static final LinearAcceleration GRAV = MetersPerSecondPerSecond.of(9.81);
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

    public static final class PositionConstants {
        /** Height of the Hub */
        public static final Distance HUB_HEIGHT = Feet.of(6);

        /** Position of the Blue side Hub */
        public static final Pose2d BLUE_HUB = new Pose2d(new Translation2d(4.5974, 4.034536), new Rotation2d());
        /** Position of the Red side Hub */
        public static final Pose2d RED_HUB = new Pose2d(new Translation2d(11.938, 4.034536), new Rotation2d());
    }
}