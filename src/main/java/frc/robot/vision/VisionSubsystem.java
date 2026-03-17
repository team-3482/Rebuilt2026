// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.geometry.struct.Translation2dStruct;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.LimelightHelpers.RawFiducial;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;

/** Subsystem that manages odometry and vision using a Limelight and QuestNav */
public class VisionSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class VisionSubsystemHolder {
        private static final VisionSubsystem INSTANCE = new VisionSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static VisionSubsystem getInstance() {
        return VisionSubsystemHolder.INSTANCE;
    }

    QuestNav questNav = new QuestNav();
    PoseFrame[] poseFrames;
    PoseEstimate limelightPose;

    RawFiducial[] fiducials;


    private VisionSubsystem() {
        super("VisionSubsystem");

        HttpCamera LLCamera = new HttpCamera(
            VisionConstants.LIMELIGHT,
            "http://" + "10.34.82.13" + ":5800/stream.mjpg" // TODO: this IP address is probably wrong
        );

        CameraServer.startAutomaticCapture(LLCamera);
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();

        boolean tracking = questNav.isTracking();
        Logger.recordOutput("QuestNav/Tracking", tracking);

        if (tracking) {
            // Get the latest pose data frames from the Quest
            poseFrames = questNav.getAllUnreadPoseFrames();

            updateSwervePoseEstimation();

            SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());
            SmartDashboard.putNumber("QuestNav/FramesPerRobotCycle", poseFrames.length);
            SmartDashboard.putNumber("QuestNav/BatteryPercent", questNav.getBatteryPercent().getAsInt());

            Logger.recordOutput("QuestNav/Latency", questNav.getLatency());
            Logger.recordOutput("QuestNav/FramesPerRobotCycle", poseFrames.length);
            Logger.recordOutput("QuestNav/BatteryPercent", questNav.getBatteryPercent().getAsInt());
        }

        limelightPose = getLimelightPose();
        // Logger.recordOutput("Limelight/Pose", limelightPose.pose);

        if(DriverStation.isDisabled()) {
            resetPose();
        }
    }

    /**
     * Get the latest QuestNav Pose3d
     * @return The QuestNav Pose3d
     */
    public Pose3d getPose3d() {
        if (poseFrames.length > 0) {
            // Get the most recent Quest pose
            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();

            // Transform by the mount pose to get your robot pose
            return questPose.transformBy(VisionConstants.ROBOT_TO_QUEST.inverse());
        }

        return null;
    }

    /**
     * Get the latest QuestNav Pose2d
     * @return The QuestNav Pose2d
     */
    public Pose2d getPose2d() {
        Pose3d pose3d = getPose3d();

        if(pose3d != null) {
            return pose3d.toPose2d();
        }

        return null;
    }

    /** Reset pose from absolute Limelight data if QuestNav relative data is inaccurate */
    public void resetPose(){
        // TODO: test if this works. I think it might not, and instead the limelight pose should be provided directly
        // if it does work, then maybe the limelight vision data should be added periodically
        SwerveSubsystem.getInstance().addVisionMeasurement(
            limelightPose.pose,
            limelightPose.timestampSeconds,
            VisionConstants.LIMELIGHT_TRUST_STD_DEVS
        );

        Pose3d currentPose = new Pose3d(SwerveSubsystem.getInstance().getState().Pose);
        questNav.setPose(currentPose.transformBy(VisionConstants.ROBOT_TO_QUEST));
    }

    /** Periodically adds vision measurements to the swerve odometry */
    private void updateSwervePoseEstimation() {
        for (PoseFrame poseFrame : poseFrames) {
            double timestamp = poseFrame.dataTimestamp();

            // Transform by the mount pose to get the robot pose
            Pose3d robotPose = poseFrame.questPose3d().transformBy(VisionConstants.ROBOT_TO_QUEST.inverse());

            // Add the measurement
            SwerveSubsystem.getInstance().addVisionMeasurement(robotPose.toPose2d(), timestamp, VisionConstants.QUESTNAV_TRUST_STD_DEVS);
        }
    }

    /**
     * Get the latest Limelight pose data
     * @return the pose estimate
     */
    private PoseEstimate getLimelightPose() {
        double rotationDegrees = SwerveSubsystem.getInstance().getState().Pose.getRotation().getDegrees();

        LimelightHelpers.SetRobotOrientation(
            VisionConstants.LIMELIGHT,
            rotationDegrees,
            0.0, 0.0, 0.0, 0.0, 0.0
        );

        fiducials = LimelightHelpers.getRawFiducials(VisionConstants.LIMELIGHT);

        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT);
    }

    /**
     * Getter method to know if an AprilTag is in view of the Limelight
     * @return if it's in view
     */
    private boolean tagInView(){
        return LimelightHelpers.getTV(VisionConstants.LIMELIGHT);
    }

    /**
     * Getter method to find the ambiguity value of currently visible AprilTags
     * @return the ambiguity value
     */
    private double getLimelightAmbiguity(){ // If this doesn't work, just switch to distance
        double lowestAmbiguity = Double.MAX_VALUE;

        if(fiducials != null) {
            for (RawFiducial fiducial : fiducials) {
                if (fiducial.ambiguity < lowestAmbiguity) {
                    lowestAmbiguity = fiducial.ambiguity;
                }
            }
        }

        return lowestAmbiguity;
    }

    /**
     * Whether to trust the Limelight pose or not.
     * @return true if it can be trusted
     */
    public boolean trustLimelightData(){
        return tagInView() && getLimelightAmbiguity() <= VisionConstants.MAX_APRILTAG_AMBIGUITY;
    }
}