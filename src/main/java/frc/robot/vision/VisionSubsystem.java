// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.VirtualConstants;
import frc.robot.swerve.SwerveSubsystem;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

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

    private VisionSubsystem() {
        super("VisionSubsystem");
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();

        boolean tracking = questNav.isTracking();
        Logger.recordOutput("QuestNav/Tracking", tracking);

        if (tracking) {
            if (questNav.isTracking()) {
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
            return questPose.transformBy(PhysicalConstants.Vision.ROBOT_TO_QUEST.inverse());
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

    /**
     * Tells QuestNav an accurate pose to start tracking from.
     * Will run at beginning of match with AprilTag pose data
     * or later if necessary to recalibrate the Quest.
     * @param actualPose The trusted and accurate pose.
     */
    public void resetPose(Pose3d actualPose){
        questNav.setPose(actualPose.transformBy(PhysicalConstants.Vision.ROBOT_TO_QUEST));
    }

    private void updateSwervePoseEstimation() {
        for (PoseFrame poseFrame : poseFrames) {
            double timestamp = poseFrame.dataTimestamp();

            // Transform by the mount pose to get the robot pose
            Pose3d robotPose = poseFrame.questPose3d().transformBy(PhysicalConstants.Vision.ROBOT_TO_QUEST.inverse());

            // Add the measurement
            SwerveSubsystem.getInstance().addVisionMeasurement(robotPose.toPose2d(), timestamp, VirtualConstants.Vision.QUESTNAV_TRUST_STD_DEVS);
        }
    }
}