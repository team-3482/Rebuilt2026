// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.template.ExampleSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

/** An example command that does nothing. */
public class CalibrateQuestCommand extends Command {
    public CalibrateQuestCommand() {
        setName("CalibrateQuestCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(VisionSubsystem.getInstance());
    }

    double turnDegreesPerSecondRate = 37.5;

    ArrayList<Pose2d> collectedPoses = new ArrayList<>();

    double rotationTime = 562.5 / turnDegreesPerSecondRate;
    double samplePeriod = rotationTime / 15.0; // 20 samples around circle

    private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds();

    Timer timer = new Timer();
    Timer timer2 = new Timer();

    @Override
    public void initialize() {
        timer.restart();
        timer2.restart();
    }

    @Override
    public void execute() {
        SwerveSubsystem.getInstance().setControl(
            drive.withSpeeds(new ChassisSpeeds(0, 0, Units.degreesToRadians(turnDegreesPerSecondRate))
            )
        );
        if (timer2.advanceIfElapsed(samplePeriod)) {
            // quest offset must be zero!
            collectedPoses.add(VisionSubsystem.getInstance().getPose2d());
        }
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setControl(
            drive.withSpeeds(new ChassisSpeeds())
        );

        String outputStr = convertCordsToCSVText(collectedPoses);

        System.out.println(outputStr);
        Logger.recordOutput("QuestNav/CalibrationData", outputStr);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(rotationTime);
    }

    public String convertCordsToCSVText(List<Pose2d> poses) {
        String csvText = "X, Y\n";
        for (Pose2d pose : poses) {
            csvText += pose.getX() + ", " + pose.getY() + "\n";
        }

        return csvText;
    }
}