// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;

/** An example command that does nothing. */
public class MovePivotCommand extends Command {
    private Angle position;

    public MovePivotCommand(Angle position) {
        setName("MovePivotCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem.getInstance());

        this.position = position;
    }

    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().motionMagicPosition(this.position);
    }

    @Override
    public void execute() {
        double angle = IntakeSubsystem.getInstance().getPosition().in(Degrees);
        Logger.recordOutput("Intake/PivotAngle", angle);
        SmartDashboard.putNumber("Intake/PivotAngle", angle);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return IntakeSubsystem.getInstance().withinTolerance(this.position);
    }
}