// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.IntakeConstants;

/** An example command that does nothing. */
public class MovePivotCommand extends Command {
    private double position;
    
    public MovePivotCommand(double position) {
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
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return IntakeSubsystem.getInstance().withinTolerance(this.position);
    }
}