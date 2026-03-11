// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.Degrees;

/** Move shooter hood to position */
public class AngleHoodCommand extends Command {
    private double position;

    public AngleHoodCommand(double position) {
        setName("AngleHoodCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());

        this.position = position;
    }

    public AngleHoodCommand(Angle angle) {
        setName("AngleHoodCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem.getInstance());

        this.position = (angle.in(Degrees) - ShooterConstants.HOOD_ANGLE_MIN.in(Degrees))
            / (ShooterConstants.HOOD_ANGLE_MAX.in(Degrees) - ShooterConstants.HOOD_ANGLE_MIN.in(Degrees));
    }

    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setHoodAngle(position);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}