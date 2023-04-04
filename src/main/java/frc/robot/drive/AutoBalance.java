// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.cosmetics.PwmLEDs;

public class AutoBalance extends CommandBase {
    private final DriveSubsystem drive;

    private PIDController pid = new PIDController(0.008, 0, 0.001);
    // p = 0.008, d = 0.001, increase d first, then decrease p after
    private PwmLEDs lights;
    private Integer count = 0;

    // private final PwmLEDs lights;

    /** Creates a new AutoBalance. */
    public AutoBalance(
            final DriveSubsystem drive, PwmLEDs lights) {
        this.drive = drive;
        this.lights = lights;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println(drive.getRawRobotPitch());
        var angleMultX = Math.abs(drive.getRobotYaw().getDegrees()) > 90 ? 1 : -1;
        var angleMultY = drive.getRobotYaw().getDegrees() > 0 ? -1 : 1;

        // Drives back and forth with pid until balanced on the platform
        if (count < 10) {
            count++;
        } else {
            count = 0;
            if (Math.abs(drive.getRobotRoll().getDegrees() * angleMultX) >= AutoConstants.BALANCE_ACCURACY_DEG) {
                drive.set(new ChassisSpeeds(pid.calculate(drive.getRobotRoll().getDegrees() * angleMultX, 0), 0, 0));
                lights.setDefault();
            } else if (Math.abs(drive.getRobotYaw().getDegrees() * angleMultY) <= AutoConstants.BALANCE_ACCURACY_DEG) {
                drive.set(new ChassisSpeeds(pid.calculate(drive.getRobotYaw().getDegrees() * angleMultY, 0), 0, 0));
                lights.setDefault();
            } else {
                // drive.set(new ChassisSpeeds(0, 0, 0.001));
                lights.setSolid(Color.kForestGreen);
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        lights.setDefault();
        // drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
