// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.DriveSubsystem.AutoDriveLineBuilder;

public class AutoDrive extends CommandBase {

    private AutoDriveLineBuilder params;
    private DriveSubsystem drive;

    private double startX;
    private double startY;
    private double startRot;
    private final PIDController pidx;
    private final PIDController pidy;
    private final PIDController pidr;

    private final SlewRateLimiter slewx;
    private final SlewRateLimiter slewy;
    private final SlewRateLimiter slewr;
    public double startTime = 0;

    /** Creates a new AutoDrive. */
    public AutoDrive(DriveSubsystem drive, AutoDriveLineBuilder args) {
        params = args;
        this.drive = drive;
        addRequirements(drive);
        pidx = drive.pidx;
        pidy = drive.pidy;
        pidr = drive.pidr;

        slewx = drive.slewx;
        slewy = drive.slewy;
        slewr = drive.slewr;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d pose = drive.getOdometryPose();
        startX = pose.getX();
        startY = pose.getY();
        startRot = pose.getRotation().getDegrees();

        pidx.reset();
        pidy.reset();
        pidr.reset();

        slewx.reset(0);
        slewy.reset(0);
        slewr.reset(0);

        startTime = Timer.getFPGATimestamp();

    }

    private boolean checkVal(double currVal, double targetVal) {
        return targetVal > 0 ? currVal > targetVal : currVal < targetVal;
    }

    private boolean checkStarting(Optional<Double> min, double current) {
        return min.isEmpty() || checkVal(current, min.get());
    }

    public double capVal(double val, double cap) {
        double retval = val;
        double tempCap = Math.abs(cap);
        retval = retval > tempCap ? tempCap : retval;
        retval = retval < -tempCap ? -tempCap : retval;
        return retval;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (params.startTime.isPresent() && Timer.getFPGATimestamp() - startTime < params.startTime.get()) {
            return;
        }
        Pose2d pose = drive.getOdometryPose();
        Pose2d setPose = params.targetPose;
        Rotation2d rot = drive.getRobotYaw();

        double xVal = 0;
        double yVal = 0;
        double rotVal = 0;

        boolean useX = checkStarting(params.startXatY, pose.getY())
                && checkStarting(params.startXatRot, rot.getDegrees());
        boolean useY = checkStarting(params.startYatX, pose.getX())
                && checkStarting(params.startYatRot, rot.getDegrees());
        boolean useRot = params.useRot && checkStarting(params.startRotAtY, pose.getY())
                && checkStarting(params.startRotAtX, pose.getX());

        if (useX) {
            xVal = params.usePidX ? pidx.calculate(pose.getX(), setPose.getX()) : params.maxXSpeed;
        } else if (params.holdXTillXStarts) {
            xVal = pidx.calculate(pose.getX(), startX);
        }
        if (params.useSlewX) {
            xVal = slewx.calculate(xVal);
        }
        if (useY) {
            yVal = params.usePidY ? pidy.calculate(pose.getY(), setPose.getY()) : params.maxYSpeed;
            if (params.useSlewY) {
                yVal = slewy.calculate(yVal);
            }
        } else if (params.holdYTillYStarts) {
            yVal = pidy.calculate(pose.getY(), startY);
        }
        if (params.useSlewY) {
            yVal = slewy.calculate(yVal);
        }
        if (useRot) {
            rotVal = pidr.calculate(rot.getDegrees(), setPose.getRotation().getDegrees());
            if (params.useSlewRot) {
                rotVal = slewr.calculate(rotVal);
            }
        } else if (params.holdRotTillRotStarts) {
            rotVal = pidr.calculate(rot.getDegrees(), startRot);
        }
        if (params.useSlewRot) {
            rotVal = slewr.calculate(rotVal);
        }

        xVal = capVal(xVal, params.maxXSpeed);
        yVal = capVal(yVal, params.maxYSpeed);
        rotVal = capVal(rotVal, params.maxRotSpeed);

        drive.set(new ChassisSpeeds(xVal, yVal, rotVal));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    public boolean checkTolerance(double target, Optional<Double> tolerance, double current) {
        return tolerance.isEmpty() || Math.abs(target - current) < tolerance.get();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        Pose2d pose = drive.getOdometryPose();
        return !(params.rotTolerance.isEmpty() && params.xTolerance.isEmpty() && params.yTolerance.isEmpty()) &&
                (checkTolerance(params.targetPose.getX(), params.xTolerance, pose.getX()) &&
                        checkTolerance(params.targetPose.getY(), params.yTolerance, pose.getY()) &&
                        (!params.useRot || checkTolerance(params.targetPose.getRotation().getDegrees(),
                                params.rotTolerance, pose.getRotation().getDegrees())));
    }
}
