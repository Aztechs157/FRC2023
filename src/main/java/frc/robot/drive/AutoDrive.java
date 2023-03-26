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

    // return if the magnitude of the current has exceeded the magnitude of the
    // target
    private boolean checkVal(double currVal, double targetVal) {
        return targetVal > 0 ? currVal > targetVal : currVal < targetVal;
    }

    // if min is empty, start, otherwise check value to recheck min
    private boolean checkStarting(Optional<Double> min, double current) {
        return min.isEmpty() || checkVal(current, min.get());
    }

    // caps the magnitude of val to cap
    public double capVal(double val, double cap, double min) {
        double retval = val;
        double tempCap = Math.abs(cap);
        retval = retval > tempCap ? tempCap : retval;
        retval = retval < -tempCap ? -tempCap : retval;
        retval = Math.abs(retval) < min ? 0 : retval;
        return retval;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // wait to start if wanted
        if (params.startTime.isPresent() && Timer.getFPGATimestamp() - startTime < params.startTime.get()) {
            return;
        }
        Pose2d pose = drive.getOdometryPose();
        Pose2d setPose = params.targetPose;
        Rotation2d rot = drive.getRobotPitch();

        System.out.println(
                "xDist: " + pose.getX() + "\nyDist: " + pose.getY() + "\nrot: " + pose.getRotation().getDegrees());

        double xVal = 0;
        double yVal = 0;
        double rotVal = 0;

        // checks to see if we are driving each axis independently
        boolean useX = checkStarting(params.startXatY, pose.getY())
                && checkStarting(params.startXatRot, rot.getDegrees());
        boolean useY = checkStarting(params.startYatX, pose.getX())
                && checkStarting(params.startYatRot, rot.getDegrees());
        boolean useRot = params.useRot && checkStarting(params.startRotAtY, pose.getY())
                && checkStarting(params.startRotAtX, pose.getX());

        // use the x axis if wanted, otherwise hold x if desired, otherwise do nothing
        if (useX) {
            xVal = params.usePidX ? pidx.calculate(pose.getX(), setPose.getX()) : params.maxXSpeed;
        } else if (params.holdXTillXStarts) {
            xVal = pidx.calculate(pose.getX(), startX);
        }
        // use if slew is wanted on X
        if (params.useSlewX) {
            xVal = slewx.calculate(xVal);
        }
        // use the y axis if wanted, otherwise hold y if desired, otherwise do nothing
        if (useY) {
            yVal = params.usePidY ? pidy.calculate(pose.getY(), setPose.getY()) : params.maxYSpeed;
        } else if (params.holdYTillYStarts) {
            yVal = pidy.calculate(pose.getY(), startY);
        }
        // use if slew is wanted on Y
        if (params.useSlewY) {
            yVal = slewy.calculate(yVal);
        }
        // rotate around the z axis if wanted, otherwise hold rotation if desired,
        // otherwise do nothing
        if (useRot) {
            rotVal = pidr.calculate(rot.getDegrees(), setPose.getRotation().getDegrees());
        } else if (params.holdRotTillRotStarts) {
            rotVal = pidr.calculate(rot.getDegrees(), startRot);
        }
        // use if slew is wanted on rotation
        if (params.useSlewRot) {
            rotVal = slewr.calculate(rotVal);
        }

        // caps values to max speed
        xVal = capVal(xVal, params.maxXSpeed, params.minX);
        yVal = capVal(yVal, params.maxYSpeed, params.minY);
        rotVal = capVal(rotVal, params.maxRotSpeed, params.minRot);

        System.out.println("xvcal: " + xVal + "\nyVal: " + yVal + "\nrotVal: " + rotVal);

        drive.set(new ChassisSpeeds(xVal, yVal, -rotVal));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // optional, if no tolerance we return true, otherwise check if the distance
    // from target is in tolerance
    public boolean checkTolerance(double target, Optional<Double> tolerance, double current) {
        return tolerance.isEmpty() || Math.abs(target - current) < tolerance.get();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        Pose2d pose = drive.getOdometryPose();
        // if no tolerances return false, otherwise check tolerance on X and Y, and
        // tolerance on Rotation if applicable
        return !(params.rotTolerance.isEmpty() && params.xTolerance.isEmpty() && params.yTolerance.isEmpty()) &&
                (checkTolerance(params.targetPose.getX(), params.xTolerance, pose.getX()) &&
                        checkTolerance(params.targetPose.getY(), params.yTolerance, pose.getY()) &&
                        (!params.useRot || checkTolerance(params.targetPose.getRotation().getDegrees(),
                                params.rotTolerance, pose.getRotation().getDegrees())));
    }
}
