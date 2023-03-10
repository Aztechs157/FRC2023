// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            Constants.DriveConstants.WHEEL_LOCATIONS);

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("157/Swerve");
    private float gyroOffset = 0.0f;
    public PIDController pidx = new PIDController(0.01, 0, 0);
    public PIDController pidy = new PIDController(0.01, 0, 0);
    public PIDController pidr = new PIDController(1, 0, 0);

    public SwervePod[] swervePods = new SwervePod[] {
            new SwervePod(DriveConstants.POD_CONFIGS[0], table.getSubTable("Pod 1")),
            new SwervePod(DriveConstants.POD_CONFIGS[1], table.getSubTable("Pod 2")),
            new SwervePod(DriveConstants.POD_CONFIGS[2], table.getSubTable("Pod 3")),
            new SwervePod(DriveConstants.POD_CONFIGS[3], table.getSubTable("Pod 4"))
    };

    public DriveSubsystem() {
        final var resetCommand = runOnce(this::resetGyro).ignoringDisable(true);
        SmartDashboard.putData("Reset Yaw", resetCommand);
    }

    public void resetGyro() {
        gyro.zeroYaw();
        addGyroOffset(0);
    }

    public void resetDisplacement() {
        gyro.resetDisplacement();
    }

    public double getXDisplacement() {
        return gyro.getDisplacementX();
    }

    public double getYDisplacement() {
        return gyro.getDisplacementY();
    }

    public Command driveWithRotation(double desiredAngle, double xSpeed, double ySpeed) {
        return run(() -> driveDistanceWithRotation(desiredAngle, xSpeed, ySpeed));
    }

    public Command driveWithRotationWithStop(double desiredAngle, double xSpeed, double ySpeed) {
        return runEnd(() -> driveDistanceWithRotation(desiredAngle, xSpeed, ySpeed), () -> stop());
    }

    public void driveDistanceWithRotation(double desiredAngle, double xSpeed, double ySpeed) {
        set(new ChassisSpeeds(xSpeed, ySpeed,
                pidr.calculate(getRobotPitch().getDegrees(), desiredAngle)));
    }

    // TODO test this please, it might just work or just need a few negatives. it
    // uses Accellerometer data to attempt to drive for a distance.
    public void driveDistanceAccellerometer(double xPos, double yPos, double angle) {
        set(new ChassisSpeeds(pidx.calculate(getXDisplacement(), xPos), pidy.calculate(getYDisplacement(), yPos),
                pidr.calculate(getRobotPitch().getDegrees(), angle)));
    }

    public void set(final ChassisSpeeds inputSpeeds) {
        final var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(inputSpeeds, getRobotYaw());
        final var states = kinematics.toSwerveModuleStates(speeds);

        for (var i = 0; i < states.length; i++) {
            swervePods[i].set(states[i]);
        }
    }

    public void stop() {
        for (final var swervePod : swervePods) {
            swervePod.stop();
        }
    }

    public void setSingle(final SwerveModuleState state) {
        swervePods[0].set(state);
    }

    public void directSetSingle(final double rollSpeed, final double spinSpeed) {
        swervePods[0].directSet(rollSpeed, spinSpeed);
    }

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final NetworkTable gyroTable = NetworkTableInstance.getDefault().getTable("157/Gyro");

    public Rotation2d getRobotYaw() {
        return Rotation2d.fromDegrees(((-gyro.getYaw() + 180 + gyroOffset) % 360) - 180);
    }

    public Command addGyroOffset(float degrees) {
        return runOnce(() -> gyroOffset = degrees);
    }

    public double getRawRobotPitch() {
        return gyro.getRoll();
    }

    public Rotation2d getRobotRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public Rotation2d getRobotPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    @Override
    public void periodic() {
        gyroTable.getEntry("Yaw").setDouble(gyro.getYaw());
        gyroTable.getEntry("Pitch").setDouble(gyro.getPitch());
        gyroTable.getEntry("Roll").setDouble(gyro.getRoll());
        table.getEntry("Raw Drive Position").setDouble(getRawDrivePosition());
    }

    public void resetDrivePosition() {
        for (final var swervePod : swervePods) {
            swervePod.resetDrivePosition();
        }
    }

    public double getRawDrivePosition() {
        double result = 0;
        for (final var swervePod : swervePods) {
            result += swervePod.getRawDrivePosition();
        }
        return result / swervePods.length;
    }

    public Command resetDrivePositionCommand() {
        return runOnce(() -> resetDrivePosition());
    }

    // auto command that will have the robot drive relative to field
    public Command driveRawDistanceCommand(final ChassisSpeeds inputSpeeds, final double rawDistance) {
        return run(() -> set(inputSpeeds))
                .until(() -> Math.abs(getRawDrivePosition() - rawDistance) <= AutoConstants.DRIVE_ACCURACY)
                .finallyDo((a) -> stop());
    }

    public Command turnToAngleCommand(final Rotation2d desiredAngle, float turnSpeed) {
        return run(() -> set(new ChassisSpeeds(0, 0, turnSpeed)))
                .until(() -> Math
                        .abs(getRobotYaw().getDegrees() - desiredAngle.getDegrees()) <= AutoConstants.TURN_ACCURACY_DEG)
                .finallyDo((a) -> stop());
    }

    // These 3 methods are used for orienting wheels the same direction, currently
    // not in use, however look into further if distance based auto is wanted TODO:
    // Test
    public void resetPostitions() {
        for (SwervePod swervePod : swervePods) {
            swervePod.directSet(0, (180 - swervePod.getCurrentAngle()) / 90);
        }
    }

    public boolean checkAllPositionsCloseToZero() {
        for (SwervePod swervePod : swervePods) {
            if (!(swervePod.getCurrentAngle() < 20 || swervePod.getCurrentAngle() > 340)) {
                return false;
            }
        }
        return true;
    }

    public Command resetPositionsCommand() {
        return runEnd(this::resetPostitions, this::stop).until(this::checkAllPositionsCloseToZero);
    }
}
