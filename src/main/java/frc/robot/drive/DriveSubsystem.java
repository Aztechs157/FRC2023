// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.ArrayList;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
    // tune these values for auto (mostly p, maybe some d)
    public PIDController pidx = new PIDController(0.6, 0, 0);
    public PIDController pidy = new PIDController(0.6, 0, 0);
    public PIDController pidr = new PIDController(0.5, 0, 0);

    public SlewRateLimiter slewx = new SlewRateLimiter(DriveConstants.AUTO_SLEW_RATE);
    public SlewRateLimiter slewy = new SlewRateLimiter(DriveConstants.AUTO_SLEW_RATE);
    public SlewRateLimiter slewr = new SlewRateLimiter(DriveConstants.AUTO_SLEW_ROTATE_VAL);

    public SwervePod[] swervePods = new SwervePod[] {
            new SwervePod(DriveConstants.POD_CONFIGS[0], table.getSubTable("Pod 1")),
            new SwervePod(DriveConstants.POD_CONFIGS[1], table.getSubTable("Pod 2")),
            new SwervePod(DriveConstants.POD_CONFIGS[2], table.getSubTable("Pod 3")),
            new SwervePod(DriveConstants.POD_CONFIGS[3], table.getSubTable("Pod 4"))
    };

    public DriveSubsystem() {
        pidr.enableContinuousInput(-180, 180);
        final var resetCommand = runOnce(this::resetGyro).ignoringDisable(true);
        SmartDashboard.putData("Reset Yaw", resetCommand);
        setAllEncoderMultipliers(0.03921201641325);
    }

    public void setAllEncoderMultipliers(double distancePerRotation) {
        for (SwervePod swervePod : swervePods) {
            swervePod.setDistancePerRotation(distancePerRotation);
        }
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
    public void driveDistanceOdometer(double xPos, double yPos, double angle) {
        var pose = odometer.getPoseMeters();
        set(new ChassisSpeeds(pidx.calculate(pose.getX(), xPos), pidy.calculate(pose.getY(), yPos),
                pidr.calculate(getRobotPitch().getDegrees(), angle)));
    }

    public Command driveToPosWithAngleOdometry(double xPos, double yPos, double angle, double tolerance) {
        return runEnd(() -> driveDistanceOdometer(xPos, yPos, angle), () -> stop()).until(() -> {
            var pose = odometer.getPoseMeters();
            return pose.getX() < tolerance && pose.getY() < tolerance;
        });
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

    public Pose2d getOdometryPose() {
        return odometer.getPoseMeters();
    }

    @Override
    public void periodic() {
        gyroTable.getEntry("Yaw").setDouble(gyro.getYaw());
        gyroTable.getEntry("Pitch").setDouble(gyro.getPitch());
        gyroTable.getEntry("Roll").setDouble(gyro.getRoll());
        table.getEntry("Raw Drive Position").setDouble(getRawDrivePosition());
        odometer.update(getRobotPitch(), getModulePositions());
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

    public SwerveModulePosition[] getModulePositions() {
        ArrayList<SwerveModulePosition> retval = new ArrayList<SwerveModulePosition>(swervePods.length);
        for (SwervePod swervePod : swervePods) {
            retval.add(swervePod.getPosition());

        }
        return retval.toArray(new SwerveModulePosition[] {});
    }

    public Command resetPositionsCommand() {
        return runEnd(this::resetPostitions, this::stop).until(this::checkAllPositionsCloseToZero);
    }

    public static class AutoDriveLineBuilder {
        protected Pose2d targetPose;

        protected boolean useRot = true;

        protected Optional<Double> startXatY = Optional.empty();
        protected Optional<Double> startXatRot = Optional.empty();

        protected Optional<Double> startYatX = Optional.empty();
        protected Optional<Double> startYatRot = Optional.empty();

        protected Optional<Double> startRotAtX = Optional.empty();
        protected Optional<Double> startRotAtY = Optional.empty();

        protected Optional<Double> startTime = Optional.empty();

        protected Optional<Double> xTolerance = Optional.empty();
        protected Optional<Double> yTolerance = Optional.empty();
        protected Optional<Double> rotTolerance = Optional.empty();

        protected Double maxXSpeed = 1.0;
        protected Double maxYSpeed = 1.0;
        protected Double maxRotSpeed = 65.0;

        protected double minX = 0.08;
        protected double minY = 0.08;
        protected double minRot = 0.1;

        protected boolean useSlewX = false;
        protected boolean useSlewY = false;
        protected boolean useSlewRot = false;

        protected boolean usePidX = true;
        protected boolean usePidY = true;

        protected boolean holdRotTillRotStarts = false;
        protected boolean holdXTillXStarts = false;
        protected boolean holdYTillYStarts = false;

        public AutoDriveLineBuilder(double xDist, double yDist) {
            this(xDist, yDist, 0.0);
            useRot = false;
        }

        public AutoDriveLineBuilder(double xDist, double yDist, double targetAngle) {
            targetPose = new Pose2d(xDist, yDist, Rotation2d.fromDegrees(0.0));
        }

        public AutoDriveLineBuilder(Pose2d pose) {
            targetPose = pose;
        }

        public AutoDriveLineBuilder startXAtY(Optional<Double> val) {
            startXatY = val;
            return this;
        }

        public AutoDriveLineBuilder startXAtY(double val) {
            startXatY = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder startXatRot(Optional<Double> val) {
            startXatRot = val;
            return this;
        }

        public AutoDriveLineBuilder startXatRot(double val) {
            startXatRot = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder startYatX(Optional<Double> val) {
            startYatX = val;
            return this;
        }

        public AutoDriveLineBuilder startYatX(double val) {
            startYatX = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder startYatRot(Optional<Double> val) {
            startYatRot = val;
            return this;
        }

        public AutoDriveLineBuilder startYatRot(double val) {
            startYatRot = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder startRotAtX(Optional<Double> val) {
            startRotAtX = val;
            return this;
        }

        public AutoDriveLineBuilder startRotAtX(double val) {
            startRotAtX = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder startRotAtY(Optional<Double> val) {
            startRotAtY = val;
            return this;
        }

        public AutoDriveLineBuilder startRotAtY(double val) {
            startRotAtY = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder startTime(Optional<Double> val) {
            startTime = val;
            return this;
        }

        public AutoDriveLineBuilder startTime(double val) {
            startTime = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder xTolerance(Optional<Double> val) {
            xTolerance = val;
            return this;
        }

        public AutoDriveLineBuilder xTolerance(double val) {
            xTolerance = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder yTolerance(Optional<Double> val) {
            yTolerance = val;
            return this;
        }

        public AutoDriveLineBuilder yTolerance(double val) {
            yTolerance = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder rotTolerance(Optional<Double> val) {
            rotTolerance = val;
            return this;
        }

        public AutoDriveLineBuilder rotTolerance(double val) {
            rotTolerance = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder xyTolerance(Optional<Double> val) {
            xTolerance = val;
            yTolerance = val;
            return this;
        }

        public AutoDriveLineBuilder xyTolerance(double val) {
            xTolerance = Optional.of(val);
            yTolerance = Optional.of(val);
            return this;
        }

        public AutoDriveLineBuilder useSlewXY(boolean val) {
            useSlewX = val;
            useSlewY = val;
            return this;
        }

        public AutoDriveLineBuilder useSlewAll(boolean val) {
            useSlewX = val;
            useSlewY = val;
            useSlewRot = val;
            return this;
        }

        public AutoDriveLineBuilder useSlewX(boolean val) {
            useSlewX = val;
            return this;
        }

        public AutoDriveLineBuilder useSlewY(boolean val) {
            useSlewY = val;
            return this;
        }

        public AutoDriveLineBuilder useSlewRot(boolean val) {
            useSlewRot = val;
            return this;
        }

        public AutoDriveLineBuilder holdXTillXStarts(boolean val) {
            holdXTillXStarts = val;
            return this;
        }

        public AutoDriveLineBuilder holdYTillYStarts(boolean val) {
            holdYTillYStarts = val;
            return this;
        }

        public AutoDriveLineBuilder holdRotTillRotStarts(boolean val) {
            holdRotTillRotStarts = val;
            return this;
        }

        public AutoDriveLineBuilder holdXYTillXYStarts(boolean val) {
            holdXTillXStarts = val;
            holdYTillYStarts = val;
            return this;
        }

        public AutoDriveLineBuilder HoldAllTillStart(boolean val) {
            holdXTillXStarts = val;
            holdYTillYStarts = val;
            holdRotTillRotStarts = val;
            return this;
        }

        public AutoDriveLineBuilder maxXSpeed(double val) {
            maxXSpeed = val;
            return this;
        }

        public AutoDriveLineBuilder maxYSpeed(double val) {
            maxYSpeed = val;
            return this;
        }

        public AutoDriveLineBuilder maxRotSpeed(double val) {
            maxRotSpeed = val;
            return this;
        }

        public AutoDriveLineBuilder maxXYSpeed(double val) {
            maxXSpeed = val;
            maxYSpeed = val;
            return this;
        }

        public AutoDriveLineBuilder usePidY(boolean val) {
            usePidY = val;
            return this;
        }

        public AutoDriveLineBuilder usePidX(boolean val) {
            usePidX = val;
            return this;
        }
    }

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(0),
            getModulePositions());

}
