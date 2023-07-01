package frc.robot.drive;

import org.assabet.aztechs157.Expect;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants.DriveConstants;

public class SwervePod {

    public record Config(int driveMotorId, int angleMotorId, int angleEncoderId, boolean driveMotorInverted) {
    }

    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;
    private final CANCoder angleEncoder;
    private final NetworkTable table;

    public SwervePod(final Config config, final NetworkTable table) {
        this.table = table;
        driveMotor = new CANSparkMax(config.driveMotorId, MotorType.kBrushless);
        angleMotor = new CANSparkMax(config.angleMotorId, MotorType.kBrushless);
        angleEncoder = new CANCoder(config.angleEncoderId);

        table.getEntry("Inverted").setBoolean(config.driveMotorInverted);

        driveMotor.setIdleMode(DriveConstants.DRIVE_IDLE_MODE);
        driveMotor.setInverted(false);
        angleMotor.setIdleMode(DriveConstants.ANGLE_IDLE_MODE);
        angleMotor.setInverted(true);
        angleEncoder.configSensorDirection(false);
    }

    public void set(final SwerveModuleState state) {
        goToAngle(wrapDegrees(state.angle.getDegrees()));
        drive(state.speedMetersPerSecond);
    }

    public void directSet(final double driveSpeed, final double angleSpeed) {
        getCurrentAngle();

        setAngleSpeed(angleSpeed);
        drive(driveSpeed);
    }

    public void stop() {
        setAngleSpeed(0);
        drive(0);
    }

    private final SlewRateLimiter driveSlewrate = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
    private boolean reversed = false;

    private void drive(double speed) {
        table.getEntry("Input Speed").setDouble(speed);
        table.getEntry("Roll Reversed?").setBoolean(reversed);

        if (reversed) {
            speed = -speed;
        }

        final var inverted = table.getEntry("Inverted");
        if (inverted.getBoolean(false)) {
            speed = -speed;
        }

        driveMotor.set(driveSlewrate.calculate(speed));
    }

    public void resetDrivePosition() {
        driveMotor.getEncoder().setPosition(0);
    }

    public double getRawDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    private double wrapDegrees(final double degrees) {
        var wrapped = degrees % 360;

        if (wrapped < 0) {
            wrapped += 360;
        }

        Expect.number(wrapped).greaterOrEqual(0).lessOrEqual(360);
        return wrapped;
    }

    public double getCurrentAngle() {
        return angleEncoder.getAbsolutePosition();
    }

    private void setAngleSpeed(final double speed) {
        angleMotor.set(speed);
    }

    // Next 3 methods are used to find the shortest distance to get to a given angle
    private void goToAngle(final double target) {
        table.getEntry("Input Angle").setDouble(target);
        table.getEntry("Current Angle").setDouble(getCurrentAngle());

        final var initialDelta = computeInitialDelta(target);
        table.getEntry("Initial Delta").setDouble(initialDelta);

        final var shortestDelta = computeShortestDelta(initialDelta);
        table.getEntry("Shorted Delta").setDouble(shortestDelta);

        final var pidOutput = computeAnglePidOutput(shortestDelta);
        table.getEntry("PID Output").setDouble(pidOutput);
        setAngleSpeed(pidOutput);
    }

    private double computeInitialDelta(final double target) {
        final double initial = getCurrentAngle();

        Expect.number(target).greaterOrEqual(0).lessOrEqual(360);
        Expect.number(initial).greaterOrEqual(0).lessOrEqual(360);

        var initialDelta = target - initial;
        Expect.number(initialDelta).greaterOrEqual(-360).lessOrEqual(360);

        if (initialDelta < 0) {
            initialDelta += 360;
        }
        Expect.number(initialDelta).greaterOrEqual(0).lessOrEqual(360);

        return initialDelta;
    }

    private double computeShortestDelta(final double initialDelta) {
        Expect.number(initialDelta).greaterOrEqual(0).lessOrEqual(360);

        if (initialDelta < 180) {
            return initialDelta;
        } else {
            return initialDelta - 360;
        }
    }

    private final PIDController anglePid = new PIDController(DriveConstants.ANGLE_KP, 0, DriveConstants.ANGLE_KD);

    private double computeAnglePidOutput(final double shortestDelta) {
        final var pidOutput = anglePid.calculate(getCurrentAngle() + shortestDelta, getCurrentAngle());
        return pidOutput;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getRawDrivePosition(), Rotation2d.fromDegrees(getCurrentAngle()));
    }

    public void setDistancePerRotation(double distancePerRotation) {
        driveMotor.getEncoder().setPositionConversionFactor(distancePerRotation); // this is in meters
    }
}
