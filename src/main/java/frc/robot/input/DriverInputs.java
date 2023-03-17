package frc.robot.input;

import org.assabet.aztechs157.input.layouts.Layout;
import org.assabet.aztechs157.input.layouts.MapLayout;
import org.assabet.aztechs157.input.layouts.DynamicLayout;
import org.assabet.aztechs157.input.models.LogitechExtreme3D;
import org.assabet.aztechs157.input.models.LogitechGamepadF310;
import org.assabet.aztechs157.input.values.Axis;
import org.assabet.aztechs157.input.values.Button;
import org.assabet.aztechs157.numbers.Deadzone;
import org.assabet.aztechs157.numbers.Range;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriverInputs extends DynamicLayout {
    public static final Axis.Key intakeSpeed = new Axis.Key("Intake Speed");
    public static final Axis.Key runIntakeMotorIn = new Axis.Key("Run Intake Motor In");
    public static final Axis.Key runIntakeMotorOut = new Axis.Key("Run Intake Motor Out");
    public static final Axis.Key runIntakeMotorInDriver = new Axis.Key("Run Intake Motor In via Driver");
    public static final Axis.Key runIntakeMotorOutDriver = new Axis.Key("Run Intake Motor Out via Driver");
    public static final Button.Key setIntakeSolenoidForward = new Button.Key("Set Intake Solenoid Forward");
    public static final Button.Key setIntakeSolenoidBackward = new Button.Key("Set Intake Solenoid Backward");
    public static final Button.Key lowPosition = new Button.Key("Set to low position");
    public static final Button.Key midPosition = new Button.Key("Set to mid position");
    public static final Button.Key highPosition = new Button.Key("Set to high position");
    public static final Button.Key loadingPosition = new Button.Key("Set to loading position");
    public static final Button.Key startPosition = new Button.Key("Set to start position");
    public static final Button.Key autoBalance = new Button.Key("Run the balance on platform command");

    public static final Axis.Key driveSpeedX = new Axis.Key("Drive Speed X");
    public static final Axis.Key driveSpeedY = new Axis.Key("Drive Speed Y");
    public static final Axis.Key driveRotation = new Axis.Key("Drive Rotation");
    public static final Axis.Key rotateWrist = new Axis.Key("Rotate Wrist");
    public static final Axis.Key rotateElbow = new Axis.Key("Rotate Elbow");
    public static final Axis.Key elevator = new Axis.Key("Lift Elevator");
    public static final Axis.Key carriage = new Axis.Key("Lift Carriage");

    private static final NetworkTableEntry entry = NetworkTableInstance.getDefault().getEntry("157/Drive/StickEnabled");

    public DriverInputs() {
        super(() -> true ? flightStickLayout() : dualLogitechLayout());

        entry.setDefaultBoolean(false);
        entry.setPersistent();
    }

    private static final Deadzone deadzone = Deadzone.forAxis(new Range(-0.2, 0.2));
    private static final Rotation2d maxRotationPerSecond = Rotation2d.fromDegrees(65);

    private static Layout dualLogitechLayout() {
        final var layout = new MapLayout("Default Eric/Connor Layout");
        final var driver = new LogitechGamepadF310(0);
        final var operator = new LogitechGamepadF310(1);

        final var speedModifier = 0.20;

        layout.assign(driveSpeedX, driver.leftStickX.map(deadzone::apply).scaledBy(speedModifier));
        layout.assign(driveSpeedY, driver.leftStickY.map(deadzone::apply).scaledBy(speedModifier));
        layout.assign(driveRotation, driver.rightStickX.map(deadzone::apply).scaledBy(speedModifier)
                .scaledBy(maxRotationPerSecond.getDegrees()));
        layout.assign(autoBalance, driver.a);
        layout.assign(runIntakeMotorInDriver, driver.rightTriggerHeld);
        layout.assign(runIntakeMotorOutDriver, driver.leftTriggerHeld);

        layout.assign(intakeSpeed, new Axis("Operator and driver triggers", () -> {
            if (Math.abs(operator.combinedTriggersHeld.get()) > Math.abs(driver.combinedTriggersHeld.get())) {
                return operator.combinedTriggersHeld.get();
            }
            return driver.combinedTriggersHeld.get();
        }));

        layout.assign(runIntakeMotorIn, operator.rightTriggerHeld);
        layout.assign(runIntakeMotorOut, operator.leftTriggerHeld);
        layout.assign(setIntakeSolenoidForward, operator.rightBumper);
        layout.assign(setIntakeSolenoidBackward, operator.leftBumper);
        layout.assign(lowPosition, operator.x);
        layout.assign(midPosition, operator.a);
        layout.assign(highPosition, operator.y);
        layout.assign(loadingPosition, operator.b);
        layout.assign(startPosition, operator.start);

        layout.assign(rotateWrist, operator.rightStickY.map(deadzone::apply).scaledBy(-0.5));
        layout.assign(rotateElbow, operator.leftStickY.map(deadzone::apply).scaledBy(-0.5));

        layout.assign(elevator, operator.pov.y.scaledBy(-0.25));
        layout.assign(carriage, operator.pov.x.scaledBy(0.75));

        return layout;
    }

    private static Layout flightStickLayout() {
        final var layout = new MapLayout("driver flight, operator logitech");
        final var driver = new LogitechExtreme3D(0);
        final var operator = new LogitechGamepadF310(1);

        final var speedModifier = 0.20;

        layout.assign(driveSpeedX, driver.stickX.map(deadzone::apply).scaledBy(speedModifier));
        layout.assign(driveSpeedY, driver.stickY.map(deadzone::apply).scaledBy(speedModifier));
        layout.assign(driveRotation, driver.stickRotate.map(deadzone::apply).scaledBy(speedModifier)
                .scaledBy(maxRotationPerSecond.getDegrees()));
        layout.assign(autoBalance, driver.button4);
        layout.assign(intakeSpeed, new Axis("Operator and driver triggers", () -> {
            double driverSpeed = driver.thumb.get() ? 1 : (driver.trigger.get() ? -1 : 0);
            if (Math.abs(operator.combinedTriggersHeld.get()) > Math.abs(driverSpeed)) {
                return operator.combinedTriggersHeld.get();
            }
            return driverSpeed;
        }));

        layout.assign(runIntakeMotorIn, operator.rightTriggerHeld);
        layout.assign(runIntakeMotorOut, operator.leftTriggerHeld);
        layout.assign(setIntakeSolenoidForward, operator.rightBumper);
        layout.assign(setIntakeSolenoidBackward, operator.leftBumper);
        layout.assign(lowPosition, operator.x);
        layout.assign(midPosition, operator.a);
        layout.assign(highPosition, operator.y);
        layout.assign(loadingPosition, operator.b);
        layout.assign(startPosition, operator.start);

        layout.assign(rotateWrist, operator.rightStickY.scaledBy(-0.5));
        layout.assign(rotateElbow, operator.leftStickY.scaledBy(-0.5));

        layout.assign(elevator, operator.pov.y.scaledBy(-0.25));
        layout.assign(carriage, operator.pov.x.scaledBy(0.75));

        return layout;
    }
}
