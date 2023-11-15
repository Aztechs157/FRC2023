// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.input.DriverInputs;
import frc.robot.lib.NumberUtil;
import frc.robot.statemachines.SubsystemGroup.SafetyLogic;

public class WristSubsystem extends SubsystemBase {

    private final CANSparkMax wristMotor = new CANSparkMax(WristConstants.MOTOR_ID, MotorType.kBrushless);
    private final Counter wristAbsEncoder = new Counter(Mode.kSemiperiod);
    private double wristSpeed = 0.0;
    private Double positiveInfinity = Double.POSITIVE_INFINITY;
    private Double negativeInfinity = Double.NEGATIVE_INFINITY;

    public WristSubsystem() {
        wristMotor.setInverted(true);
        wristMotor.setIdleMode(WristConstants.WRIST_IDLE_MODE);
        wristAbsEncoder.setSemiPeriodMode(true);
        wristAbsEncoder.setUpSource(WristConstants.ABS_ENCODER_ROTATION_ID);
        wristAbsEncoder.reset();
    }

    public Command runWrist(final DriverInputs inputs) {
        return runEnd(() -> {
            final double speed = inputs.axis(DriverInputs.rotateWrist).get();
            final double position = getWristRotationPosition();
            if (position != positiveInfinity && position != negativeInfinity) {
                rotateWrist(speed);

            } else {
                wristMotor.set(speed);
            }
        }, () -> rotateWrist(0));
    }

    public Command runWristSpeed(final double speed) {
        return runEnd(() -> rotateWrist(speed), () -> rotateWrist(0));
    }

    public Command stopWrist() {
        return runOnce(() -> rotateWrist(0));
    }

    public double getWristRotationPosition() {
        return NumberUtil.ticksToDegs(wristAbsEncoder.getPeriod());
    }

    public void rotateWrist(final double speed) {
        wristSpeed = speed;

        final double limitedSpeed = WristConstants.ROTATE_LIMITS.limitMotionWithinRange(
                speed, getWristRotationPosition());
        wristMotor.set(limitedSpeed);
    }

    public void test() {
        final var testTable = table.getSubTable("Test1");
        final var output = testTable.getEntry("Output");

        final var speedEntry = testTable.getEntry("Speed Input");
        speedEntry.setDefaultNumber(0);
        final var speed = speedEntry.getNumber(0).doubleValue();

        final var encoderEntry = testTable.getEntry("Encoder Input");
        encoderEntry.setDefaultNumber(0);
        final var position = encoderEntry.getNumber(0).doubleValue();

        final double limitedSpeed = WristConstants.ROTATE_LIMITS.limitMotionWithinRange(speed, position);
        output.setNumber(limitedSpeed);
    }

    public void stop() {
        runWristSpeed(0);
    }

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("157/Arm");

    @Override
    public void periodic() {
        table.getEntry("Wrist").setNumber(getWristRotationPosition());
        table.getEntry("WristSpeed").setNumber(wristSpeed);
        test();
    }

    public final Command turnDownToPos(double pos) {
        return runWristSpeed(-0.3).until(() -> getWristRotationPosition() < pos);
    }

    public static class WristState implements SafetyLogic {

        private double wristPosition;
        public static PIDController pid = new PIDController(0.01, 0, 0);
        private double minElbowPos;
        private double maxElevatorPos;

        public WristState(final double wristPosition, final double minArmPos, final double maxElevatorPos) {
            this.wristPosition = wristPosition;
            this.minElbowPos = minArmPos;
            this.maxElevatorPos = maxElevatorPos;
        }

        public static final WristState start = new WristState(WristConstants.START_POS,
                WristConstants.START_POS_MIN_ARM, WristConstants.OTHER_POS_ELEVATOR);
        public static final WristState low = new WristState(WristConstants.LOW_POS, WristConstants.LOW_POS_MIN_ARM,
                WristConstants.OTHER_POS_ELEVATOR);
        public static final WristState mid = new WristState(WristConstants.MID_POS, WristConstants.MID_POS_MIN_ARM,
                WristConstants.OTHER_POS_ELEVATOR);
        public static final WristState loading = new WristState(WristConstants.LOADING_POS,
                WristConstants.LOADING_POS_MIN_ARM, WristConstants.OTHER_POS_ELEVATOR);
        public static final WristState high = new WristState(WristConstants.HIGH_POS, WristConstants.HIGH_POS_MIN_ARM,
                WristConstants.HIGH_POS_MAX_ELEVATOR);

        @Override
        public SafetyLogic lowPosition() {
            return low;
        }

        @Override
        public SafetyLogic highPosition() {
            return high;
        }

        @Override
        public SafetyLogic loadingPosition() {
            return loading;
        }

        @Override
        public SafetyLogic defaultPosition() {
            return start;
        }

        @Override
        public double stateCalculate(double speed, double elbowPosition, double wristPosition, double elevatorPosition,
                double carriagePosition) {
            // Runs the wrist for the states
            double pidVal = pid.calculate(wristPosition, this.wristPosition);
            if ((elbowPosition > this.minElbowPos && elevatorPosition < this.maxElevatorPos) || pidVal > 0) {
                return pidVal;
            }

            return 0;
        }

        @Override
        public SafetyLogic midPosition() {
            return mid;
        }

    }

}
