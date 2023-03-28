// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elbow;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;
import frc.robot.input.DriverInputs;
import frc.robot.lib.NumberUtil;
import frc.robot.statemachines.SubsystemGroup.SafetyLogic;

public class ElbowSubsystem extends SubsystemBase {

    private final CANSparkMax elbowMotor = new CANSparkMax(ElbowConstants.MOTOR_ID, MotorType.kBrushless);
    private final Counter elbowAbsEncoder = new Counter(Mode.kSemiperiod);
    private double elbowSpeed = 0.0;

    public ElbowSubsystem() {
        elbowMotor.setInverted(true);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        final var tab = Shuffleboard.getTab("Encoder Debug");
        tab.addNumber("elbow position", this::getElbowRotationPosition);
        elbowAbsEncoder.setSemiPeriodMode(true);
        elbowAbsEncoder.setUpSource(ElbowConstants.ABS_ENCODER_ROTATION_ID);
        elbowAbsEncoder.reset();
    }

    public Command runElbow(final DriverInputs inputs) {
        return runEnd(() -> {
            final double speed = inputs.axis(DriverInputs.rotateElbow).get();
            rotateElbow(speed);
        }, () -> rotateElbow(0));
    }

    public void stop() {
        rotateElbow(0);
    }

    public double getElbowRotationPosition() {
        return 360 - NumberUtil.ticksToDegs(elbowAbsEncoder.getPeriod());
    }

    public void rotateElbow(final double speed) {
        elbowSpeed = speed;

        final double limitedSpeed = ElbowConstants.ROTATE_LIMITS.limitMotionWithinRange(
                speed, getElbowRotationPosition());
        elbowMotor.set(limitedSpeed);
    }

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("157/Elbow");

    public static class ElbowState implements SafetyLogic {

        private double elbowPosition;
        private static PIDController mainPID = new PIDController(0.02, 0, 0.01);
        private final SlewRateLimiter slew = new SlewRateLimiter(0.1);
        private double minCarriagePos;
        private double minWristPos;
        private PIDController elbowDownPid;
        private ElbowStates state;

        public enum ElbowStates {
            start, low, mid, loading, high
        }

        public ElbowState(final double elbowPosition, final PIDController elbowDownPid,
                final double minCarriagePos, double minWristPos, ElbowStates state) {
            this.elbowPosition = elbowPosition;
            this.elbowDownPid = elbowDownPid;
            this.minCarriagePos = minCarriagePos;
            this.minWristPos = minWristPos;
            this.state = state;
        }

        public static final ElbowState start = new ElbowState(ElbowConstants.START_POS, mainPID,
                ElbowConstants.START_POS_MIN_CARRIAGE,
                ElbowConstants.START_POS_MIN_WRIST, ElbowStates.start);
        public static final ElbowState low = new ElbowState(ElbowConstants.LOW_POS, mainPID,
                ElbowConstants.LOW_POS_MIN_CARRIAGE,
                ElbowConstants.LOW_POS_MIN_WRIST, ElbowStates.low);
        public static final ElbowState mid = new ElbowState(ElbowConstants.MID_POS, mainPID,
                ElbowConstants.MID_POS_MIN_CARRIAGE,
                ElbowConstants.MID_POS_MIN_WRIST, ElbowStates.mid);
        public static final ElbowState loading = new ElbowState(ElbowConstants.LOADING_POS, mainPID,
                ElbowConstants.LOADING_POS_MIN_CARRIAGE, ElbowConstants.LOADING_POS_MIN_WRIST, ElbowStates.loading);
        public static final ElbowState high = new ElbowState(ElbowConstants.HIGH_POS, mainPID,
                ElbowConstants.HIGH_POS_MIN_CARRIAGE,
                ElbowConstants.HIGH_POS_MIN_WRIST, ElbowStates.high);

        @Override
        public SafetyLogic lowPosition() {
            return low;
        }

        @Override
        public SafetyLogic midPosition() {
            return mid;
        }

        @Override
        public SafetyLogic loadingPosition() {
            return loading;
        }

        @Override
        public SafetyLogic highPosition() {
            return high;
        }

        @Override
        public SafetyLogic defaultPosition() {
            return start;
        }

        @Override
        public double stateCalculate(double speed, double elbowPosition, double wristPosition, double elevatorPosition,
                double carriagePosition) {
            // This switch case stops the elbow from moving when going towards high position
            // if it is too low, as to stop the elbow from colliding into a pole
            var s = 0.0;
            switch (this.state) {
                case start:
                case low:
                case mid:
                case loading:
                    if (carriagePosition > this.minCarriagePos && wristPosition > this.minWristPos) {
                        s = this.slew.calculate(this.elbowDownPid.calculate(elbowPosition, this.elbowPosition));
                    }
                    break;
                case high:
                    if (elevatorPosition < ElbowConstants.SAFETY_ELEVATOR_LIMIT_HIGH
                            || elbowPosition < this.elbowPosition) {
                        s = this.slew.calculate(this.elbowDownPid.calculate(elbowPosition, this.elbowPosition));
                    }
                    break;
                default:
                    break;
            }
            return s < 0.75 ? s : 0.75;
        }

    }

    @Override
    public void periodic() {
        table.getEntry("Elbow").setNumber(getElbowRotationPosition());
        table.getEntry("ElbowSpeed").setNumber(elbowSpeed);
    }

}
