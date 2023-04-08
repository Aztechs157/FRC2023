// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lift;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.input.DriverInputs;
import frc.robot.lib.NumberUtil;
import frc.robot.statemachines.SubsystemGroup.SafetyLogic;

public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax elevatorMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID,
            MotorType.kBrushless);
    private final Counter elevatorAbsEncoder = new Counter(Mode.kSemiperiod);
    private double elevatorSpeed = 0.0;

    public void reset() {
        ElevatorState.slew.reset(0);
    }

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorAbsEncoder.setSemiPeriodMode(true);
        elevatorAbsEncoder.setUpSource(ElevatorConstants.ABS_ENCODER_ROTATION_ID);
        elevatorAbsEncoder.reset();
    }

    public Command runElevator(final DriverInputs inputs) {
        return runEnd(() -> {
            final double speed = inputs.axis(DriverInputs.elevator).get();
            runElevatorMotor(-speed);
        }, () -> runElevatorMotor(0));
    }

    public double getElevatorPosition() {
        return NumberUtil.ticksToDegs(elevatorAbsEncoder.getPeriod()); // kinda a hack, same dealio as carriage
    }

    public void runElevatorMotor(final double speed) {
        elevatorSpeed = speed;

        final double elevatorLimits = ElevatorConstants.ELEVATOR_LIMITS.limitMotionWithinRange(speed,
                getElevatorPosition());
        elevatorMotor.set(elevatorLimits);
    }

    public void stop() {
        runElevatorMotor(0);
    }

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("157/Elevator");

    @Override
    public void periodic() {
        table.getEntry("Elevator").setNumber(getElevatorPosition());
        table.getEntry("ElevatorSpeed").setNumber(elevatorSpeed);
    }

    public static final PIDController mainPid = new PIDController(0.01, 0, 0);

    public static class ElevatorState implements SafetyLogic {
        public final double elevatorPosition;
        public final PIDController elevatorUpPid;
        public final PIDController elevatorDownPid;
        public static final SlewRateLimiter slew = new SlewRateLimiter(ElevatorConstants.SLEW_POSITIVE_VAL,
                ElevatorConstants.SLEW_NEGATIVE_VAL, 0);
        private ElevatorStates state;

        public enum ElevatorStates {
            start, low, high, mid, loading
        }

        public ElevatorState(final double elevatorPosition, final PIDController elevatorUpPid,
                final PIDController elevatorDownPid, ElevatorStates state) {
            this.elevatorPosition = elevatorPosition;
            this.elevatorUpPid = elevatorUpPid;
            this.elevatorDownPid = elevatorDownPid;
            this.state = state;
        }

        public static final ElevatorState start = new ElevatorState(
                ElevatorConstants.START_POS, mainPid, mainPid, ElevatorStates.start);
        public static final ElevatorState low = new ElevatorState(
                ElevatorConstants.LOW_POS, mainPid, mainPid, ElevatorStates.low);
        // the min wrist position theoretically will work at 157, however, may not be
        // safe, so will likely need some testing and logic for a safer min pos.
        public static final ElevatorState mid = new ElevatorState(
                ElevatorConstants.MID_POS, mainPid, mainPid, ElevatorStates.mid);
        public static final ElevatorState loading = new ElevatorState(
                ElevatorConstants.LOADING_POS, mainPid, mainPid, ElevatorStates.loading);
        public static final ElevatorState high = new ElevatorState(
                ElevatorConstants.HIGH_POS, mainPid, mainPid, ElevatorStates.high);

        @Override
        public SafetyLogic lowPosition() {
            return low;
        }

        @Override
        public SafetyLogic midPosition() {
            return mid;
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
            // Switch case for the elevator to ensure the elevator is safe to move (no
            // crushing of other systems), check comments for each state for further details
            switch (this.state) {
                // Ensure the wrist and elbow won't be crushed when moving the elevator down
                case mid:
                case start:
                    if (elbowPosition > ElevatorConstants.SAFETY_ELBOW_LIMIT_START_MID
                            && wristPosition > ElevatorConstants.SAFETY_WRIST_LIMIT_START_MID) {
                        return mainPid.calculate(elevatorPosition, this.elevatorPosition);
                    }
                    break;
                // Ensures everything is safe to move before elevator goes too low
                case low:
                    if (elbowPosition > ElevatorConstants.SAFETY_ELBOW_LIMIT_LOW
                            && wristPosition > ElevatorConstants.SAFETY_WRIST_LIMIT_LOW
                            && carriagePosition > ElevatorConstants.SAFETY_CARRIAGE_LIMIT_LOW) {
                        return mainPid.calculate(elevatorPosition, this.elevatorPosition);
                    }
                    break;
                // Will run the elbow up if already too low to ensure it won't collide with the
                // pole, check the switch case in elbow for further detail
                case high:
                    if (elbowPosition > ElevatorConstants.SAFETY_ELBOW_LIMIT_HIGH
                            && wristPosition < ElevatorConstants.SAFETY_WRIST_LIMIT_HIGH) {
                        return mainPid.calculate(elevatorPosition, this.elevatorPosition);
                    }
                    break;
                // Runs the elevator up to loading as there are no safety limits we need to keep
                // in mind for loading
                case loading:
                    return mainPid.calculate(elevatorPosition, this.elevatorPosition);

                default:

                    break;
            }
            return 0;
        }
    }
}
