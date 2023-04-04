// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lift;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CarriageConstants;
import frc.robot.input.DriverInputs;
import frc.robot.statemachines.SubsystemGroup.SafetyLogic;

public class CarriageSubsystem extends SubsystemBase {
    private final CANSparkMax carriageMotor = new CANSparkMax(CarriageConstants.CARRIAGE_MOTOR_ID,
            MotorType.kBrushless);
    private final AnalogInput carriage10Pot = new AnalogInput(CarriageConstants.CARRIAGE_ANALOG_ID);
    private double carriageSpeed = 0.0;

    /** Creates a new CarriageSubsystem. */
    public CarriageSubsystem() {
        carriageMotor.setIdleMode(IdleMode.kBrake);
    }

    public Command runCarriage(final DriverInputs inputs) {
        return runEnd(() -> {
            final double speed = inputs.axis(DriverInputs.carriage).get();
            runCarriageMotor(speed);
        }, () -> runCarriageMotor(0));
    }

    public void stop() {
        runCarriageMotor(0);
    }

    public double getCarriagePosition() {
        return 4000 - carriage10Pot.getValue(); // is a hack, idk keep it I guess?
    }

    public void runCarriageMotor(final double speed) {
        carriageSpeed = speed;

        final double carriageLimits = CarriageConstants.CARRIAGE_LIMITS.limitMotionWithinRange(speed,
                getCarriagePosition());
        carriageMotor.set(carriageLimits);
    }

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("157/Carriage");

    @Override
    public void periodic() {
        table.getEntry("Carriage").setNumber(getCarriagePosition());
        table.getEntry("CarriageSpeed").setNumber(carriageSpeed);
    }

    public static class CarriageState implements SafetyLogic {
        private static PIDController mainPID = new PIDController(0.01, 0, 0.00);

        private double carriagePosition;
        private PIDController carriageDownPid;
        private double minElbowPos;

        public CarriageState(final double carriagePosition, final PIDController carriageDownPid,
                final double minElbowPos) {
            this.carriagePosition = carriagePosition;
            this.carriageDownPid = carriageDownPid;
            this.minElbowPos = minElbowPos;
        }

        public static final CarriageState start = new CarriageState(CarriageConstants.START_POS, mainPID,
                CarriageConstants.START_POS_MIN_ELBOW);
        public static final CarriageState low = new CarriageState(CarriageConstants.LOW_POS, mainPID,
                CarriageConstants.LOW_POS_MIN_ELBOW);
        public static final CarriageState mid = new CarriageState(CarriageConstants.MID_POS, mainPID,
                CarriageConstants.MID_POS_MIN_ELBOW);
        public static final CarriageState loading = new CarriageState(CarriageConstants.LOADING_POS, mainPID,
                CarriageConstants.LOADING_POS_MIN_ELBOW);
        public static final CarriageState high = new CarriageState(CarriageConstants.HIGH_POS, mainPID,
                CarriageConstants.HIGH_POS_MIN_ELBOW);

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
            var x = 0.0;
            // Ensures the carriage is safe to move (elbow won't get crushed)
            if (elbowPosition > this.minElbowPos) {
                // if (carriagePosition < this.carriagePosition - 100) {
                // return 0.2;
                // } else if (carriagePosition > this.carriagePosition + 100) {
                // return -0.2;
                //
                x = this.carriageDownPid.calculate(carriagePosition, this.carriagePosition);
            }
            if (Math.abs(x) > 0.9) {
                x = x > 0 ? 0.9 : -0.9;
            }
            return x;
        }

    }
}
