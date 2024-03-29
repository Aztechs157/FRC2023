// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.cosmetics.PwmLEDs;
import frc.robot.input.DriverInputs;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
    private final DoubleSolenoid solenoid = new DoubleSolenoid(
            IntakeConstants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH,
            IntakeConstants.SOLENOID_FORWARD_ID,
            IntakeConstants.SOLENOID_BACKWARD_ID);
    private final DigitalInput intakeSensor = new DigitalInput(IntakeConstants.INTAKE_SENSOR_ID);
    private final Compressor airCompressor = new Compressor(IntakeConstants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH);
    private boolean isOpen = false;
    private final PwmLEDs lights;

    private double stoppedSpeed = 0.0;

    public IntakeSubsystem(PwmLEDs lights) {
        airCompressor.enableDigital();
        this.lights = lights;
        final var tab = Shuffleboard.getTab("Encoder Debug");
        tab.addBoolean("intake sensor", this::getSensor);
    }

    public Command coneMode(final DriverInputs inputs, final IntakeSubsystem intakesubsystem) {
        Command x = runOnce(() -> {
            lights.setClimb(Color.kGold, Color.kBlack, 3, 2, 2);
            setSolenoid(Value.kForward);
            stoppedSpeed = 0.25;
        }).andThen(runIntake(inputs).until(this::getSensor)).andThen(runOnce(() -> {
            setSolenoid(Value.kReverse);
            lights.setSolid(Color.kGold);
            stoppedSpeed = 0;
        }));
        // System.out.println("coneMode requires intake: " + x.hasRequirement(this));
        return x;
    }

    public Command cubeMode(final DriverInputs inputs, final IntakeSubsystem intakeSubsystem) {
        Command x = runOnce(() -> {
            lights.setClimb(Color.kPurple, Color.kBlack, 3, 2, 2);
            setSolenoid(Value.kForward);
            stoppedSpeed = 0.25;
        }).andThen(runIntake(inputs).until(this::getSensor)).andThen(runOnce(() -> {
            lights.setSolid(Color.kPurple);
            stoppedSpeed = 0;
        }));
        // System.out.println("cubeMode requires intake: " + x.hasRequirement(this));
        return x;

    }

    public Command runIntake(final DriverInputs inputs) {
        return runEnd(() -> {
            var speed = inputs.axis(DriverInputs.intakeSpeed).get();

            final var newSpeed = Math.abs(speed) < stoppedSpeed ? stoppedSpeed : speed;
            motor.set(-newSpeed);
        }, () -> {
            motor.set(0);
        });
    }

    public Command runMotor(final double speed) {
        return runEnd(() -> {

            var stopSpeed = stoppedSpeed;
            final var newSpeed = Math.abs(speed) < stopSpeed ? stopSpeed : speed;
            motor.set(newSpeed);
        }, () -> motor.set(0));
    }

    public Command intake(final double speed) {
        return runMotor(isOpen ? speed : speed).until(this::getSensor);
    }

    public Command setSolenoidWithLights(final DoubleSolenoid.Value value) {
        return runOnce(() -> {
            solenoid.set(value);
            isOpen = value == DoubleSolenoid.Value.kForward;
            if (isOpen) {
                lights.setSolid(Color.kPurple);
                stoppedSpeed = 0;
            } else {
                lights.setSolid(Color.kYellow);
                stoppedSpeed = 0;
            }
        });
    }

    public void setSolenoid(final DoubleSolenoid.Value val) {
        isOpen = val == DoubleSolenoid.Value.kForward;
        solenoid.set(val);
    }

    private boolean getSensor() {
        return !intakeSensor.get();
    }

    // private final NetworkTableEntry sensorEntry =
    // NetworkTableInstance.getDefault().getEntry("157/Intake/Sensor");

    public Command ejectCargo() {
        return runMotor(1);
    }

    @Override
    public void periodic() {
        // sensorEntry.setBoolean(getSensor());
        // System.out.println(getSensor());
    }

}
