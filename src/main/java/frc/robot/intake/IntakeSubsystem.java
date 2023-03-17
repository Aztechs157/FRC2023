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

    public IntakeSubsystem(PwmLEDs lights) {
        airCompressor.enableDigital();
        this.lights = lights;
    }

    public Command runIntake(final DriverInputs inputs) {
        return runEnd(() -> {
            final var speed = inputs.axis(DriverInputs.intakeSpeed).get();
            motor.set(-speed);
        }, () -> motor.set(0));
    }

    public Command runMotor(final double speed) {
        return runEnd(() -> motor.set(speed), () -> motor.set(0));
    }

    public Command intake(final double speed) {
        return runMotor(isOpen ? speed : speed).until(this::getSensor);
    }

    public Command setSolenoid(final DoubleSolenoid.Value value) {
        return runOnce(() -> {
            solenoid.set(value);
            isOpen = value == DoubleSolenoid.Value.kForward;
            if (isOpen) {
                lights.setColor1(Color.kPurple);
            } else {
                lights.setColor2(Color.kYellow);
            }
        });
    }

    private boolean getSensor() {
        return !intakeSensor.get();
    }

    // private final NetworkTableEntry sensorEntry =
    // NetworkTableInstance.getDefault().getEntry("157/Intake/Sensor");

    public Command ejectCargo() {
        return runOnce(() -> System.out.println("ejecting!!!")).andThen(runMotor(1));
    }

    @Override
    public void periodic() {
        // sensorEntry.setBoolean(getSensor());
        // System.out.println(getSensor());
    }

}
