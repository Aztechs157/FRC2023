// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.assabet.aztechs157.numbers.Range;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.drive.SwervePod;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class CosmeticConstants {
        public static final int LIGHT_ID = 0;
        public static final double SOLID_YELLOW_VALUE = 0.69;
        public static final double SOLID_PURPLE_VALUE = 0.91;
        public static final int LIGHT_LENGTH = 69;
    }

    public static class DriveConstants {
        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

        public static final double DRIVE_SLEW_RATE = 1;
        public static final double ANGLE_KP = 0.01;
        public static final double ANGLE_KD = 0.00009;

        public static final SwervePod.Config[] POD_CONFIGS = new SwervePod.Config[] {
                new SwervePod.Config(1, 2, 3, false),
                new SwervePod.Config(4, 5, 6, false),
                new SwervePod.Config(7, 8, 9, false),
                new SwervePod.Config(10, 11, 12, false),
        };

        private static final double CENTER_TO_POD_METER = Units.inchesToMeters(9.25);

        public static final Translation2d[] WHEEL_LOCATIONS = new Translation2d[] {
                new Translation2d(-CENTER_TO_POD_METER, CENTER_TO_POD_METER),
                new Translation2d(CENTER_TO_POD_METER, CENTER_TO_POD_METER),
                new Translation2d(CENTER_TO_POD_METER, -CENTER_TO_POD_METER),
                new Translation2d(-CENTER_TO_POD_METER, -CENTER_TO_POD_METER)
        };

        public static final double SLEWRATE_VAL = 0.7;
        public static final double SLEW_ROTATE_VAL = 100;
        public static final double AUTO_SLEW_RATE = 1;
        public static final double AUTO_SLEW_ROTATE_VAL = 100;
    }

    public static class AutoConstants {
        public static final ChassisSpeeds AUTO_SPEEDS = new ChassisSpeeds(-0.3, 0, 0);
        public static final double AUTO_DISTANCE = 3000000; // This value isn't used but will be if distance based
                                                            // autonomous is ever made, change if that is the case
        public static final double EJECT_SPEED = 0.3;
        public static final double EJECT_TIME = 0.3;
        public static final double START_TO_LOADING_TIME = 1;
        public static final double LOADING_TO_START_TIME = 1;
        public static final double TURN_ACCURACY_DEG = 20;
        public static final double DRIVE_ACCURACY = 20;
        public static final double BALANCE_ACCURACY_DEG = 2.5;
        public static final double MAX_SPEED = 2.596896;
    }

    public static class IntakeConstants {
        public static final int MOTOR_ID = 13;
        public static final int SOLENOID_FORWARD_ID = 0;
        public static final int SOLENOID_BACKWARD_ID = 1;
        public static final int PNEUMATICS_HUB_ID = 51;
        public static final int INTAKE_SENSOR_ID = 3;
    }

    public static class WristConstants {
        public static final int ABS_ENCODER_ROTATION_ID = 2;
        public static final int MOTOR_ID = 14;
        public static final Range OLD_LIMITS = new Range(36, 179);
        public static final Range ROTATE_LIMITS = new Range(36, 179); // 280 is full max

        public static final double START_POS = toNewRange(140, OLD_LIMITS, ROTATE_LIMITS);// 140;
        public static final double LOW_POS = toNewRange(136, OLD_LIMITS, ROTATE_LIMITS);// 140;
        public static final double MID_POS = toNewRange(64 + 10, OLD_LIMITS, ROTATE_LIMITS);// 72;
        public static final double LOADING_POS = toNewRange(79, OLD_LIMITS, ROTATE_LIMITS);// 75;
        public static final double HIGH_POS = toNewRange(79 + 10, OLD_LIMITS, ROTATE_LIMITS);// 72;

        public static final double START_POS_MIN_ARM = toNewRange(200, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 60;
        public static final double LOW_POS_MIN_ARM = toNewRange(60, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 60;
        public static final double MID_POS_MIN_ARM = toNewRange(103, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 103;
        public static final double LOADING_POS_MIN_ARM = toNewRange(103, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 103;
        public static final double HIGH_POS_MIN_ARM = toNewRange(103, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 103;

        public static final double HIGH_POS_MAX_ELEVATOR = toNewRange(1400, ElevatorConstants.OLD_LIMITS,
                ElevatorConstants.ELEVATOR_LIMITS);
        public static final double OTHER_POS_ELEVATOR = 99999;

    }

    public static class ElbowConstants {
        public static final int ABS_ENCODER_ROTATION_ID = 1;
        public static final int MOTOR_ID = 15;
        public static final int SERVO_ID = 1;

        public static final Range OLD_LIMITS = new Range(60, 230);
        public static final Range ROTATE_LIMITS = new Range(139, 222); // 162 fully down, 336 fully up

        public static final double START_POS = toNewRange(230, OLD_LIMITS, ROTATE_LIMITS); // 230;
        public static final double LOW_POS = toNewRange(60 + 5, OLD_LIMITS, ROTATE_LIMITS);// 60;
        public static final double MID_POS = toNewRange(218, OLD_LIMITS, ROTATE_LIMITS);// 218;
        public static final double LOADING_POS = toNewRange(187, OLD_LIMITS, ROTATE_LIMITS);// 187;
        public static final double HIGH_POS = toNewRange(190, OLD_LIMITS, ROTATE_LIMITS);// 190;

        public static final double START_POS_MIN_CARRIAGE = toNewRange(1790, CarriageConstants.OLD_LIMITS,
                CarriageConstants.CARRIAGE_LIMITS);// 1790;
        public static final double LOW_POS_MIN_CARRIAGE = toNewRange(2159 + 1000, CarriageConstants.OLD_LIMITS,
                CarriageConstants.CARRIAGE_LIMITS);// 2159;
        public static final double MID_POS_MIN_CARRIAGE = toNewRange(1790, CarriageConstants.OLD_LIMITS,
                CarriageConstants.CARRIAGE_LIMITS);// 1790;
        public static final double LOADING_POS_MIN_CARRIAGE = toNewRange(1790, CarriageConstants.OLD_LIMITS,
                CarriageConstants.CARRIAGE_LIMITS);// 1790;
        public static final double HIGH_POS_MIN_CARRIAGE = toNewRange(1790, CarriageConstants.OLD_LIMITS,
                CarriageConstants.CARRIAGE_LIMITS);// 1790;

        public static final double START_POS_MIN_WRIST = toNewRange(30, WristConstants.OLD_LIMITS,
                WristConstants.ROTATE_LIMITS);// 36;
        public static final double LOW_POS_MIN_WRIST = toNewRange(121, WristConstants.OLD_LIMITS,
                WristConstants.ROTATE_LIMITS);// 121;
        public static final double MID_POS_MIN_WRIST = toNewRange(30, WristConstants.OLD_LIMITS,
                WristConstants.ROTATE_LIMITS);// 36;
        public static final double LOADING_POS_MIN_WRIST = toNewRange(30, WristConstants.OLD_LIMITS,
                WristConstants.ROTATE_LIMITS);// 36;
        public static final double HIGH_POS_MIN_WRIST = toNewRange(30, WristConstants.OLD_LIMITS,
                WristConstants.ROTATE_LIMITS);// 36;

        public static final double OTHER_POS_MIN_ELEVATOR = toNewRange(1675, ElevatorConstants.OLD_LIMITS,
                ElevatorConstants.ELEVATOR_LIMITS);
        public static final double HIGH_POS_MIN_ELEVATOR = toNewRange(1475, ElevatorConstants.OLD_LIMITS,
                ElevatorConstants.ELEVATOR_LIMITS);

        public static final double SAFETY_ELEVATOR_LIMIT_HIGH = toNewRange(1520, ElevatorConstants.OLD_LIMITS,
                ElevatorConstants.ELEVATOR_LIMITS);
    }

    public static class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ID = 16;
        public static final Range OLD_LIMITS = new Range(1315, 1675);
        public static final Range ELEVATOR_LIMITS = new Range(170, 1580); // end is the bottom most and start is
                                                                          // the top most
        public static final int ELEVATOR_ANALOG_ID = 3;

        public static final double START_POS = toNewRange(1675, OLD_LIMITS, ELEVATOR_LIMITS);
        public static final double LOW_POS = toNewRange(1675, OLD_LIMITS, ELEVATOR_LIMITS);
        public static final double MID_POS = toNewRange(1698 - 100, OLD_LIMITS, ELEVATOR_LIMITS);
        public static final double LOADING_POS = toNewRange(1520 - 100, OLD_LIMITS, ELEVATOR_LIMITS);
        public static final double HIGH_POS = toNewRange(1330 - 50, OLD_LIMITS, ELEVATOR_LIMITS);

        public static final double SLEW_POSITIVE_VAL = 10;
        public static final double SLEW_NEGATIVE_VAL = -1;

        public static final double SAFETY_ELBOW_LIMIT_START_MID = toNewRange(190, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 190;
        public static final double SAFETY_ELBOW_LIMIT_LOW = toNewRange(60, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 60;
        public static final double SAFETY_ELBOW_LIMIT_HIGH = toNewRange(157, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 157;
        public static final double SAFETY_WRIST_LIMIT_START_MID = toNewRange(30, WristConstants.OLD_LIMITS,
                WristConstants.ROTATE_LIMITS);// 60;
        public static final double SAFETY_WRIST_LIMIT_LOW = toNewRange(121, WristConstants.OLD_LIMITS,
                WristConstants.ROTATE_LIMITS);// 121;
        public static final double SAFETY_WRIST_LIMIT_HIGH = toNewRange(1000, WristConstants.OLD_LIMITS,
                WristConstants.ROTATE_LIMITS);// 100;
        public static final double SAFETY_CARRIAGE_LIMIT_LOW = toNewRange(3075, CarriageConstants.OLD_LIMITS,
                CarriageConstants.CARRIAGE_LIMITS);// 3075;
    }

    public static class CarriageConstants {
        public static final int CARRIAGE_MOTOR_ID = 17;
        public static final Range OLD_LIMITS = new Range(2015, 3350);
        public static final Range CARRIAGE_LIMITS = new Range(2780, 3950);
        public static final int CARRIAGE_ANALOG_ID = 0;

        public static final double START_POS = toNewRange(2165, OLD_LIMITS, CARRIAGE_LIMITS);// 2015;
        public static final double LOW_POS = toNewRange(3430, OLD_LIMITS, CARRIAGE_LIMITS);// 3350;
        public static final double MID_POS = toNewRange(3430, OLD_LIMITS, CARRIAGE_LIMITS);// 3350;
        public static final double LOADING_POS = toNewRange(3430, OLD_LIMITS, CARRIAGE_LIMITS);// 3350;
        public static final double HIGH_POS = toNewRange(3430, OLD_LIMITS, CARRIAGE_LIMITS);// 3350;

        public static final double START_POS_MIN_ELBOW = toNewRange(133, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 133;
        public static final double LOW_POS_MIN_ELBOW = toNewRange(42, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 42;
        public static final double MID_POS_MIN_ELBOW = toNewRange(103, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 103;
        public static final double LOADING_POS_MIN_ELBOW = toNewRange(103, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 103;
        public static final double HIGH_POS_MIN_ELBOW = toNewRange(103, ElbowConstants.OLD_LIMITS,
                ElbowConstants.ROTATE_LIMITS);// 103;
    }

    // TODO may not work if start is the smaller value, fix this before using on the
    // elevator
    public static double toNewRange(double oldVal, Range oldRange, Range newRange) {
        double conversion = (newRange.end() - newRange.start()) / (oldRange.end() - oldRange.start());
        return ((oldVal - oldRange.start()) * conversion) + newRange.start();

    }
}
