// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
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
    public static class OperatorConstants {
        public static final int DRIVER_INPUT_ID = 0;
    }

    public static class DriveConstants {
        public static final IdleMode ROLL_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode SPIN_IDLE_MODE = IdleMode.kCoast;

        public static final double ROLL_SLEW_RATE = 1;
        public static final double SPIN_KP = 0.01;
        public static final double SPIN_KD = 0.00009;

        public static final SwervePod.Config[] POD_CONFIGS = new SwervePod.Config[] {
                new SwervePod.Config(1, 2, 3, false),
                new SwervePod.Config(4, 5, 6, false),
                new SwervePod.Config(7, 8, 9, false),
                new SwervePod.Config(10, 11, 12, false),
        };

        public static final double CENTER_TO_POD_METER = Units.inchesToMeters(9.25);
        public static final Translation2d[] WHEEL_LOCATIONS = new Translation2d[] {
                new Translation2d(-CENTER_TO_POD_METER, CENTER_TO_POD_METER),
                new Translation2d(CENTER_TO_POD_METER, CENTER_TO_POD_METER),
                new Translation2d(CENTER_TO_POD_METER, -CENTER_TO_POD_METER),
                new Translation2d(-CENTER_TO_POD_METER, -CENTER_TO_POD_METER),
        };
    }
}
