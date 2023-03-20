//TODO: name comments better

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.cosmetics.PwmLEDs;
import frc.robot.drive.AutoBalance;
import frc.robot.drive.AutoDrive;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.FullDrive;
import frc.robot.drive.DriveSubsystem.AutoDriveLineBuilder;
import frc.robot.elbow.ElbowSubsystem;
import frc.robot.input.DriverInputs;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.wrist.WristSubsystem;
import frc.robot.lift.CarriageSubsystem;
import frc.robot.lift.ElevatorSubsystem;
import frc.robot.statemachines.SubsystemGroup;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final PwmLEDs lightsSubsystem = new PwmLEDs();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(lightsSubsystem);
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final ElbowSubsystem elbowSubsystem = new ElbowSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CarriageSubsystem carriageSubsystem = new CarriageSubsystem();
    private final SubsystemGroup group = new SubsystemGroup(elevatorSubsystem, carriageSubsystem, elbowSubsystem,
            wristSubsystem);

    private final DriverInputs driverInputs = new DriverInputs();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        CameraServer.startAutomaticCapture();

        driveSubsystem.setDefaultCommand(new FullDrive(driveSubsystem, driverInputs));

        wristSubsystem.setDefaultCommand(wristSubsystem.runWrist(driverInputs));
        elbowSubsystem.setDefaultCommand(elbowSubsystem.runElbow(driverInputs));

        elevatorSubsystem.setDefaultCommand(elevatorSubsystem.runElevator(driverInputs));
        carriageSubsystem.setDefaultCommand(carriageSubsystem.runCarriage(driverInputs));

        intakeSubsystem.setDefaultCommand(intakeSubsystem.runIntake(driverInputs));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxControllerXbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4ControllerPS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick}
     * Flight joysticks.
     */
    private void configureBindings() {
        driverInputs.button(DriverInputs.autoBalance).whileHeld(new AutoBalance(driveSubsystem));
        driverInputs.button(DriverInputs.lowPosition).whileHeld(group.lowPosCommand(1));
        driverInputs.button(DriverInputs.midPosition).whileHeld(group.midPosConeCommand(1));
        driverInputs.button(DriverInputs.loadingPosition).whileHeld(group.loadingPosCommand(1));
        driverInputs.button(DriverInputs.highPosition).whileHeld(group.highPosCommand(1));
        driverInputs.button(DriverInputs.startPosition).whileHeld(group.startingPosCommand(1));
        driverInputs.button(DriverInputs.setIntakeSolenoidForward)
                .whenPressed(intakeSubsystem.setSolenoid(DoubleSolenoid.Value.kForward));
        driverInputs.button(DriverInputs.setIntakeSolenoidBackward)
                .whenPressed(intakeSubsystem.setSolenoid(DoubleSolenoid.Value.kReverse));
    }

    public SendableChooser<Command> chooser = new SendableChooser<>();
    {
        Shuffleboard.getTab("Driver").add("Auto Choose", chooser);
        chooser.setDefaultOption("scoreHighThenLeaveCommunityThenEngage", scoreHighThenLeaveCommunityThenEngage());
        chooser.addOption("WristDownThenEjectThenRunDistance", WristDownThenEjectThenRunDistance());
        chooser.addOption("WristDownThenEjectThenPoorlyDock", WristDownThenEjectThenPoorlyDock());
        chooser.addOption("WristDownThenEjectThenBetterDock", WristDownThenEjectThenBetterDock());
        chooser.addOption("WristDownThenEjectThenLeaveCommunityThenBetterDock",
                WristDownThenEjectThenLeaveCommunityThenBetterDock());
        chooser.addOption("scoreHighThenRunDistance", scoreHighThenRunDistance());
        chooser.addOption("scoreHighThenEngage", scoreHighThenEngage());
        chooser.addOption("leaveCommunityThenEngage", leaveCommunityThenEngage());
        chooser.addOption("everythingIsBrokenDoNothing", new InstantCommand(() -> System.out.println(":(")));
        chooser.addOption("twoPiecethenEngage", TwoPieceThenEngage());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return scoreHighThenLeaveCommunityThenEngage();
        return new ProxyCommand(chooser::getSelected);
    }

    // Do not use unless very specific case calls for it (IE: ONLY DRIVE IS WORKING,
    // IN WHICH CASE YOU SHOULD BE PANICKING)
    public Command runDistanceWithSpeeds(double x, double y, double dist) {
        return driveSubsystem.resetDrivePositionCommand()
                .andThen(driveSubsystem.driveRawDistanceCommand(new ChassisSpeeds(x, y, 0), dist));
    }

    // Do not use unless very specific case calls for it (INCASE WE WANT TO SCORE
    // MID)
    public Command WristDownThenEjectThenRunDistance() {
        return driveSubsystem.addGyroOffset(180.0f).andThen(wristSubsystem.turnDownToPos(180))
                .andThen(intakeSubsystem.ejectCargo().withTimeout(0.5))
                .andThen(runDistanceWithSpeeds(-0.3, 0.0, 3000.0).withTimeout(4.2));
    }

    // Do not use unless very specific case calls for it (IE: THE GYRO DOESN'T WORK
    // FOR WHATEVER REASON)
    public Command WristDownThenEjectThenPoorlyDock() {
        return driveSubsystem.addGyroOffset(180.0f).andThen(wristSubsystem.turnDownToPos(180))
                .andThen(intakeSubsystem.ejectCargo().withTimeout(0.5))
                .andThen(runDistanceWithSpeeds(-0.5, 0.0, 3000.0).withTimeout(1.75))
                .andThen(driveSubsystem.driveRawDistanceCommand(
                        new ChassisSpeeds(0, 0, 0.001),
                        100000));
    }

    // Do not use unless very specific case calls for it (IE: OUR STATES AREN'T
    // WORKING)
    public Command WristDownThenEjectThenBetterDock() {
        return driveSubsystem.addGyroOffset(180.0f).andThen(wristSubsystem.turnDownToPos(75))
                .andThen(intakeSubsystem.ejectCargo().withTimeout(0.5))
                .andThen(runDistanceWithSpeeds(-0.5, 0.0, 3000.0).withTimeout(1.75))
                .andThen(new AutoBalance(driveSubsystem));
    }

    // Do not use unless very specific case calls for it (IE: OUR STATES AREN'T
    // WORKING)
    public Command WristDownThenEjectThenLeaveCommunityThenBetterDock() {
        return driveSubsystem.addGyroOffset(180.0f).andThen(wristSubsystem.turnDownToPos(90))
                .andThen(intakeSubsystem.ejectCargo().withTimeout(0.5))
                .andThen(runDistanceWithSpeeds(-0.5, 0.0, 6000.0).withTimeout(2.9))
                .andThen(runDistanceWithSpeeds(0.5, 0.0, -3000.0).withTimeout(1.85))
                .andThen(new AutoBalance(driveSubsystem));
    }

    // SCORES A CUBE HIGH THEN LEAVES COMMUNITY
    public Command scoreHighThenRunDistance() {
        return new SequentialCommandGroup(driveSubsystem.addGyroOffset(180),
                group.midPosConeCommand(1).withTimeout(1.3),
                intakeSubsystem.runMotor(-1).withTimeout(0.3),
                group.startingPosCommand(1).withTimeout(1.4),
                runDistanceWithSpeeds(-0.3, 0.0, 3000.0).withTimeout(4.2));
    }

    // SCORES A CUBE HIGH THEN ENGAGES ON CHARGING PLATFORM WITHOUT LEAVING
    // COMMUNITY
    public Command scoreHighThenEngage() {
        return new SequentialCommandGroup(driveSubsystem.addGyroOffset(180),
                group.midPosConeCommand(1).withTimeout(1.3),
                intakeSubsystem.runMotor(-1).withTimeout(0.3),
                group.startingPosCommand(1).withTimeout(1.4),
                runDistanceWithSpeeds(-0.5, 0.0, -3000.0).withTimeout(1.75),
                new AutoBalance(driveSubsystem));
    }

    // SCORES A CUBE HIGH THEN LEAVES COMMUNITY THEN ENGAGES ON CHARING PLATFORM
    public Command scoreHighThenLeaveCommunityThenEngage() {
        return new SequentialCommandGroup(driveSubsystem.addGyroOffset(180),
                group.highPosCommand(1).withTimeout(1.3),
                intakeSubsystem.runMotor(-1).withTimeout(0.4),
                group.startingPosCommand(1).withTimeout(1.4),
                wristSubsystem.stopWrist(),
                runDistanceWithSpeeds(-0.5, 0.0, 6000.0).withTimeout(2.9),
                runDistanceWithSpeeds(0.5, 0.0, -3000.0).withTimeout(1.85),
                new AutoBalance(driveSubsystem));
    }

    public Command leaveCommunityThenEngage() {
        return new SequentialCommandGroup(
                driveSubsystem.addGyroOffset(180),
                runDistanceWithSpeeds(-0.5, 0.0, 6000.0).withTimeout(2.9),
                runDistanceWithSpeeds(0.5, 0.0, -3000.0).withTimeout(1.85),
                new AutoBalance(driveSubsystem));
    }

    public Command TwoPieceThenEngage() {
        // for wpi, might need to change desired angle and definitely distance gone
        // Figure out what side this works on, then mirror it for the opposite color
        return new SequentialCommandGroup(intakeSubsystem.intake(-1).withTimeout(0.75),
                new ParallelCommandGroup(driveSubsystem.driveWithRotation(0, 1, 0),
                        group.lowPosCommand(1),
                        intakeSubsystem.intake(1)),
                new ParallelCommandGroup(driveSubsystem.driveWithRotation(180, -1, 0),
                        group.startingPosCommand(1),
                        intakeSubsystem.intake(0.1)).withTimeout(3),
                group.highPosCommand(1).withTimeout(1.3),
                intakeSubsystem.ejectCargo().withTimeout(0.5),
                new ParallelCommandGroup(group.startingPosCommand(1),
                        new WaitCommand(0.5).andThen(driveSubsystem.driveWithRotation(0, 0.5, .5)))
                        .withTimeout(0.5 + 1.75), // first value is the wait, second value is the drive time, and maybe
                // increase Y to adjust for charge station (if hit charge station
                // side, increase Y) maybe add a forward to get further up platform
                new AutoBalance(driveSubsystem));
    }

    public Command TwoPiecethenEngageWithOdometry() {
        return new SequentialCommandGroup(intakeSubsystem.intake(-1).withTimeout(0.75),
                new ParallelRaceGroup(
                        group.lowPosCommand(1),
                        intakeSubsystem.intake(1),
                        new AutoDrive(driveSubsystem, new AutoDriveLineBuilder(5, 0, 0)
                                .startRotAtX(3.25)
                                .useSlewXY(true)
                                .xTolerance(0.05)
                                .holdRotTillRotStarts(true))),
                new ParallelRaceGroup(group.startingPosCommand(1),
                        intakeSubsystem.intake(0.1),
                        new AutoDrive(driveSubsystem, new AutoDriveLineBuilder(0, 0, 180)
                                .xTolerance(0.05)
                                .useSlewAll(true))),
                group.highPosCommand(1).withTimeout(1.3),
                intakeSubsystem.intake(-1).withTimeout(0.5),
                new ParallelRaceGroup(group.startingPosCommand(1),
                        new AutoDrive(driveSubsystem, new AutoDriveLineBuilder(1.75, -1.75, 0)
                                .startTime(0.5)
                                .startXAtY(0.5)
                                .holdXTillXStarts(true)
                                .useSlewAll(true)
                                .maxXSpeed(0.5)
                                .usePidX(false)
                                .xTolerance(0.1))),
                new AutoBalance(driveSubsystem));
    }

    public Command trajectoryFollower() {
        TrajectoryConfig trajConf = new TrajectoryConfig(AutoConstants.MAX_SPEED, AutoConstants.MAX_ACCELL);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(
                new Translation2d(1, 0),
                new Translation2d(1, 1)), new Pose2d(0, 1, Rotation2d.fromDegrees(180)), trajConf);
        return trajectoryFollower(traj);
    }

    public Command trajectoryFollower(Trajectory traj) {
        ProfiledPIDController pidr = new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(Math.PI / 3, Math.PI / 6));
        pidr.enableContinuousInput(-Math.PI, Math.PI);
        return new SwerveControllerCommand(traj, driveSubsystem::getOdometryPose, driveSubsystem.kinematics,
                driveSubsystem.pidx, driveSubsystem.pidy, pidr, driveSubsystem::setRaw, driveSubsystem);
    }

    /*
     * min arm with low carriage: 227
     * max arm overall: 151
     * min arm overall: 255
     *
     *
     * max wrist: 258
     * min wrist(?): 165
     *
     * max carriage: 2348
     * min carriage: 952
     *
     * min elevator: 2034
     * max elevator: 550
     *
     */
}
