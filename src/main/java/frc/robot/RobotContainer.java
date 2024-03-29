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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    private final DriverInputs driverInputs = DriverInputs.createFromChooser();

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
        driverInputs.button(DriverInputs.autoBalance).whileHeld(new AutoBalance(driveSubsystem, lightsSubsystem));
        driverInputs.button(DriverInputs.lowPosition).whileHeld(group.lowPosCommand(1));
        driverInputs.button(DriverInputs.midPosition).whileHeld(group.midPosConeCommand(1));
        driverInputs.button(DriverInputs.loadingPosition).whileHeld(group.loadingPosCommand(1));
        driverInputs.button(DriverInputs.highPosition).whileHeld(group.highPosCommand(1));
        driverInputs.button(DriverInputs.startPosition).whileHeld(group.startingPosCommand(1));
        driverInputs.button(DriverInputs.setIntakeSolenoidForward)
                .whenPressed(intakeSubsystem.setSolenoidWithLights(DoubleSolenoid.Value.kForward));
        driverInputs.button(DriverInputs.setIntakeSolenoidBackward)
                .whenPressed(intakeSubsystem.setSolenoidWithLights(DoubleSolenoid.Value.kReverse));
        Command coneCommand = intakeSubsystem.coneMode(driverInputs, intakeSubsystem);
        Command cubeCommand = intakeSubsystem.cubeMode(driverInputs, intakeSubsystem);

        driverInputs.button(DriverInputs.ConeIntake).whenPressed(coneCommand);
        driverInputs.button(DriverInputs.ConeIntake).whenPressed(new InstantCommand(() -> {
            cubeCommand.cancel();
        }));
        driverInputs.button(DriverInputs.cubeIntake).whenPressed(cubeCommand);
        driverInputs.button(DriverInputs.cubeIntake).whenPressed(new InstantCommand(() -> {
            coneCommand.cancel();
        }));
    }

    public SendableChooser<Command> chooser = new SendableChooser<>();
    {
        Shuffleboard.getTab("Driver").add("Auto Choose", chooser);
        chooser.setDefaultOption("scoreHighThenLeaveCommunityThenEngage", scoreHighThenLeaveCommunityThenEngage());
        chooser.addOption("WristDownThenEjectThenRunDistance", wristDownThenEjectThenRunDistance());
        chooser.addOption("WristDownThenEjectThenPoorlyDock", wristDownThenEjectThenPoorlyDock());
        chooser.addOption("WristDownThenEjectThenBetterDock", wristDownThenEjectThenBetterDock());
        chooser.addOption("WristDownThenEjectThenLeaveCommunityThenBetterDock",
                wristDownThenEjectThenLeaveCommunityThenBetterDock());
        chooser.addOption("scoreHighThenRunDistance", scoreHighThenRunDistance());
        chooser.addOption("scoreHighThenEngage", scoreHighThenEngage());
        chooser.addOption("leaveCommunityThenEngage", leaveCommunityThenEngage());
        chooser.addOption("everythingIsBrokenDoNothing", new InstantCommand(() -> System.out.println(":(")));
        chooser.addOption("twoPiecethenEngage", twoPieceThenEngage());
        chooser.addOption("twoPieceWithOdometry", twoPieceWithOdometry());
        chooser.addOption("twoPieceThenEngageWithOdometry", twoPieceThenEngageWithOdometry());
        chooser.addOption("simpleTwoPiece", simpleTwoPiece());
        chooser.addOption("coneHigh", battlecryConeHigh());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return scoreHighThenLeaveCommunityThenEngage();
        return new ProxyCommand(chooser::getSelected);
        // return TwoPieceWithOdometry();
    }

    // Do not use unless very specific case calls for it (IE: ONLY DRIVE IS WORKING,
    // IN WHICH CASE YOU SHOULD BE PANICKING)
    public Command runDistanceWithSpeeds(double x, double y, double dist) {
        return driveSubsystem.resetDrivePositionCommand()
                .andThen(driveSubsystem.driveRawDistanceCommand(new ChassisSpeeds(x, y, 0), dist));
    }

    // Do not use unless very specific case calls for it (INCASE WE WANT TO SCORE
    // MID)
    public Command wristDownThenEjectThenRunDistance() {
        return driveSubsystem.addGyroOffset(180.0f).andThen(wristSubsystem.turnDownToPos(180))
                .andThen(intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME))
                .andThen(runDistanceWithSpeeds(-0.3, 0.0, 3000.0).withTimeout(4.2));
    }

    // Do not use unless very specific case calls for it (IE: THE GYRO DOESN'T WORK
    // FOR WHATEVER REASON)
    public Command wristDownThenEjectThenPoorlyDock() {
        return driveSubsystem.addGyroOffset(180.0f).andThen(wristSubsystem.turnDownToPos(180))
                .andThen(intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME))
                .andThen(runDistanceWithSpeeds(-0.5, 0.0, 3000.0).withTimeout(1.75))
                .andThen(driveSubsystem.driveRawDistanceCommand(
                        new ChassisSpeeds(0, 0, 0.001),
                        100000));
    }

    // Do not use unless very specific case calls for it (IE: OUR STATES AREN'T
    // WORKING)
    public Command wristDownThenEjectThenBetterDock() {
        return driveSubsystem.addGyroOffset(180.0f).andThen(wristSubsystem.turnDownToPos(75))
                .andThen(intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME))
                .andThen(runDistanceWithSpeeds(-0.5, 0.0, 3000.0).withTimeout(1.75))
                .andThen(new AutoBalance(driveSubsystem, lightsSubsystem));
    }

    // Do not use unless very specific case calls for it (IE: OUR STATES AREN'T
    // WORKING)
    public Command wristDownThenEjectThenLeaveCommunityThenBetterDock() {
        return driveSubsystem.addGyroOffset(180.0f).andThen(wristSubsystem.turnDownToPos(90))
                .andThen(intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME))
                .andThen(runDistanceWithSpeeds(-0.5, 0.0, 6000.0).withTimeout(2.9))
                .andThen(runDistanceWithSpeeds(0.5, 0.0, -3000.0).withTimeout(1.85))
                .andThen(new AutoBalance(driveSubsystem, lightsSubsystem));
    }

    // SCORES A CUBE HIGH THEN LEAVES COMMUNITY
    public Command scoreHighThenRunDistance() {
        return new SequentialCommandGroup(driveSubsystem.addGyroOffset(180),
                group.loadingPosCommand(1).withTimeout(AutoConstants.START_TO_LOADING_TIME),
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME),
                group.startingPosCommand(1).withTimeout(AutoConstants.LOADING_TO_START_TIME),
                runDistanceWithSpeeds(-0.3, 0.0, 3000.0).withTimeout(4.2));
    }

    // SCORES A CUBE HIGH THEN ENGAGES ON CHARGING PLATFORM WITHOUT LEAVING
    // COMMUNITY
    public Command scoreHighThenEngage() {
        return new SequentialCommandGroup(driveSubsystem.addGyroOffset(180),
                group.loadingPosCommand(1).withTimeout(AutoConstants.START_TO_LOADING_TIME),
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME),
                group.startingPosCommand(1).withTimeout(AutoConstants.LOADING_TO_START_TIME),
                runDistanceWithSpeeds(-0.5, 0.0, -3000.0).withTimeout(1.75),
                new AutoBalance(driveSubsystem, lightsSubsystem));
    }

    // SCORES A CUBE HIGH THEN LEAVES COMMUNITY THEN ENGAGES ON CHARING PLATFORM
    public Command scoreHighThenLeaveCommunityThenEngage() {
        return new SequentialCommandGroup(driveSubsystem.addGyroOffset(180),
                group.loadingPosCommand(1).withTimeout(AutoConstants.START_TO_LOADING_TIME), // 1.3
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME), // 0.4
                group.startingPosCommand(1).withTimeout(AutoConstants.LOADING_TO_START_TIME), // 1.4
                wristSubsystem.stopWrist(),
                runDistanceWithSpeeds(-0.5, 0.0, 6000.0).withTimeout(3),
                runDistanceWithSpeeds(0.5, 0.0, -3000.0).withTimeout(2.05),
                new AutoBalance(driveSubsystem, lightsSubsystem));
    }

    public Command leaveCommunityThenEngage() {
        return new SequentialCommandGroup(
                driveSubsystem.addGyroOffset(180),
                runDistanceWithSpeeds(-0.5, 0.0, 6000.0).withTimeout(2.9),
                runDistanceWithSpeeds(0.5, 0.0, -3000.0).withTimeout(1.85),
                new AutoBalance(driveSubsystem, lightsSubsystem));
    }

    public Command simpleTwoPiece() {
        return new SequentialCommandGroup(
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED + .15).withTimeout(AutoConstants.EJECT_TIME),
                new ParallelCommandGroup(
                        group.lowPosCommand(1),
                        intakeSubsystem.cubeMode(driverInputs, intakeSubsystem),
                        runDistanceWithSpeeds(-0.5, 0, 6000).withTimeout(2.9)).withTimeout(3.2),
                driveSubsystem.driveWithRotation(180, 0, 0),
                new ParallelCommandGroup(group.startingPosCommand(1),
                        runDistanceWithSpeeds(0.5, 0, -6000)).withTimeout(3.2),
                group.loadingPosCommand(1).withTimeout(AutoConstants.START_TO_LOADING_TIME),
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME));
    }

    public Command twoPieceThenEngage() {
        // for wpi, might need to change desired angle and definitely distance gone
        // Figure out what side this works on, then mirror it for the opposite color
        return new SequentialCommandGroup(
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED + .15).withTimeout(AutoConstants.EJECT_TIME),
                new ParallelCommandGroup(driveSubsystem.driveWithRotation(0, -1, 0),
                        group.lowPosCommand(1),
                        intakeSubsystem.cubeMode(driverInputs, intakeSubsystem)),
                new WaitCommand(5),
                new ParallelCommandGroup(driveSubsystem.driveWithRotation(180, 1, 0),
                        group.startingPosCommand(1),
                        intakeSubsystem.intake(0.1)).withTimeout(3),
                new WaitCommand(5),
                group.loadingPosCommand(1).withTimeout(AutoConstants.START_TO_LOADING_TIME),
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME),
                // new ParallelCommandGroup(group.startingPosCommand(1),
                // new WaitCommand(0.5).andThen(driveSubsystem.driveWithRotation(0, 0.5, .5)))
                // .withTimeout(0.5 + 1.75), // first value is the wait, second value is the
                // drive time, and maybe
                // increase Y to adjust for charge station (if hit charge station
                // side, increase Y) maybe add a forward to get further up platform
                new AutoBalance(driveSubsystem, lightsSubsystem));
    }

    public Command twoPieceWithOdometry() {
        double allySideMultiplier = DriverStation.getAlliance().compareTo(Alliance.Red) == 0 ? 1 : -1;
        return new SequentialCommandGroup(driveSubsystem.resetOdometryCommand(),
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED + .15).withTimeout(AutoConstants.EJECT_TIME),
                new ParallelRaceGroup(
                        // group.lowPosCommand(1),
                        intakeSubsystem.cubeMode(driverInputs, intakeSubsystem),
                        new AutoDrive(driveSubsystem,
                                new AutoDriveLineBuilder(-5, 0 * allySideMultiplier, 0
                                        * allySideMultiplier)
                                        .xTolerance(
                                                0.5)
                                        .useSlewAll(
                                                true)
                                        .maxXYSpeed(0.25))),

                driveSubsystem.stopCommand(),
                new PrintCommand("testing 1"),
                new WaitCommand(5),
                new PrintCommand("testing 2"),
                new ParallelRaceGroup(group.startingPosCommand(1),
                        intakeSubsystem.runMotor(0.1),
                        new AutoDrive(driveSubsystem,
                                new AutoDriveLineBuilder(0, 0 * allySideMultiplier, 180 * allySideMultiplier)
                                        .xTolerance(0.25)
                                        .useSlewAll(
                                                true)
                                        .maxXYSpeed(0.5))),
                new WaitCommand(5),
                group.loadingPosCommand(1).withTimeout(AutoConstants.START_TO_LOADING_TIME),
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME),
                group.startingPosCommand(AutoConstants.LOADING_TO_START_TIME));
    }

    public Command twoPieceThenEngageWithOdometry() {
        double allySideMultiplier = DriverStation.getAlliance().compareTo(Alliance.Red) == 0 ? 1 : -1;
        return new SequentialCommandGroup(
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED + .1).withTimeout(AutoConstants.EJECT_TIME),
                new ParallelRaceGroup(
                        group.lowPosCommand(1),
                        intakeSubsystem.intake(1),
                        new AutoDrive(driveSubsystem,
                                new AutoDriveLineBuilder(5, 0 * allySideMultiplier, 0 * allySideMultiplier))),
                new ParallelRaceGroup(group.startingPosCommand(1),
                        intakeSubsystem.intake(0.1),
                        new AutoDrive(driveSubsystem,
                                new AutoDriveLineBuilder(0, 0 * allySideMultiplier, 180 * allySideMultiplier)
                                        .xTolerance(0.05)
                                        .useSlewAll(true))),
                group.loadingPosCommand(1).withTimeout(AutoConstants.START_TO_LOADING_TIME),
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME),
                new ParallelRaceGroup(group.startingPosCommand(AutoConstants.LOADING_TO_START_TIME),
                        new AutoDrive(driveSubsystem,
                                new AutoDriveLineBuilder(1.75, -1.75 * allySideMultiplier, 180 * allySideMultiplier)
                                        .startTime(0.5)
                                        .startXAtY(-0.5 * allySideMultiplier)
                                        .holdXTillXStarts(true)
                                        .useSlewAll(true)
                                        .maxXSpeed(0.5)
                                        .usePidX(false)
                                        .xTolerance(0.1))),
                new AutoBalance(driveSubsystem, lightsSubsystem));
    }

    public Command battlecryConeHigh() {
        return new SequentialCommandGroup(driveSubsystem.addGyroOffset(180),
                group.highPosCommand(1).withTimeout(AutoConstants.START_TO_HIGH_TIME),
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME),
                group.startingPosCommand(1).withTimeout(AutoConstants.HIGH_TO_START_TIME),
                runDistanceWithSpeeds(-0.5, 0.0, 6000.0).withTimeout(2.9));
    }

    public Command battlecryConeHighThenEngage() {
        return new SequentialCommandGroup(driveSubsystem.addGyroOffset(180),
                group.highPosCommand(1).withTimeout(AutoConstants.START_TO_HIGH_TIME),
                intakeSubsystem.runMotor(AutoConstants.EJECT_SPEED).withTimeout(AutoConstants.EJECT_TIME),
                group.startingPosCommand(1).withTimeout(AutoConstants.HIGH_TO_START_TIME),
                runDistanceWithSpeeds(-0.5, 0.0, -3000.0).withTimeout(1.75),
                new AutoBalance(driveSubsystem, lightsSubsystem));
    }
}
