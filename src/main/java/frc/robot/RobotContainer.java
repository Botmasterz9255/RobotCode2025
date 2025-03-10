// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.opencv.features2d.FlannBasedMatcher;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorStateMachine;
import frc.robot.commands.SetElevatorPose;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.commands.SetWristPose;
import frc.robot.commands.ElevatorStateMachine.ElevatorState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;


public class RobotContainer {
    /* swerve drive nonsense */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

            
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);


    /* subystems */
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final WristSubsystem wrist = new WristSubsystem();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();

    /* commands */
    private final SetIntakeSpeed intakeCoral = new SetIntakeSpeed(intake, -0.90);
    private final SetIntakeSpeed outtakeCoral = new SetIntakeSpeed(intake, 0.90);
    //private final SetIntakeSpeed outtakeCoral = new SetIntakeSpeed(intake, 1.0);

    // functions
    public void setState(ElevatorState state) {
        elevatorStateMachine.setSetpoint(state);
    }

    // private final SetWristPose wristHomePose = new SetWristPose(wrist, 0);
    // private final SetWristPose wristintakePose = new SetWristPose(wrist, 16);
    //private final SetWristPose wristintakePose = new SetWristPose(wrist, 15);
    //private final SetWristPose wristLv1 = new SetWristPose(wrist, 2);
    //private final SetWristPose wristLv2 = new SetWristPose(wrist, 2);
    //private final SetWristPose wristL = new SetWristPose(wrist, 2);

    //private final SetElevatorPose elevatorHome = new SetElevatorPose(elevator, 0);
    // private final SetElevatorPose elevatorLv1 = new SetElevatorPose(elevator,Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel1);
    // private final SetElevatorPose elevatorLv2 = new SetElevatorPose(elevator,Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel2);
    // private final SetElevatorPose elevatorLv3 = new SetElevatorPose(elevator,Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel3);
    private  final ElevatorStateMachine elevatorStateMachine = new ElevatorStateMachine(elevator, wrist);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController opController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //private final SendableChooser<Command> autoChooser;
    /*
    private void namedcommands(){
        NamedCommands.registerCommand("Place Coral L1", (new InstantCommand(() -> {setState(ElevatorState.L1);})));
    }
    */

    public RobotContainer() {
        
        configureBindings();
        /*
        namedcommands();
        
        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        */
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                robotCentric.withVelocityX(driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        elevator.setDefaultCommand(elevatorStateMachine);

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        /* op controls */
        //opController.b().whileTrue(intakeCoral);
        opController.b().onTrue(new InstantCommand(() -> setState(ElevatorState.FS))
        .andThen(new InstantCommand(()->wrist.setSetPoint(14)))
        );

        opController.a().onTrue(new InstantCommand(() -> {setState(ElevatorState.L1);}));
        opController.x().onTrue(new InstantCommand(() -> {setState(ElevatorState.L2);}));
        opController.y().onTrue(new InstantCommand(() -> {setState(ElevatorState.L3);}));

        opController.leftBumper().whileTrue(intakeCoral);
        opController.rightBumper().whileTrue(outtakeCoral);

        //autos 
       
        //SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

        //autoChooser.addOption("One Piece Auto", AutoBuilder.buildAuto("One Piece Auto"));
        //autoChooser.addOption("Leave", AutoBuilder.buildAuto("Leave"));
        //SmartDashboard.putData("Auto Chooser", autoChooser);
       
    
        
    }

    public Command getAutonomousCommand() {
        double time_delay = 2; // seconds
        double speed = 3; // mps
// boolean isRed =
// DriverStation.getAlliance().isPresent()
// && DriverStation.getAlliance().get() == Alliance.Red;
// if (!isRed) {
//     speed = -3;
// }

        return 
            drivetrain.applyRequest(() -> drive.withVelocityX(speed).withVelocityY(0).withRotationalRate(0))
              .withTimeout(time_delay);
        //return Commands.print("No autonomous command configured");
        //return autoChooser.getSelected();
    }
}
