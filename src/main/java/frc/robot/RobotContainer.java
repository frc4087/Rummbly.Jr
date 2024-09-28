// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import au.grapplerobotics.LaserCan;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrankenArm;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotContainer {
  
  public LaserCan LaunchSensor = new LaserCan(TunerConstants.LaunchSensor);
  public LaserCan IntakeSensor = new LaserCan(TunerConstants.IntakeSensor);
  public TalonFX IntakeFeedMotor = new TalonFX(TunerConstants.IntakeFeed);
  public TalonFX IntakeCenterMotor = new TalonFX(TunerConstants.IntakeCenter);
  public TalonFX LauncherFeedMotor = new TalonFX(TunerConstants.LaunchFeed);
  public TalonFX LaunchRtFlywheel = new TalonFX(TunerConstants.LaunchRtFlywheel);
  public TalonFX LaunchLtFlywheel = new TalonFX(TunerConstants.LaunchLtFlywheel);
  public final CommandXboxController m_joystick = new CommandXboxController(1);
  //private final BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem(0);

  Trigger xButton = m_joystick.x();
  Trigger yButton = m_joystick.y();
  Trigger aButton = m_joystick.a();
  Trigger bButton = m_joystick.b();
  Trigger bBumpTrigger = m_joystick.rightBumper();
  Trigger aBumpTrigger = m_joystick.leftBumper();
  Trigger cTrigger = m_joystick.rightTrigger();

  private static final double MAX_LAUNCH_DISTANCE_MM = 10.0;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final FrankenArm frankenArm = new FrankenArm();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(/*-*/joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(/*-*/joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(/*-*/joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(/*-*/joystick.getLeftY(), /*-*/joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // m_joystick.a().onTrue(new RunCommand(() -> {
    //   frankenArm.setArmPosition(FrankenArm.SETPOINTIntake);
    //   frankenArm.runIntake();
    // }, frankenArm));


    m_joystick.rightBumper().onTrue(new RunCommand(() -> {
      frankenArm.setArmPosition(FrankenArm.SETPOINTIntake);
      frankenArm.runIntake();
    }, frankenArm).onlyWhile(frankenArm.limit::get).finallyDo(interrupted -> {
      IntakeFeedMotor.set(0);
      IntakeCenterMotor.set(0);
      LauncherFeedMotor.set(0);
      //frankenArm.CoastMode();
      System.out.println("Object detected, stopping intake motors");
  }));

    m_joystick.a().onTrue(new RunCommand(() -> frankenArm.setArmPosition(FrankenArm.SETPOINTNear), frankenArm));

    m_joystick.leftBumper().onTrue(new RunCommand(() -> {
      //frankenArm.setArmPosition(FrankenArm.SETPOINTAmp);
      frankenArm.runAmp();
    }, frankenArm));

    m_joystick.x().onTrue(new RunCommand(() -> {
      frankenArm.setArmPosition(FrankenArm.SETPOINTClose);
    }, frankenArm));

    m_joystick.y().onTrue(new RunCommand(() -> {
      frankenArm.setArmPosition(FrankenArm.SETPOINTAmp);
    }, frankenArm));

    m_joystick.b().onTrue(new RunCommand(() -> frankenArm.setArmPosition(FrankenArm.SETPOINTFar), frankenArm));

    m_joystick.rightTrigger().onTrue(new RunCommand(() -> {
      //frankenArm.setArmPosition(FrankenArm.SETPOINTIntake);
      frankenArm.runLauncher();
    }, frankenArm));

  }

  public Command getAutonomousCommand() {
    // Define and return your autonomous command here
    return null;
  }
}
