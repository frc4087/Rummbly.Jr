// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.PhysicsSim;
// import au.grapplerobotics.LaserCan;
// import frc.robot.Robot;

// /**
//  * The VM is configured to automatically run this class, and to call the functions corresponding to
//  * each mode, as described in the TimedRobot documentation. If you change the name of this class or
//  * the package after creating this project, you must also update the build.gradle file in the
//  * project.
//  */

// public class FrankenArm extends SubsystemBase {
  
//   public LaserCan lc;
//   public LaserCan lc2;
//   public TalonFX IntakeFeedMotor = new TalonFX(TunerConstants.IntakeFeed);
//   public TalonFX IntakeCenterMotor = new TalonFX(TunerConstants.IntakeCenter);
//   public TalonFX LauncherFeedMotor = new TalonFX(TunerConstants.LaunchFeed);
//   public TalonFX LaunchRtFlywheel = new TalonFX(TunerConstants.LaunchRtFlywheel);
//   public TalonFX LaunchLtFlywheel = new TalonFX(TunerConstants.LaunchLtFlywheel);
 
//   double setpoint1 = 05;
//   double setpoint2 = 30;
//   double setpoint3 = 60;
//   double fullSpeedAhead = 1;
//   double Die = 0;
//   double warmup = 0.5;  

//     public void initDefaultCommand() {
//     if (m_joystick.getRawButton(8)) {
//       m_fx.setControl(m_mmReq.withPosition(setpoint1));
//       LaunchRtFlywheel.set(warmup);
//       LaunchLtFlywheel.set(warmup);
//       }
      

//     if (m_joystick.getRawButton(1)) {
//       m_fx.setControl(m_mmReq.withPosition(setpoint1));
//       IntakeFeedMotor.set(fullSpeedAhead);
//       IntakeCenterMotor.set(fullSpeedAhead);
//       LauncherFeedMotor.set(fullSpeedAhead);
//       shootOff();
//       //checkSet();
//       intakeOff();
//       }

//     if (m_joystick.getRawButton(2)) {
//       m_fx.setControl(m_mmReq.withPosition(setpoint2));
//       }

//     if (m_joystick.getRawButton(3)) {
//       m_fx.setControl(m_mmReq.withPosition(setpoint3));
//       }

//       if (m_joystick.getRawButton(4)) {
//       m_fx.setControl(m_mmReq.withPosition(setpoint1));
//       LaunchRtFlywheel.set(fullSpeedAhead);
//       LaunchLtFlywheel.set(fullSpeedAhead);
//       LauncherFeedMotor.set(fullSpeedAhead);
//       }
//     }

//     public void shootOff(){
//     LaserCan.Measurement measurement = lc.getMeasurement();
//     if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 100) {
//       IntakeFeedMotor.set(Die);
//       IntakeCenterMotor.set(Die);
//       LauncherFeedMotor.set(Die);
//     }
//   }
  
//   public void intakeOff(){
//     LaserCan.Measurement measurement = lc2.getMeasurement();
//     if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 100) {
//       LauncherFeedMotor.set(Die);
//     }
//   }

//   public void checkSet(){
//     var k_tolerance.setTolerance(5, 10);
//     if(MathUtil.isNear(m_ex.measurement, setpoint1, k_tolerance)) {
//       LauncherFeedMotor.set(0.5);
//     }
//   }
//}

//public class FrankenArm extends TimedRobot {
  private final TalonFX m_fx = new TalonFX(1, "canivore");
  public final CANcoder m_ex = new CANcoder(1, "canivore");
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final XboxController m_joystick = new XboxController(1);

  private int m_printCount = 0;

  private final Mechanisms m_mechanisms = new Mechanisms();

  @Override
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(m_fx, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 5; // 5 (mechanism) rotations per second cruise
    mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.1 seconds to reach max accel 
    mm.MotionMagicJerk = 100;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  @Override
  public void robotPeriodic() {
    if (m_printCount++ > 10) {
      m_printCount = 0;
      System.out.println("Pos: " + m_fx.getPosition());
      System.out.println("Vel: " + m_fx.getVelocity());
      System.out.println();
    }
    m_mechanisms.update(m_fx.getPosition(), m_fx.getVelocity());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}