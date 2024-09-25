// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.TunerConstants;

import au.grapplerobotics.LaserCan;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class FrankenArm extends SubsystemBase {
  
  public LaserCan LaunchSensor = new LaserCan(TunerConstants.LaunchSensor);
  public LaserCan IntakeSensor = new LaserCan(TunerConstants.IntakeSensor);
  public TalonFX IntakeFeedMotor = new TalonFX(TunerConstants.IntakeFeed);
  public TalonFX IntakeCenterMotor = new TalonFX(TunerConstants.IntakeCenter);
  public TalonFX LauncherFeedMotor = new TalonFX(TunerConstants.LaunchFeed);
  public TalonFX LaunchRtFlywheel = new TalonFX(TunerConstants.LaunchRtFlywheel);
  public TalonFX LaunchLtFlywheel = new TalonFX(TunerConstants.LaunchLtFlywheel);
  public final TalonFX armMotor = new TalonFX(TunerConstants.ArmAngle);
  public final CANcoder armSensor = new CANcoder(TunerConstants.ArmSensor);
  public final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  public static final double TOLERANCE = 5.0;
  public static final double SETPOINTIntake = 4.0;
  public static final double SETPOINTFar = 2.9;
  public static final double SETPOINTNear = 42.436;  
  public static final double SETPOINTAmp = 116.467;
  private static final double MAX_INTAKE_DISTANCE_MM = 200.0;
  private static final double HYSTERESIS_MM = 10.0;
  private static final double MAX_LAUNCH_DISTANCE_MM = 15.0;
  private static final double HYSTERESIS2_MM = 10.0;


  private boolean isObjectDetected = false;
  public boolean isChecking = false;
  private boolean isObject2Detected = false;
  private boolean isIntakeConfigured = false;

  public FrankenArm() {
    
    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    
    // Create MotionMagicConfigs object
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 15000;
    motionMagicConfigs.MotionMagicAcceleration = 6000;

    // Apply MotionMagicConfigs to TalonFXConfiguration
    armMotorConfig.MotionMagic = motionMagicConfigs;

    // Create Slot0Configs object for PID configurations
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.3;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0001;

    // Apply Slot0Configs to TalonFXConfiguration
    armMotorConfig.Slot0 = slot0Configs;

    StatusCode status = armMotor.getConfigurator().apply(armMotorConfig);

    CANcoderConfiguration armSensorConfig = new CANcoderConfiguration();
    status = armSensor.getConfigurator().apply(armSensorConfig);

    // Set initial positions or states
    armSensor.setPosition(0.0); // Set the position of the CANcoder sensor

    // Set neutral mode
    armMotor.setNeutralMode(NeutralModeValue.Brake);

    initializeArm();
  }

  private void initializeArm() {
    // Initial calibration
    setArmPosition(SETPOINTIntake); // Move to a known position
}

public void setArmPosition(double position) {
    armMotor.setControl(motionMagic.withPosition(position));
  }

// public void runIntake() {
//     IntakeFeedMotor.getConfigurator().apply(new TalonFXConfiguration());
//     IntakeFeedMotor.setInverted(false);
//     IntakeFeedMotor.set(0.6);

//     IntakeCenterMotor.set(0.6);
//     // shootOffIntake();
//     // startChecking();
//     // shootOffLaucher();
// } 

public void runLauncher() {
  LaunchRtFlywheel.set(1);
  LaunchLtFlywheel.set(1);
  LauncherFeedMotor.set(1);
}

public void runIntake() {
  IntakeFeedMotor.set(0.5);
  IntakeCenterMotor.set(0.5);
  LauncherFeedMotor.set(0.35);
  //shootOffIntake();
  // startChecking();
  shootOffLauncher();
}

// public void shootOffIntake() {
//   LaserCan.Measurement measurement = IntakeSensor.getMeasurement();
//   if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 200.0) {
//     IntakeFeedMotor.set(0);
//     IntakeCenterMotor.set(0);
//     LauncherFeedMotor.set(0);
//   }
//   }

  // public void shootOffIntake() {
  //   LaserCan.Measurement measurement = IntakeSensor.getMeasurement();

  //   if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
  //       if (!isObjectDetected && measurement.distance_mm <= MAX_INTAKE_DISTANCE_MM) {
  //           isObjectDetected = true;
  //           IntakeFeedMotor.set(0);
  //           IntakeCenterMotor.set(0);
  //           System.out.println("Object detected, stopping intake motors");
  //       } else if (isObjectDetected && measurement.distance_mm > MAX_INTAKE_DISTANCE_MM + HYSTERESIS_MM) {
  //           isObjectDetected = false;
  //           System.out.println("Object no longer detected, motors can be restarted");
  //       }
  //   } else {
  //       System.out.println("Invalid measurement from LaserCAN");
  //   }
  // }

public void shootOffLauncher() {
    LaserCan.Measurement measurement = LaunchSensor.getMeasurement();

    if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        if (!isObject2Detected && measurement.distance_mm <= MAX_LAUNCH_DISTANCE_MM) {
            isObject2Detected = true;
            IntakeFeedMotor.set(0);
            IntakeCenterMotor.set(0);
            LauncherFeedMotor.set(0);
            System.out.println("Object detected, stopping intake motors");
        } else if (isObject2Detected && measurement.distance_mm > MAX_LAUNCH_DISTANCE_MM + HYSTERESIS2_MM) {
            isObject2Detected = false;
            System.out.println("Object no longer detected, motors can be restarted");
        }
    } else {
        System.out.println("Invalid measurement from LaserCAN");
    }
}

// private void configureLaserCAN() {
//   LaunchSensor.setRangingMode(LaserCan.RangingMode.SHORT);
//   LaunchSensor.setRegionOfInterest(0, 0, 16, 16);
//   LaunchSensor.setTimingBudget(33000);
//   LaunchSensor.setRangeContinuous();

//   while (!IntakeSensor.isRangeValid()) {
//       Timer.delay(0.01);
//   }

//   System.out.println("LaserCAN configuration complete");
// }

// public void shootOffLaucher() {
//   LaserCan.Measurement measurement = LaunchSensor.getMeasurement();
//   if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 2.0) {
//     IntakeFeedMotor.set(0);
//     IntakeCenterMotor.set(0);
//     LauncherFeedMotor.set(0);
//   }
//   }

  // public void startChecking() {
  //   if (!isChecking) {
  //       isChecking = true;
  //       new Thread(() -> {
  //           while (isChecking) {
  //               double currentPosition = armSensor.getPosition().getValue(); // Extract the value from StatusSignal
  //               if (Math.abs(currentPosition - SETPOINTIntake) <= TOLERANCE) {
  //                   LauncherFeedMotor.set(0.6);
  //                   IntakeCenterMotor.set(0.6);
  //                   isChec
  // king = false; // Stop checking once within tolerance
  //               } else {
  //                   LauncherFeedMotor.set(0);
  //                   IntakeCenterMotor.set(0);
  //               }
  //               try {
  //                   Thread.sleep(100); // Sleep for a short period to avoid busy-waiting
  //               } catch (InterruptedException e) {
  //                   Thread.currentThread().interrupt();
  //               }
  //           }
  //       }).start();
  //   }
//}
}

// public static final double FULL_SPEED = 0.65;
// public static final double STOP = 0;
// public static final double WARMUP = 0.5;

// public final TalonFX m_fx = new TalonFX(TunerConstants.ArmAngle);
  // public final CANcoder m_ex = new CANcoder(TunerConstants.ArmSensor);
  //public final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  // public final double k_tolerance = 3.0;
  // public boolean isChecking = false;
  // public CommandXboxController m_joystick = new CommandXboxController(RobotContainer.m_joystick);
  // double setpoint1 = 05;
  // double setpoint2 = 10;
  // double setpoint3 = 20;
  // double fullSpeedAhead = 0.65;
  // double Die = 0;
  // double warmup = 0.5;

//   public void shootOff(){
  //   LaserCan.Measurement measurement = IntakeSensor.getMeasurement();
  //   if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 100) {
  //     IntakeFeedMotor.set(Die);
  //     IntakeCenterMotor.set(Die);
  //     LauncherFeedMotor.set(Die);
  //   }
  // }
  
  // public void intakeOff(){
  //   LaserCan.Measurement measurement = LaunchSensor.getMeasurement();
  //   if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 300) {
  //     LauncherFeedMotor.set(Die);
  //   }
  // }

// public void checkSet() {
//     double currentPosition = m_ex.getPosition().getValue(); // Extract the value from StatusSignal
//     if (Math.abs(currentPosition - setpoint1) <= k_tolerance) {
//         LauncherFeedMotor.set(0.5);
//     } else {
//         LauncherFeedMotor.set(0);
//     }
//}

// public void startChecking() {
//     if (!isChecking) {
//         isChecking = true;
//         new Thread(() -> {
//             while (isChecking) {
//                 double currentPosition = m_ex.getPosition().getValue(); // Extract the value from StatusSignal
//                 if (Math.abs(currentPosition - setpoint1) <= k_tolerance) {
//                     LauncherFeedMotor.set(warmup);
//                     isChecking = false; // Stop checking once within tolerance
//                 } else {
//                     LauncherFeedMotor.set(fullSpeedAhead);
//                 }
//                 try {
//                     Thread.sleep(100); // Sleep for a short period to avoid busy-waiting
//                 } catch (InterruptedException e) {
//                     Thread.currentThread().interrupt();
//                 }
//             }
//         }).start();
//     }
// }
//}

//public void initDefaultCommand() {
      
    //   m_joystick.a().onTrue(new RunCommand(() -> {
    //   //m_fx.setControl(m_mmReq.withPosition(setpoint1));
    //   IntakeFeedMotor.setInverted(true);
    //   IntakeFeedMotor.set(1);
    //   IntakeCenterMotor.set(1);
    //   LauncherFeedMotor.set(1);
    //   //shootOff();
    //   //startChecking();
    //   //intakeOff();
    // }, this));

    // m_joystick.b().onTrue(new RunCommand(() -> {
    //   m_fx.setControl(m_mmReq.withPosition(setpoint2));
    // }, this));

    // m_joystick.x().onTrue(new RunCommand(() -> {
    //   m_fx.setControl(m_mmReq.withPosition(setpoint3));
    // }, this));

    // m_joystick.y().onTrue(new RunCommand(() -> {
      
    //   m_fx.setControl(m_mmReq.withPosition(setpoint1));
    //   LaunchRtFlywheel.set(1);
    //   LaunchLtFlywheel.set(1);
    //   LauncherFeedMotor.set(1);
    // }, this));