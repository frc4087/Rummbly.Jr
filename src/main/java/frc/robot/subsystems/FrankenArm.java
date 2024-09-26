// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.Queue;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;

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
  DigitalInput limit = new DigitalInput(0);
  

  public static final double TOLERANCE = 5.0;
  public static final double SETPOINTIntake = 4.0;
  public static final double SETPOINTFar = 2.9;
  public static final double SETPOINTNear = 42.436;  
  public static final double SETPOINTAmp = 116.467;
  private static final double MAX_LAUNCH_DISTANCE_MM = 10.0;
  private static final double HYSTERESIS2_MM = 10.0;

  public boolean isChecking = false;
  
  private boolean isObject2Detected = false;

  public FrankenArm() {
    final DigitalInput beamBreakSensor;
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

    CoastMode();

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

public void runLauncher() {
  LaunchRtFlywheel.set(1);
  LaunchLtFlywheel.set(1);
  LauncherFeedMotor.set(1);
}

public void runIntake() {
  IntakeFeedMotor.set(0.5);
  IntakeCenterMotor.set(0.5);
  LauncherFeedMotor.set(0.4);
  //shootOffLauncher();
  Breaky();
}

public void shootOffLauncher() {
    LaserCan.Measurement measurement = LaunchSensor.getMeasurement();

    if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        if (!isObject2Detected && measurement.distance_mm <= MAX_LAUNCH_DISTANCE_MM) {
            isObject2Detected = true;
            IntakeFeedMotor.set(0);
            IntakeCenterMotor.set(0);
            LauncherFeedMotor.set(0);
            CoastMode();
            System.out.println("Object detected, stopping intake motors");
        } else if (isObject2Detected && measurement.distance_mm > MAX_LAUNCH_DISTANCE_MM + HYSTERESIS2_MM) {
            isObject2Detected = false;
            System.out.println("Object no longer detected, motors can be restarted");
        }
    } else {
        System.out.println("Invalid measurement from LaserCAN");
    }
}

public void CoastMode(){
IntakeFeedMotor.setNeutralMode(NeutralModeValue.Brake);
IntakeCenterMotor.setNeutralMode(NeutralModeValue.Brake);
LauncherFeedMotor.setNeutralMode(NeutralModeValue.Brake);
}

public void Breaky(){
if(!limit.get()) {
  System.out.println("Beam holding strong!");;
} else {
  IntakeFeedMotor.set(0);
  IntakeCenterMotor.set(0);
  LauncherFeedMotor.set(0);
  //CoastMode();
}
}

public void checkBreakBeam() {
  if (limit.get()) {
      System.out.println("Break beam tripped! Stopping motors.");
      IntakeFeedMotor.set(0);
      IntakeCenterMotor.set(0);
      LauncherFeedMotor.set(0);
      //CoastMode();
  } else {
      System.out.println("Break beam intact. Motors can run.");
  }
}

}


