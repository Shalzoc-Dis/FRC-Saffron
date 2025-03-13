// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralArmSubsystem extends SubsystemBase {

  private final SparkMax m_extendor;
  private final SparkMaxConfig m_extendorConfig;
  private final SparkClosedLoopController m_extendorController;
  private final RelativeEncoder m_extendorEncoder;


  private final SparkMax m_shoulder;
  private final SparkMaxConfig m_shoulderConfig;
  private final SparkClosedLoopController m_shoulderController;
  private final RelativeEncoder m_shoulderEncoder;

  /** Creates a new CoralArmSubsystem. */
  public CoralArmSubsystem() {
    m_extendor = new SparkMax(0, MotorType.kBrushless);
    m_extendorController = m_extendor.getClosedLoopController();
    m_extendorEncoder = m_extendor.getEncoder();

    m_extendorConfig = new SparkMaxConfig();
    

    m_shoulder = new SparkMax(0, MotorType.kBrushless);
    m_shoulderController = m_shoulder.getClosedLoopController();
    m_shoulderEncoder = m_shoulder.getEncoder();
    m_shoulderConfig = new SparkMaxConfig();

    configureMotors();
  }

  /** Positive is out, negative is in reverse
   * @param length [-1, 1]. The sign is the direction, the magnitude is the speed
   */
  public void moveArmInDirection(double length) {

  }

  /** Positive is towards the front of the robot. 
   * @param angle [-1, 1]. The sign determins the direction, the magnitude the speed
   */
  public void moveShoulderInDirection(double angle) {

  }

  /** Extend/Retract the arm to be at the specified position.
   * @param position This is the position the arm should move to. 0 is retracted, Constants.CoralScoring.maxExtension is all the way out.
   */
  public void moveArmToPosition(double position) {

  }

  /** Move the arm to be at the specified position.
   * @param position This is the position the arm should move to. 0 is down, Constants.CoralScoring.maxShoulderRotation is all the way up.
   */
  public void moveShoulderToPosition(double position) {

  }

  private void configureMotors() {
    // Extendor
    m_extendorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    m_extendorConfig.closedLoop
      // Set the feedback sensor as the primary encoder
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Position PID constants. Set to slot 0 be default
      .p(Constants.CoralScoring.extendorPositionPID[0])
      .i(Constants.CoralScoring.extendorPositionPID[1])
      .d(Constants.CoralScoring.extendorPositionPID[2])
      .outputRange(-1, 1)
      // Velocity PID constants. Set to slot 1
      .p(Constants.CoralScoring.extendorVelocityPID[0], ClosedLoopSlot.kSlot1)
      .i(Constants.CoralScoring.extendorVelocityPID[1], ClosedLoopSlot.kSlot1)
      .d(Constants.CoralScoring.extendorVelocityPID[2], ClosedLoopSlot.kSlot1)
      .velocityFF(Constants.CoralScoring.extendorVelocityPID[3], ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1);

    m_extendor.configure(m_extendorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Shoulder
    m_shoulderConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    m_shoulderConfig.closedLoop
      // Set the feedback sensor as the primary encoder
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Position PID constants. Set to slot 0 be default
      .p(Constants.CoralScoring.shoulderPositionPID[0])
      .i(Constants.CoralScoring.shoulderPositionPID[1])
      .d(Constants.CoralScoring.shoulderPositionPID[2])
      .outputRange(-1, 1)
      // Velocity PID constants. Set to slot 1
      .p(Constants.CoralScoring.shoulderVelocityPID[0], ClosedLoopSlot.kSlot1)
      .i(Constants.CoralScoring.shoulderVelocityPID[1], ClosedLoopSlot.kSlot1)
      .d(Constants.CoralScoring.shoulderVelocityPID[2], ClosedLoopSlot.kSlot1)
      .velocityFF(Constants.CoralScoring.shoulderVelocityPID[3], ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1);

    m_shoulder.configure(m_extendorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}