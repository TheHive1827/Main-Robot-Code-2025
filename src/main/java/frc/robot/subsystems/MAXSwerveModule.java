// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
// import swervelib.parser.SwerveParser;
// import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.*;
import com.ctre.phoenix6.hardware.CANcoder;
// import swervelib.encoders.CANCoderSwerve;
import frc.robot.Configs;


public class MAXSwerveModule {
  private final SparkMax m_drivingMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;
 
  private final CANcoder m_absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, 
  int AbsoluteCanID, double absoluteEncoderOffset, boolean absoluteEncoderReversed,double chassisAngularOffset) {
    m_drivingMotor = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningCANId, MotorType.kBrushless);

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    m_absoluteEncoder = new CANcoder(AbsoluteCanID, "else");
  

    m_drivingEncoder = m_drivingMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingMotor.getClosedLoopController();
    m_turningClosedLoopController = m_turningMotor.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingMotor.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public double getDrivePosition(){
    return m_drivingEncoder.getPosition();
  }

  public double getTurningPosition(){
    return m_turningEncoder.getPosition();
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public double getAbsoluteEncoderRad(){
    double angle = m_turningEncoder.getPosition();
    angle = 2 * Math.PI / 4096;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;

    System.out.println("Desired State: " + desiredState);
    System.out.println("Corrected Desired State: " + correctedDesiredState);
    System.out.println("Driving Encoder Position: " + m_drivingEncoder.getPosition());
    System.out.println("Turning Encoder Position: " + m_turningEncoder.getPosition());
  }
  
  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public void stop(){
    m_drivingMotor.set(0);
    m_turningMotor.set(0);
  }
}
