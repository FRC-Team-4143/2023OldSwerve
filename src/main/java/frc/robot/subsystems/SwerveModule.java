// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.AnalogEncoder;
// import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.Preferences;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveModule {
  private final TalonSRX m_driveMotor;
  private final TalonSRX m_turningMotor;
  private int turningChannel;
  private double turningMotorOffset;
  

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      double turnOffset,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    m_driveMotor = new TalonSRX(driveMotorChannel);
    m_turningMotor = new TalonSRX(turningMotorChannel);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        turningChannel = turningMotorChannel;
        turningMotorOffset = Preferences.getDouble("Module"+turningChannel, turnOffset);
        
   // m_driveEncoder = new Encoder(driveEncoderChannels[0], driveEncoderChannels[1]);

   // m_turningEncoder = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    //m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    //m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.  - Math.PI + turningMotorOffset
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }
  public double getTurningMotorPosition(){
    double sensorPos = m_turningMotor.getSelectedSensorPosition()/4096.0*360.0*Math.PI/180.0 - Math.PI - turningMotorOffset;
    sensorPos *= -1.;
     if (sensorPos < -Math.PI){
      return sensorPos += 2*Math.PI;
     }
     else if (sensorPos > Math.PI) {
       return sensorPos -= 2*Math.PI;
     }
      return sensorPos;

  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  //m_turningMotor.getSelectedSensorPosition()/360*2*Math.PI
  public SwerveModuleState getState() {
  
    return new SwerveModuleState(
      m_driveMotor.getMotorOutputPercent() * 10, new Rotation2d(getTurningMotorPosition()));
     }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double position = getTurningMotorPosition();
    SmartDashboard.putNumber(turningChannel + " Motor ", position);
    return new SwerveModulePosition(
        m_driveMotor.getMotorOutputPercent() * 10, new Rotation2d(position));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */

   public void setWheelOffsets() {
      turningMotorOffset = m_turningMotor.getSelectedSensorPosition()/4096.0*360.0*Math.PI/180.0 - Math.PI;
      Preferences.setDouble("Module"+turningChannel, turningMotorOffset);

   }
  public void setDesiredState(SwerveModuleState desiredState) {
    SmartDashboard.putNumber(turningChannel + "Current angle", getTurningMotorPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningMotorPosition()));

    // Calculate the drive output from the drive PID controller.
    double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getMotorOutputPercent(), state.speedMetersPerSecond);

        driveOutput = state.speedMetersPerSecond /  DriveConstants.kMaxSpeedMetersPerSecond;

    // Calculate the turning motor output from the turning PID controller.
    //(turningMotorOffset - state.angle.getRadians()) calculates the delta to get the desired angle to the current angle
    double turnOutput =
        m_turningPIDController.calculate(getTurningMotorPosition(), state.angle.getRadians());
        // System.out.println(turningChannel + " current angle "+ getTurningMotorPosition());
        // System.out.println(turningChannel + " desired angle " + state.angle.getRadians());
    // Calculate the turning motor output from the turning PID controller.

    turnOutput = Math.min(Math.max(-.7, turnOutput), .7);
    
    m_driveMotor.set(TalonSRXControlMode.PercentOutput,driveOutput);
    m_turningMotor.set(TalonSRXControlMode.PercentOutput,turnOutput);
    SmartDashboard.putNumber(turningChannel + "Current turning output", turnOutput);
    SmartDashboard.putNumber(turningChannel + " Current drive output", driveOutput);
  }
  
 }
