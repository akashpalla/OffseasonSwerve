/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Wheel  {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX drive;
  public TalonSRX steer;

  public Wheel(TalonSRX _drive, TalonSRX _steer, int driveID, int steerID){
    drive = _drive;
    steer = _steer;
  
    drive = new TalonSRX(driveID);
    steer = new TalonSRX(steerID);

    drive.configFactoryDefault();
    steer.configFactoryDefault();
    
    steer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    steer.setSensorPhase(false);
		steer.setInverted(false);

    steer.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		steer.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    steer.configNominalOutputForward(0, Constants.kTimeoutMs);
		steer.configNominalOutputReverse(0, Constants.kTimeoutMs);
		steer.configPeakOutputForward(0.7, Constants.kTimeoutMs);
    steer.configPeakOutputReverse(-0.7, Constants.kTimeoutMs);
    steer.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		steer.config_kF(Constants.kSlotIdx, Constants.kSteerF, Constants.kTimeoutMs);
		steer.config_kP(Constants.kSlotIdx, Constants.kSteerP, Constants.kTimeoutMs);
		steer.config_kI(Constants.kSlotIdx, Constants.kSteerI, Constants.kTimeoutMs);
		steer.config_kD(Constants.kSlotIdx, Constants.kSteerD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		steer.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		steer.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* Zero the sensor */
		steer.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  /** 
   * throttle: -1 to 1
   * strafe -1 to 1
   * rotate -0.5 to 0.5
  */
  public void swerveDrive(double throttle, double rotate){
    rotate *= Constants.kTICKS;
    double currPos = steer.getSelectedSensorPosition(0);
    double steerError = Math.IEEEremainder(rotate - currPos, Constants.kTICKS);

    boolean isInverted = Math.abs(steerError) > 0.25 * Constants.kTICKS;
 
    if (isInverted) {
      steerError -= Math.copySign(0.5 * Constants.kTICKS, steerError);
      throttle = -throttle;
    }

    drive.set(ControlMode.PercentOutput, throttle);
    steer.set(ControlMode.MotionMagic, currPos + steerError);
  }
}
