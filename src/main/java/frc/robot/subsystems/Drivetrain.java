/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.util.Limelight;

/**
 * Add your docs here.
 */ 
public class Drivetrain extends Subsystem {
 
  private final Wheel[] wheels;
  private final double[] ws;
  private final double[] wa;

  private double kLengthComponent;
  private double kWidthComponent;

  private Limelight limelight;

  Wheel frontLeft, frontRight, backLeft, backRight;

  public Drivetrain(){

    limelight = new Limelight();

    frontLeft = new Wheel(Hardware.driveFrontLeft, Hardware.steerFrontLeft, Constants.kDriveFrontLeft, Constants.kSteerFrontLeft);
    frontRight = new Wheel(Hardware.driveFrontRight, Hardware.steerFrontRight, Constants.kDriveFrontRight, Constants.kSteerFrontRight);
    backLeft = new Wheel(Hardware.driveBackLeft, Hardware.steerBackLeft, Constants.kDriveBackLeft, Constants.kSteerBackLeft);
    backRight = new Wheel(Hardware.driveBackRight, Hardware.steerBackRight, Constants.kDriveBackRight, Constants.kSteerBackRight);
    wheels = new Wheel[]{frontLeft, frontRight, backLeft, backRight};
  
    double length = Constants.kDrivebaselength;
    double width = Constants.kDrivebaseWidth;
    double radius = Math.hypot(length, width);
    kLengthComponent = length / radius;
    kWidthComponent = width / radius;

    wa = new double [4];
    ws = new double [4];

  }

  // Throttle: -1 to 1, Strafe: -1 to 1, rotate -1 to 1
  public void swerveDrive(double throttle, double strafe, double rotate){
    double a = strafe - rotate * kLengthComponent;
    double b = strafe + rotate * kLengthComponent;
    double c = throttle - rotate * kWidthComponent;
    double d = throttle + rotate * kWidthComponent;

    // wheel speed
    ws[0] = Math.hypot(b, d);
    ws[1] = Math.hypot(b, c);
    ws[2] = Math.hypot(a, d);
    ws[3] = Math.hypot(a, c);

    // wheel Rotate
    wa[0] = Math.atan2(b, d) * 0.5 / Math.PI;
    wa[1] = Math.atan2(b, c) * 0.5 / Math.PI;
    wa[2] = Math.atan2(a, d) * 0.5 / Math.PI;
    wa[3] = Math.atan2(a, c) * 0.5 / Math.PI;

    // normalize wheel speed
    final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
    if (maxWheelSpeed > 1.0) {
      for (int i = 0; i < 4; i++) {
        ws[i] /= maxWheelSpeed;
      }
    }

    frontLeft.swerveDrive(ws[0], wa[0]);
    frontRight.swerveDrive(ws[1], wa[1]);
    backLeft.swerveDrive(ws[2], wa[2]);
    backRight.swerveDrive(ws[3], wa[3]);
  }


  public void alignWithTarget(){
    
    if(limelight.hasTarget())
    {
      double angle = limelight.getHorizontalOffset();
      double throttle = (Constants.kMaxTargetArea - limelight.getArea());

      double leftOutput = throttle * Constants.kVisionDriveP + angle* Constants.kVisionTurnP;
      double rightOutput = throttle * Constants.kVisionDriveP - angle* Constants.kVisionTurnP;

      //drive(leftOutput, rightOutput)    
    }
  }


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoystick());
  }
}
