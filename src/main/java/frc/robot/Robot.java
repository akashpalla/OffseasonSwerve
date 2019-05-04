/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Drivetrain;


public class Robot extends TimedRobot {
  public static OI oi;
  public static Drivetrain drivetrain;
  
  public void robotInit() {
    drivetrain = new Drivetrain();
    oi = new OI();
  }

  public void robotPeriodic() {
 
  }

  public void disabledInit() {
  
  }

  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  public void autonomousInit() {
    
  }

  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  public void teleopInit() {
  
  }

 public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  public void testPeriodic() {
  }
}
