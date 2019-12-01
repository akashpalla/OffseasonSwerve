/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.ObjectInputFilter.Config;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  Trajectory trajectory;
  double startTime;
  RamseteController controller;
  DifferentialDriveKinematics kinematics;
  DifferentialDriveOdometry currDrivetrainPos;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    controller = new RamseteController(2.0, 0.7);
    kinematics = new DifferentialDriveKinematics(.5);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(10, 10);
    
    //Trajectory Watpoints
    Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d end = new Pose2d(10,3,Rotation2d.fromDegrees(15));
    Pose2d mid = new Pose2d(5, 1.5, Rotation2d.fromDegrees(0));
    
    var path = new ArrayList<Pose2d>();
    path.add(start);
    path.add(mid);
    path.add(end);

    trajectory = TrajectoryGenerator.generateTrajectory(
      path,
      trajectoryConfig
    );

    double duration = trajectory.getTotalTimeSeconds();

   
   /* var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(5, 1.5));
    double[] x = new double[]{2};
    double[] y = new double[]{0};

    ControlVectorList list = new ControlVectorList();
    list.add(new Spline.ControlVector(x, y))
    ControlVector vector = new ControlVector(x,y);
    
    var trajectory = TrajectoryGenerator.generateTrajectory(
      start,
      interiorWaypoints,
      end,
      trajectoryConfig);    
*/

     m_oi = new OI();
    // chooser.addOption("My Auto", new MyAutoCommand());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {



  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {

    currDrivetrainPos = new DifferentialDriveOdometry(kinematics, Rotation2d.fromDegrees(0));
    startTime = Timer.getFPGATimestamp();

   

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous com
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    double leftEncoder = 0;
    double rightencoder = 0;
    double gyroAngle = 0;
    currDrivetrainPos.update(Rotation2d.fromDegrees(gyroAngle), leftEncoder, rightencoder);
    double timeSinceStart = Timer.getFPGATimestamp() -startTime;
    Trajectory.State goal = trajectory.sample(timeSinceStart); // sample the trajectory at 3.4 seconds from the beginning
    ChassisSpeeds adjustedSpeeds = controller.calculate(currDrivetrainPos.getPoseMeters(), goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
    double left = wheelSpeeds.leftMetersPerSecond;
    double right = wheelSpeeds.rightMetersPerSecond;
   
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // tele
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
