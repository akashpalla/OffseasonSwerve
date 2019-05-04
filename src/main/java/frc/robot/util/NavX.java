/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


/**
 * Add your docs here.
 */
public class NavX {

    AHRS navX;
    public NavX(){
        navX = new AHRS(SPI.Port.kMXP);
        navX.zeroYaw();
    }

    public void zeroYaw(){
        navX.zeroYaw();
    }

    public double getGyroAngle(){
        return navX.getYaw();
    }
}
