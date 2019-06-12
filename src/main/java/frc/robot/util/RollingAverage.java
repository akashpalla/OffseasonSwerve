/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.ArrayList;

/**
 * Add your docs here.
 */
public class RollingAverage {

    private int size;
    private double sum;
    private double [] arr;

    public RollingAverage(int _size){
        size = _size;
        arr = new double[size];
        sum = 0;
    }

    public void add(double x){
        sum -= arr[0];
        for(int i = 0; i < size-1; i++){
            arr[i] = arr[i+1];
        }
        arr[size-1] = x;
        sum+=x;
    }

    public double getAverage(){
        return sum/size;
    }
}
