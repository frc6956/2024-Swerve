// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  double tv;
  double tx;
  double ty;
  double ta;
  double ts;
  double tl;
  double cl;
  double getpipe;

  DoubleArraySubscriber pSub;
  double[] botpose;
  double[] botpose_wpiblue;
  double[] botpose_wpired;
  double[] camerapose_targetspace;
  double[] targetpose_cameraspace;
  double[] targetpose_robotspace;
  double[] botpose_targetspace;
  double tid;

  public Vision() {
    pSub = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
    targetpose_robotspace = new double[6];
  }

  @Override
  public void periodic() {
  }

  public void updateTables(){
    table = NetworkTableInstance.getDefault().getTable("limelight");
    targetpose_robotspace = pSub.get(new double[6]);
    tv = table.getEntry("tv").getDouble(0);
    tid = table.getEntry("tid").getDouble(0);
  }

  public boolean hasValidTarget(){
    updateTables();
    boolean result = (tv != 0);
    return result;
  }

  public double getTagID(){
    updateTables();
    if (hasValidTarget()){
      return tid;
    }
    return 0;
  }

  public double getRX(){
    updateTables();
    return targetpose_robotspace[0];
  }

  public double getRY(){
    updateTables();
    return targetpose_robotspace[1];
  }

  public double getRZ(){
    updateTables();
    return targetpose_robotspace[2];
  }

  public double getPitch(){
    updateTables();
    return targetpose_robotspace[3];
  }

  public double getHeading(){
    updateTables();
    return targetpose_robotspace[4];
  }

  public double getRoll(){
    updateTables();
    return targetpose_robotspace[5];
  }
}
