// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  NetworkTable table;

  double tv;
  double tx;
  double ty;
  double ta;
  double ts;
  double tl;
  double cl;
  double getpipe;

  double[] botpose;
  double[] botpose_wpiblue;
  double[] botpose_wpired;
  double[] camerapose_targetspace;
  double[] targetpose_cameraspace;
  double[] targetpose_robotspace;
  double[] botpose_targetspace;
  double tid;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    updateTables();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTables();
  }

  public void updateTables(){
    double def = 0.0;
    	// Get data from NetworkTable
			// tshort	Sidelength of shortest side of the fitted bounding box (pixels)
			// tlong	Sidelength of longest side of the fitted bounding box (pixels)
			// thor		Horizontal sidelength of the rough bounding box (0 - 320 pixels)
			// tvert	Vertical sidelength of the rough bounding box (0 - 320 pixels)
			// camtran	Results of a 3D position solution, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll) 
			//camerapose_robotspace	  3D transform of the camera in the coordinate system of the robot (array (6))          
      tv = table.getEntry("tv").getDouble(def); //Whether the limelight has any valid targets (0 or 1)
      if (tv != 0.0){
        tx = table.getEntry("tx").getDouble(def); //Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
        ty = table.getEntry("ty").getDouble(def); //Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
        ta = table.getEntry("ta").getDouble(def); //Target Area (0% of image to 100% of image)
        ts = table.getEntry("ts").getDouble(def); //Skew or rotation (-90 degrees to 0 degrees)
        tl = table.getEntry("tl").getDouble(def); //The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
        cl = table.getEntry("cl").getDouble(def);
        getpipe = table.getEntry("getpipe").getDouble(def); //True active pipeline index of the camera (0 .. 9)

        botpose = table.getEntry("botpose").getDoubleArray(new double[6]); //Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
        botpose_wpiblue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]); //Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
        botpose_wpired = table.getEntry("botpose_wpired").getDoubleArray(new double[6]); //Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
        camerapose_targetspace = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]); //3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
        targetpose_cameraspace = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
        targetpose_robotspace = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]); //3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
        botpose_targetspace = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]); //3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
        tid = table.getEntry("tid").getDouble(def); //ID of the primary in-view AprilTag
      }
    
    
  }

  public boolean hasValidTarget(){
    boolean result = (tv != 0);
    return result;
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

  public double getYaw(){
    updateTables();
    return targetpose_robotspace[4];
  }

  public double getRoll(){
    updateTables();
    return targetpose_robotspace[5];
  }
}
