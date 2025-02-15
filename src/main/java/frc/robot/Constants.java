// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static double maxStatorCurrent = 50;
    public static double pidkG = 0.1;   // Voltage output to overcome gravity without a coral
    public static double pidkS = 0.25;  // Voltage output to overcome static friction
    public static double pidkV = 0.12;  // A velocity target of 1 rps requires this voltage output.
    public static double pidkA = 0.01;  // An acceleration of 1 rps/s requires this voltage output
    public static double pidkP = 4.8;   // A position error of 2.5 rotations requires this voltage output
    public static double pidkI = 0.0;   // no output for integrated error
    public static double pidkD = 0.1;   // A velocity error of 1 rps requires this voltage output
    public static double MMVelocity = 60;       // Target cruise velocity in revolutions per second (rps)
    public static double MMAcceleration = 120;  // Target acceleration in rps/sec
    public static double MMJerk = 600;          // Target jerk in rps/s/s

    public static double initPos = 0;

    public static int kDriverControllerPort = 0;  // Controller port
}
