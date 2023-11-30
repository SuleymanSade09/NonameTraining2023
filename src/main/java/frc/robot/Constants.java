// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static int RIGHT_JOYSTICK = 0;
  public final static int PNEUMATIC_HUB_CANID = 15;
  public final static int OPEN_CHANNEL = 5;
  public final static int CLOSE_CHANNEL = 1;
  public final static int EXTEND_CHANNEL = 7;
  public final static int RETRACT_CHANNEL = 0;
  public final static int CLAW_PNEUMATIC_BUTTON = 4;
  public final static int ARM_PNEUMATIC_BUTTON = 3;
  public final static int ARM_MOTOR_CHANNEL = 21;
  public final static int ARM_MOVE_BUTTON = 2;

  public final static int ARM_POSITION_FEEDFORWARD = 1;
  public final static int ARM_VELOCITY_FEEDFORWARD = 2;
  public final static int ARM_ACCELERATION_FEEDFORWARD = 3;

  public final static int ARM_ENCODER = 8;

  public final static double kS = 2;
  public final static double kG = 1.75;
  public final static double kV = 1.95;
  public final static double kA = 0;

  public final static double kP = 1;
  public final static double kI = 1;
  public final static double kD = 1;
  
}
