// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmMotorSubsystem extends SubsystemBase {
  /** Creates a new ArmMotorSubsystem. */
  TalonFX armMotor;
  Joystick pJoystick;
  public ArmMotorSubsystem() {
    armMotor = new TalonFX(Constants.ARM_MOTOR_CHANNEL);
    pJoystick = new Joystick(Constants.RIGHT_JOYSTICK);
  }
  public void armUp() {
    armMotor.set(ControlMode.PercentOutput, 0.2);
  }
  public void armDown() {
    armMotor.set(ControlMode.PercentOutput,-0.2);
  }
  public void armStop() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }
  public double getThrottleValue() {
    return pJoystick.getThrottle();
  }
  public void moveArm() {
    armMotor.set(ControlMode.PercentOutput, getThrottleValue()*0.15);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
