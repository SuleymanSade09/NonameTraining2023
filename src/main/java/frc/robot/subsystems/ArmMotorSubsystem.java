// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArmMotorCommand;

public class ArmMotorSubsystem extends SubsystemBase {
  /** Creates a new ArmMotorSubsystem. */
  TalonFX armMotor;
  Joystick pJoystick;
  ArmFeedforward feedforward;
  //AnalogInput analogInput;
  AnalogEncoder analogEncoder;
  PIDController pid;

  double initialPos;
  double initialTimer;

  Timer timer;

  double pidValue;
  double feedforwardValue;

  double velocity;

  private double kS = Constants.kS;
  private double kG = Constants.kG;
  private double kV = Constants.kV;
  private double kA = Constants.kA;

  private double kP = Constants.kP;
  private double kI = Constants.kI;
  private double kD = Constants.kD;
  

  public ArmMotorSubsystem() {
    timer = new Timer();
    armMotor = new TalonFX(Constants.ARM_MOTOR_CHANNEL);
    pJoystick = new Joystick(Constants.RIGHT_JOYSTICK);
    //analogInput = new AnalogInput(Constants.ARM_ENCODER);
    analogEncoder = new AnalogEncoder(Constants.ARM_ENCODER);
    feedforward = new ArmFeedforward(kS, kG, kV, kA);
    feedforward.calculate(Constants.ARM_POSITION_FEEDFORWARD, Constants.ARM_VELOCITY_FEEDFORWARD, Constants.ARM_ACCELERATION_FEEDFORWARD);
    pid = new PIDController(kP, kI, kD);

    analogEncoder.reset();
    analogEncoder.setDistancePerRotation(360);

    //armVelocity...
  }

  public double armMotorWithFeedforward() {
    return feedforward.calculate(getArmPosition(), ArmMotorCommand.velocity);
  }  
  public double getArmPosition(){
    return Math.abs((analogEncoder.getDistance()));
  }
  public double pidValue(){
    return pid.calculate(getArmPosition(), getArmPosition());
  }
  // public double getVelociy(){

  //   initialPos = getArmPosition();
    
  //   initialTimer = timer.get();
  //   if (timer.get()-initialTimer >= 0.01) {
  //     velocity = Math.abs((double)(getArmPosition() - initialPos)/(timer.get()-initialTimer));
  //   }
  //   timer.stop();
  //   return velocity;
  // }
  public void armStop() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }
  public double getThrottleValue() {
    return pJoystick.getThrottle();
  }
  public void moveArm() {
    // pidValue = pidValue();
    // feedforwardValue = armMotorWithFeedforward();
    armMotor.set(ControlMode.PercentOutput, (pidValue() + armMotorWithFeedforward())*0.2);
    // armMotor.set(ControlMode.PercentOutput, getThrottleValue()*0.15);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
