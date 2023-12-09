// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmMotorSubsystem;

public class ArmMotorCommand extends CommandBase {
  /** Creates a new ArmMotorCommand. */
  ArmMotorSubsystem armMotorSubsystem;
  Timer timer;
  double initialPos;
  public static double velocity;

  public ArmMotorCommand(ArmMotorSubsystem armMotorSubsystem) {
    this.armMotorSubsystem = armMotorSubsystem;
    timer = new Timer();
    addRequirements(this.armMotorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
    initialPos = armMotorSubsystem.getArmPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get()> 0.2) {
      velocity = (armMotorSubsystem.getArmPosition() - initialPos)/timer.get();
      timer.reset();
      initialPos = armMotorSubsystem.getArmPosition();
    }
    armMotorSubsystem.moveArm();
    System.out.println("Arm velocity: " + velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armMotorSubsystem.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
