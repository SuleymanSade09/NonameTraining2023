// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.ArmMotorSubsystem;

public class ArmMotorCommand extends CommandBase {
  /** Creates a new ArmMotorCommand. */
  ArmMotorSubsystem armMotorSubsystem;
  public ArmMotorCommand(ArmMotorSubsystem armMotorSubsystem) {
    this.armMotorSubsystem = armMotorSubsystem;
    addRequirements(this.armMotorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armMotorSubsystem.moveArm();
    System.out.println("Arm velocity: " + armMotorSubsystem.getVelociy());
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
