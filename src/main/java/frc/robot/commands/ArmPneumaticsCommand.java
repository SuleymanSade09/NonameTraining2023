// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPneumaticSubsystem;

public class ArmPneumaticsCommand extends CommandBase {
  /** Creates a new ArmPneumaticsCommand. */
  ArmPneumaticSubsystem m_armPneumaticSubsystem;
  boolean initialArmPosition;
  public ArmPneumaticsCommand(ArmPneumaticSubsystem armPneumaticSubsystem) {
    m_armPneumaticSubsystem = armPneumaticSubsystem;
    addRequirements(m_armPneumaticSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialArmPosition = m_armPneumaticSubsystem.getArmPosition();
    if (m_armPneumaticSubsystem.getArmPosition()){
      m_armPneumaticSubsystem.armRetract();
      System.out.println("armRetract");
    } else {
      m_armPneumaticSubsystem.armExtend();
      System.out.println("armExtend");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (initialArmPosition != m_armPneumaticSubsystem.getArmPosition()) {
      return true;
    }
    else {
    return false;
    }
  }
}
