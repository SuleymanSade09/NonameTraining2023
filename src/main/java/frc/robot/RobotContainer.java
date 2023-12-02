// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmMotorCommand;
import frc.robot.commands.ArmPneumaticsCommand;
import frc.robot.commands.ClawPneumaticsCommand;
import frc.robot.subsystems.ArmMotorSubsystem;
import frc.robot.subsystems.ArmPneumaticSubsystem;
import frc.robot.subsystems.ClawPneumaticSubsystem;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ClawPneumaticSubsystem m_clawPneumaticsSubsystem = new ClawPneumaticSubsystem();
  private final ArmPneumaticSubsystem m_armPneumaticSubsystem = new ArmPneumaticSubsystem();
  private final ClawPneumaticsCommand m_clawPneumaticsCommand = new ClawPneumaticsCommand(m_clawPneumaticsSubsystem);

  private final ArmPneumaticsCommand m_armPneumaticsCommand = new ArmPneumaticsCommand(m_armPneumaticSubsystem);
  private final ArmMotorSubsystem m_armMotorSubsystem = new ArmMotorSubsystem();
  private final ArmMotorCommand m_ArmMotorCommand = new ArmMotorCommand(m_armMotorSubsystem);

  private final Joystick joysticks = new Joystick(Constants.RIGHT_JOYSTICK);

  //private final AnalogInput analogInput = new AnalogInput(Constants.ARM_ENCODER);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Trigger clawPneumaticsButton = new JoystickButton(joysticks, Constants.CLAW_PNEUMATIC_BUTTON);
    clawPneumaticsButton.onTrue(m_clawPneumaticsCommand);
    Trigger armPneumaticsButton = new JoystickButton(joysticks, Constants.ARM_PNEUMATIC_BUTTON);
    armPneumaticsButton.onTrue(m_armPneumaticsCommand);
    Trigger armMoveButton = new JoystickButton(joysticks, Constants.ARM_MOVE_BUTTON);
    armMoveButton.whileTrue(m_ArmMotorCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
