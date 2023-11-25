// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmPneumaticSubsystem extends SubsystemBase {
  /** Creates a new ArmPneumaticSubsystem. */
  DoubleSolenoid doubleSolenoid;
  public ArmPneumaticSubsystem() {
    doubleSolenoid = new DoubleSolenoid(
      Constants.PNEUMATIC_HUB_CANID, 
      PneumaticsModuleType.CTREPCM, 
      Constants.EXTEND_CHANNEL, 
      Constants.RETRACT_CHANNEL);
  }
  public void armExtend () {
    doubleSolenoid.set(Value.kForward);
  }
  public void armRetract() {
    doubleSolenoid.set(Value.kReverse);
  }
  public boolean getArmPosition(){
    if (doubleSolenoid.get() == Value.kForward){
      return true;
    }else{
      return false;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
