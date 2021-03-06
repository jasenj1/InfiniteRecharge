/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class VisionTakeOverGroup extends CommandGroup {

  public VisionTakeOverGroup() {
    addParallel(new ManualVisionDriving());
    addSequential(new WaitForTarget());
    addSequential(new RampDown(3, 0));
  }
}
