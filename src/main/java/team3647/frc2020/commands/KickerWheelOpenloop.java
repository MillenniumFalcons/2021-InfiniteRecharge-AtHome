/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.KickerWheel;

public class KickerWheelOpenloop extends CommandBase {
  private final KickerWheel m_kickerWheel;
  private final double demand;
  /**
   * Creates a new KickerWheelOpenloop.
   */
  public KickerWheelOpenloop(KickerWheel kickerWheel, double demand) {
    m_kickerWheel = kickerWheel;
    this.demand = demand;
    addRequirements(m_kickerWheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_kickerWheel.setOpenloop(demand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_kickerWheel.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
