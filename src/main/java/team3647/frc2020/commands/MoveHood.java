/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Hood;

public class MoveHood extends CommandBase {
    private final Hood m_hood;
    private final DoubleSupplier hoodPosition;

    /**
     * Creates a new MoveHood.
     */
    public MoveHood(Hood hood, DoubleSupplier hoodPosition) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_hood = hood;
        this.hoodPosition = hoodPosition;
        addRequirements(m_hood);
    }

    public MoveHood(Hood hood, double hoodPosition) {
        m_hood = hood;
        this.hoodPosition = () -> {
            return hoodPosition;
        };
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_hood.setPosition(hoodPosition.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
