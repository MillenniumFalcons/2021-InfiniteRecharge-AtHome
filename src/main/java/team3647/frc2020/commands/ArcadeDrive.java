package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

public class ArcadeDrive extends CommandBase{
    private final Drivetrain m_dt;
    private final DoubleSupplier turn;
    private final DoubleSupplier throttle;

    public ArcadeDrive(Drivetrain m_dt, DoubleSupplier throttle, DoubleSupplier turn) {
        this.m_dt = m_dt;
        this.turn = turn;
        this.throttle = throttle;
        addRequirements(m_dt);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_dt.resetDistanceTraveled();
        m_dt.resetEncoders();
    }

    @Override
    public void execute() {
        m_dt.arcadeDrive(throttle.getAsDouble(), turn.getAsDouble());
    }

    @Override 
    public void end(boolean interrupted) {
        m_dt.end();
    }  

    @Override 
    public boolean isFinished() {
        return false;
    }
    
}