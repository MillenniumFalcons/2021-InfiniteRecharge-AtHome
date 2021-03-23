package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.KickerWheel;

public class AccelerateKickerFlywheel extends CommandBase {
    public final Flywheel flywheel;
    public final KickerWheel kickerwheel;

    public double flywheelDemand;
    public double kickerDemand;

    public AccelerateKickerFlywheel(Flywheel flywheel, KickerWheel kickerwheel, double flywheelDemand, double kickerDemand) {
        this.flywheel = flywheel;
        this.kickerwheel = kickerwheel;
        this.flywheelDemand = flywheelDemand;
        this.kickerDemand = kickerDemand;
        addRequirements(flywheel, kickerwheel);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();

    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        kickerwheel.setRPM(kickerDemand);
        flywheel.setVelocity(flywheelDemand);
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        flywheel.end();
        kickerwheel.end();
    }

    
    
}