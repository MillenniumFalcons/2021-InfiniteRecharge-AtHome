package team3647.frc2020.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.HotDogIndexer;

public class IndexerManualMode extends CommandBase {
    private final HotDogIndexer indexer;
    private final double demand;

    public IndexerManualMode(HotDogIndexer indexer, DoubleSupplier demand) {
        this.indexer = indexer;
        this.demand = demand.getAsDouble();
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        indexer.setManual(demand);
    }

    @Override 
    public void end(boolean interrupted) {
    }  

    @Override 
    public boolean isFinished() {
        return false;
    }
}