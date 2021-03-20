package team3647.frc2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import team3647.lib.drivers.ClosedLoopFactory;
import team3647.lib.drivers.SparkMaxFactory;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;

public class Flywheel implements PeriodicSubsystem {
    private final CANSparkMax master;
    private final CANSparkMax slave;
    private final CANEncoder masterEncoder;
    private final CANEncoder slaveEncoder;
    private final CANPIDController PIDController;
    private final ClosedLoopConfig masterPIDConfig;
    private final SimpleMotorFeedforward feedforward;
    private final periodicIO pIO = new periodicIO();

    public Flywheel(SparkMaxFactory.Configuration masterConfig, SparkMaxFactory.Configuration slaveConfig, ClosedLoopConfig masterPID) {
        this.master = SparkMaxFactory.createSparkMax(masterConfig);
        this.masterEncoder = master.getEncoder();
        //slave inverted
        PIDController = ClosedLoopFactory.createSparkMaxPIDController(master, masterEncoder, masterPID, 0);
        this.slave = SparkMaxFactory.createSparkMax(slaveConfig);
        this.slaveEncoder = slave.getEncoder();
        this.masterPIDConfig = masterPID;
        feedforward = new SimpleMotorFeedforward(masterPID.kS, masterPID.kV);
        slave.follow(master);
    }

    class periodicIO {
        public double demand = 0;
        public double RPMReading = 0;
        public double prevRPMReading = 0;
        public ControlType controlType = ControlType.kDutyCycle;
        //volts b/c not talons
        public double feedforward = 0;
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.init();
    }

    public void setVelocity(double velocity) {
        //return the velocity back to encoder velocity
        pIO.demand = velocity / masterPIDConfig.kEncoderVelocityToRPM;
        double accelerationDemand = (pIO.RPMReading - velocity)/0.02;
        
        // how is the this calculated?
        pIO.feedforward = this.feedforward.calculate(velocity/60, accelerationDemand/60);
        pIO.controlType = ControlType.kVelocity;
    }

    public void setOpenLoop(double demand) {
        pIO.controlType = ControlType.kDutyCycle;
        pIO.demand = demand;
    }

    public void setSparkMax(ControlType ctrl, double demand, double feedforward) {
        //feedforward = Math.abs(feedforward) > 12 ? 12 * Math.signum(feedforward) : feedforward;
        if (Math.abs(feedforward) > 12) {
            feedforward = 12 * Math.signum(feedforward);
        }

        if (ctrl == ControlType.kDutyCycle) {
            master.set(demand);
        } else {
            //takes care of feedforard and PID already, no calculatiosn you need to do
            PIDController.setReference(demand, ctrl, 0, feedforward);
        }
    }

    @Override
    public void end() {
        master.set(0);
        pIO.demand = 0;
        pIO.feedforward = 0;
    }

    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.readPeriodicInputs();
        pIO.prevRPMReading = pIO.RPMReading;
        pIO.RPMReading = (masterEncoder.getVelocity() * masterPIDConfig.kEncoderVelocityToRPM + slaveEncoder.getVelocity() * masterPIDConfig.kEncoderTicksToUnits)/2;

    }
    
    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.writePeriodicOutputs();
        setSparkMax(pIO.controlType, pIO.demand, pIO.feedforward);
        
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
    
}