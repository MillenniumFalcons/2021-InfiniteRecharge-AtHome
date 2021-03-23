package team3647.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import team3647.lib.drivers.ClosedLoopFactory;
import team3647.lib.drivers.TalonSRXFactory;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;

public class KickerWheel implements PeriodicSubsystem {
    private final TalonSRX kickerWheel;
    private final ClosedLoopConfig PIDConfig;
    private final SimpleMotorFeedforward feedforward;
    private final periodicIO pIO = new periodicIO();
    
    public KickerWheel(TalonSRXFactory.Configuration kickerConfig, ClosedLoopConfig PIDConfig) {
        this.kickerWheel = TalonSRXFactory.createTalon(kickerConfig);
        //just for conversion purposes
        this.PIDConfig = PIDConfig;
        ClosedLoopFactory.configTalonPIDController(kickerWheel, FeedbackDevice.CTRE_MagEncoder_Relative, this.PIDConfig, 0);
        this.feedforward = new SimpleMotorFeedforward(PIDConfig.kS, PIDConfig.kV);
    }

    class periodicIO {
        public double RPMReading = 0;
        public double feedforward = 0;      
        public ControlMode controlMode = ControlMode.Disabled;       
    }

    @Override
    public void init() {

    }

    public void setRPM(double demandRPM) {
        //loop time is 0.02 --> divide to find acceleration
        double accelerationDemand = ((pIO.RPMReading - demandRPM) / 0.02);
        //feedfoward metod return evlocity or volts or rpm?
        pIO.feedforward = (this.feedforward.calculate(demandRPM, accelerationDemand) / 12.0)
            * (1023 / PIDConfig.maxVelocity);
        this.pIO.controlMode = ControlMode.Velocity;
    }

    public void setOpenLoop(double demand) {
        this.pIO.controlMode = ControlMode.PercentOutput;
        this.pIO.feedforward = demand;
    }

    public double getRPM() {
        return this.pIO.RPMReading;
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        pIO.feedforward = 0;
        kickerWheel.set(ControlMode.PercentOutput, 0);
        PeriodicSubsystem.super.end();
    }

    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.readPeriodicInputs();
        pIO.RPMReading = kickerWheel.getSelectedSensorPosition() * PIDConfig.kEncoderVelocityToRPM;
    }

    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.writePeriodicOutputs();
        kickerWheel.set(pIO.controlMode, pIO.feedforward);
    }
    
    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }

}