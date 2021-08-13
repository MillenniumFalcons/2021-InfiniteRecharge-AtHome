package team3647.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import team3647.lib.IndexerSignal;
import team3647.lib.drivers.TalonSRXFactory;
import team3647.lib.drivers.VictorSPXFactory;

public class HotDogIndexer implements PeriodicSubsystem {
    private final VictorSPX horizontalRoller;
    private final VictorSPX tunnelBeltRoller;
    private final DigitalInput tunnelBallDetection;
    private final PeriodicIO pIO;

    private boolean bannerSensorValue;

    class PeriodicIO {
        public double horizOut = 0;
        public double tunnelOut = 0;
    }

    public HotDogIndexer(VictorSPXFactory.Configuration hRollerConfig, VictorSPXFactory.Configuration TBConfig, int ballDetectPin) {
        horizontalRoller = VictorSPXFactory.createVictor(hRollerConfig);
        tunnelBeltRoller = VictorSPXFactory.createVictor(TBConfig);
        tunnelBallDetection = new DigitalInput(ballDetectPin);
        pIO = new PeriodicIO();
        
    }

    public void setSignal(IndexerSignal demand) {
        pIO.horizOut = demand.getHorizontalRollersOutput();
        pIO.tunnelOut = demand.getTunnelOutput();

        
    }

    public void setManual(double demand) {
        pIO.horizOut = demand;
        pIO.tunnelOut = demand;
    }

    public boolean getTunnelBallDetection() {
        return bannerSensorValue;
    }

    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.readPeriodicInputs();
        this.bannerSensorValue = !tunnelBallDetection.get();
    }

    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.writePeriodicOutputs();
        horizontalRoller.set(ControlMode.PercentOutput, pIO.horizOut);
        tunnelBeltRoller.set(ControlMode.PercentOutput, pIO.tunnelOut);
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.end();
        setSignal(IndexerSignal.STOP);
    }



    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }

}