package team3647.frc2020.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Drivetrain;

public class GoStraightDistancePID extends CommandBase {
    private final Drivetrain dt;
    private boolean atSetpoint;
    private int setpoint;
    private final double kP;
    private final double kI;
    private final double kD;

    private final double integralCalcRadius;
    private double lastTimeUpdate;
    private double errorSum;
    private double previousError;

    // In feet --> set point is in feet, but position is in meters
    public GoStraightDistancePID(Drivetrain dt, int setpoint, double kP, double kI, double kD) {
        this.setpoint = setpoint;
        this.dt = dt;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        atSetpoint = false;
        this.errorSum = 0;
        integralCalcRadius = 0.5;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        dt.resetDistanceTraveled();
        dt.resetEncoders();
        lastTimeUpdate = Timer.getFPGATimestamp();
        previousError = 0;
    }

    @Override
    public void execute() {
        double error = Units.feetToMeters(setpoint) - dt.getDistanceTraveled();
        double deltaTime = Timer.getFPGATimestamp() - lastTimeUpdate;
        double errorRate = (error - previousError)/deltaTime;
        if (Math.abs(error) < integralCalcRadius) {
            errorSum += error; 
        }
        double outputSpeed = kP * error + kI * errorSum * deltaTime + kD * errorRate; 
        dt.arcadeDrive(outputSpeed, 0);
        if (Math.abs(error) < 0.1) {
            atSetpoint = true;
        }
        previousError = error;
        lastTimeUpdate = Timer.getFPGATimestamp();
    }

    @Override 
    public void end(boolean interrupted) {
        dt.end();
    }  

    @Override 
    public boolean isFinished() {
        return atSetpoint;
    }
}