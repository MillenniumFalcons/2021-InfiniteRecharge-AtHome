package team3647.frc2020.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.MathUtil;
import team3647.lib.drivers.ClosedLoopFactory;
import team3647.lib.drivers.SparkMaxFactory;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import team3647.lib.wpi.HALMethods;



public class Drivetrain implements PeriodicSubsystem {
  private final CANSparkMax leftMaster;
  private final CANSparkMax rightMaster;
  private final CANSparkMax leftSlave;
  private final CANSparkMax rightSlave;
  private final double kWheelDiameter;

  private final CANPIDController leftPIDController;
  private final CANPIDController rightPIDController;
  
  private final SimpleMotorFeedforward m_feedfoward;

  private final CANEncoder leftMasterEncoder;
  private final CANEncoder rightMasterEncoder;

  private DifferentialDriveOdometry odometry;
  private final PigeonIMU gyro;

  private double throttleMulti;
  private boolean isSlowed;

  private final ClosedLoopConfig leftPIDConfig;
  private final ClosedLoopConfig rightPIDConfig;

  private ControlType controlType = ControlType.kDutyCycle;

  public final periodicIO p_IO = new periodicIO();
  public static final double kDefaultDeadband = 0.02;
  public static final double kDefaultMaxOutput = 1.0;

  protected double m_deadband = kDefaultDeadband;
  protected double m_maxOutput = kDefaultMaxOutput;
  private boolean squareInputs = false;

    //because config is already inverted
  private double m_rightSideInvertMultiplier = 1.0;


  public Drivetrain(SparkMaxFactory.Configuration leftMasterConfig, SparkMaxFactory.Configuration rightMasterConfig, 
  SparkMaxFactory.Configuration leftSlaveConfig, SparkMaxFactory.Configuration rightSlaveConfig, ClosedLoopConfig leftPID, 
  ClosedLoopConfig rightPID, double driveWheelDiameter, int gyroID, double ks, double kv, double ka) {
    this.leftPIDConfig = leftPID;
    this.rightPIDConfig = rightPID;
    this.kWheelDiameter = driveWheelDiameter;
      
    leftMaster = SparkMaxFactory.createSparkMax(leftMasterConfig);
    rightMaster = SparkMaxFactory.createSparkMax(rightMasterConfig);
    leftSlave = SparkMaxFactory.createSparkMax(leftSlaveConfig);
    rightSlave = SparkMaxFactory.createSparkMax(rightSlaveConfig);
    leftMasterEncoder = leftMaster.getEncoder();
    rightMasterEncoder = rightMaster.getEncoder();
    leftPIDController = ClosedLoopFactory.createSparkMaxPIDController(leftMaster, leftMasterEncoder, leftPIDConfig, 0);
    rightPIDController = ClosedLoopFactory.createSparkMaxPIDController(rightMaster, rightMasterEncoder, rightPIDConfig, 0);

    m_feedfoward = new SimpleMotorFeedforward(ks, kv, ka);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    throttleMulti = 0.6;

    gyro = new PigeonIMU(gyroID);

    odometry = new DifferentialDriveOdometry(new Rotation2d());
  } 

  public static class periodicIO {
    //inputs

    public double distanceTraveled = 0;
    /**meters**/
    public double leftPosition = 0;
    public double rightPosition = 0;
    public double leftVelocity = 0;
    public double rightVelocity = 0;
    /**in degs**/
    public double heading = 0;
    public double[] ypr = {0, 0, 0};


    //outputs

    //volts
    public double leftfeedforward = 0;
    public double rightfeedforward = 0;

    //no units, deponds 
    public double driveLeftOutput = 0;
    public double driveRightOutput = 0;
  }

  public void arcadeDrive(double xSpeed, double zRotation)  {
    xSpeed *= throttleMulti;
    zRotation *= 0.3;
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, m_deadband);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = applyDeadband(zRotation, m_deadband);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    p_IO.driveLeftOutput = MathUtil.clamp(leftMotorOutput, -1.0, 1.0);
    
    p_IO.driveRightOutput = MathUtil.clamp(rightMotorOutput, -1.0, 1.0);
  }

  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public void setClosedLoopVelocity(double leftVelocityMPS, double rightVelocityMPS) {
    //demand is in m/s but kVelocity is in EncoderRPM
    controlType = ControlType.kVelocity;
    p_IO.leftfeedforward = m_feedfoward.calculate(leftVelocityMPS);
    p_IO.rightfeedforward = m_feedfoward.calculate(rightVelocityMPS);
    p_IO.driveLeftOutput = leftVelocityMPS / (leftPIDConfig.kEncoderVelocityToRPM * kWheelDiameter * Math.PI / 60.0);
    p_IO.driveRightOutput = rightVelocityMPS / (leftPIDConfig.kEncoderVelocityToRPM * kWheelDiameter * Math.PI / 60.0);
  }

  public void updateEncoders() {
    p_IO.leftPosition = (leftMasterEncoder.getPosition() * leftPIDConfig.kEncoderTicksToUnits);
    p_IO.rightPosition = (rightMasterEncoder.getPosition() * rightPIDConfig.kEncoderTicksToUnits);

    p_IO.leftVelocity = leftMasterEncoder.getVelocity() * leftPIDConfig.kEncoderVelocityToRPM * kWheelDiameter * Math.PI * (1/60);
    p_IO.rightVelocity = rightMasterEncoder.getVelocity() * rightPIDConfig.kEncoderVelocityToRPM * kWheelDiameter * Math.PI * (1/60);
  }

  public void resetEncoders() {
    p_IO.leftPosition = 0;
    p_IO.rightPosition = 0;
    leftMasterEncoder.setPosition(0);
    rightMasterEncoder.setPosition(0);
    p_IO.leftVelocity = 0;
    p_IO.rightVelocity = 0;
  }

  //Yaw --> 2d rotation from top view --> base rotation
  public void setZeroYaw() {
    gyro.setYaw(0);
  }

  public void setBrake() {
    leftMaster.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    leftMaster.setIdleMode(IdleMode.kCoast);
    rightMaster.setIdleMode(IdleMode.kCoast);
    leftSlave.setIdleMode(IdleMode.kCoast);
    rightSlave.setIdleMode(IdleMode.kCoast);
  }

  public void updateDistanceTraveled() {
    p_IO.distanceTraveled = (p_IO.leftPosition + p_IO.rightPosition)/2;
  }

  public void resetDistanceTraveled(){
    p_IO.distanceTraveled = 0;
  }

  public void setSlow(boolean slowed) {
    if (slowed) {
      isSlowed = true;
      throttleMulti = 0.2;
    } else {
      isSlowed = false;
      throttleMulti = 0.6;
    }
  }

  public void setOdometry(Pose2d pos, Rotation2d ang) {
    //why is there ang again? isn't again already in Pos?
    odometry.resetPosition(pos, ang);
  }

  public void resetOdometry() {
    resetEncoders();
    resetDistanceTraveled();
    setOdometry(new Pose2d(), new Rotation2d());
  }

  public double getx() {
    return Units.metersToInches(odometry.getPoseMeters().getX());
  }

  public double gety() {
    return Units.metersToInches(odometry.getPoseMeters().getY());
  }

  public boolean getSlow() {
    return isSlowed;
  }

  public double getDistanceTraveled() {
    return p_IO.distanceTraveled;
  }

  public double getdtVelocity() {
    return (p_IO.leftVelocity + p_IO.rightVelocity)/2;
  }

  public double getHeading() {
    return p_IO.heading;
  }

  public Rotation2d getRot2d(){
    return new Rotation2d(getHeading());
  }

  public Pose2d getRobotPose() {
    return this.odometry.getPoseMeters();
  }

  public CANPIDController getLeftController() {
    return this.leftPIDController;
  }

  public CANPIDController getRightController() {
    return this.rightPIDController;
  }

  public void stop() {
      leftMaster.set(0);
      rightMaster.set(0);
      leftSlave.set(0);
      rightSlave.set(0);
  }

  @Override
  public void init() {
    resetEncoders();
    resetDistanceTraveled();
    setZeroYaw();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void end() {
    //stops everything
    stop();
    p_IO.leftfeedforward = 0;
    p_IO.rightfeedforward = 0;
    p_IO.driveLeftOutput = 0;
    p_IO.driveRightOutput = 0;
    controlType = ControlType.kDutyCycle;
  }


  @Override
  public void readPeriodicInputs() {
    updateEncoders();
    updateDistanceTraveled();
    //read gyro
    gyro.getYawPitchRoll(p_IO.ypr);
    p_IO.heading = -Math.IEEEremainder(p_IO.ypr[0], 360);
  }

  @Override 
  public void writePeriodicOutputs() {
    if (controlType == ControlType.kDutyCycle) {
      leftMaster.set(p_IO.driveLeftOutput);
      rightMaster.set(p_IO.driveRightOutput);
    } else {
      leftPIDController.setReference(p_IO.driveLeftOutput, controlType, 0, p_IO.leftfeedforward);
      rightPIDController.setReference(p_IO.driveRightOutput, controlType, 0, p_IO.rightfeedforward);
    }
  }

  @Override
  public void periodic() {
    //when its regeistered in the commandScheuler --> every command interation, this is called before the command
    PeriodicSubsystem.super.periodic();
    odometry.update(Rotation2d.fromDegrees(getHeading()), p_IO.leftPosition, p_IO.rightPosition);
    // System.out.println("heading: " + p_IO.heading);
    // System.out.println("x: " + Units.metersToInches(odometry.getPoseMeters().getX()));
    // System.out.println("Y: " + Units.metersToInches(odometry.getPoseMeters().getY()));
  }

  @Override
  public String getName() {
    // TODO Auto-generated method stub
    return "DriveTrain";
  }
}
