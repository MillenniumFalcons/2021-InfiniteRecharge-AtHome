package team3647.frc2020.autonomous;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import team3647.frc2020.robot.Constants;

public class Trajectories {
    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.cDrivetrain.kS, Constants.cDrivetrain.kV,
            Constants.cDrivetrain.kA), Constants.cDrivetrain.kDriveKinematics, Constants.cDrivetrain.maxVoltage);     
    
    private static final TrajectoryConfig forwardTrajectoryConfig =
            new TrajectoryConfig(Constants.cDrivetrain.kMaxSpeedMetersPerSecond,
            Constants.cDrivetrain.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.cDrivetrain.kDriveKinematics)
            .addConstraint(autoVoltageConstraint).setReversed(false);

    private static final TrajectoryConfig reverseTrajectoryConfig =
        new TrajectoryConfig(Constants.cDrivetrain.kMaxSpeedMetersPerSecond,
                Constants.cDrivetrain.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.cDrivetrain.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(true);
    

    public static Trajectory GalaticSearch_A_RedTraject = TrajectoryGenerator.generateTrajectory(Constants.cField.GalaticSearch_A_Red_startingPoint,
    Arrays.asList(Constants.cField.GalaticSearch_A_Red_firstBall, Constants.cField.GalaticSearch_A_Red_secondBall, Constants.cField.GalaticSearch_A_Red_thirdBall),
     Constants.cField.GalaticSearch_A_Red_endingPoint, reverseTrajectoryConfig);

     public static Trajectory GalaticSearch_B_RedTraject = TrajectoryGenerator.generateTrajectory(Constants.cField.GalaticSearch_B_Red_startingPoint,
      Arrays.asList(Constants.cField.GalaticSearch_B_Red_firstBall, Constants.cField.GalaticSearch_B_Red_secondBall, Constants.cField.GalaticSearch_B_Red_thirdBall),
       Constants.cField.GalaticSearch_B_Red_endingPoint, reverseTrajectoryConfig);

       public static Trajectory AutoNav_Slalom = TrajectoryGenerator.generateTrajectory(Constants.cField.AutoNav_Slalom_START,
      Arrays.asList(Constants.cField.AutoNav_Slalom_A, Constants.cField.AutoNav_Slalom_B, Constants.cField.AutoNav_Slalom_C, Constants.cField.AutoNav_Slalom_D, Constants.cField.AutoNav_Slalom_E,
      Constants.cField.AutoNav_Slalom_F, Constants.cField.AutoNav_Slalom_G, Constants.cField.AutoNav_Slalom_H,Constants.cField.AutoNav_Slalom_I, Constants.cField.AutoNav_Slalom_J, 
      Constants.cField.AutoNav_Slalom_K, Constants.cField.AutoNav_Slalom_L, Constants.cField.AutoNav_Slalom_M, Constants.cField.AutoNav_Slalom_N, Constants.cField.AutoNav_Slalom_O, Constants.cField.AutoNav_Slalom_P),
      Constants.cField.AutoNav_Slalom_END, forwardTrajectoryConfig);

      public static Trajectory AutoNav_BarrelRace = TrajectoryGenerator.generateTrajectory(Constants.cField.AutoNav_Barrel_Race_Start,
      Arrays.asList(Constants.cField.AUTONAV_Barrel_Race_1, Constants.cField.AUTONAV_Barrel_Race_2, Constants.cField.AUTONAV_Barrel_Race_3,
      Constants.cField.AUTONAV_Barrel_Race_4, Constants.cField.AUTONAV_Barrel_Race_5, Constants.cField.AUTONAV_Barrel_Race_6, Constants.cField.AUTONAV_Barrel_Race_7, 
      Constants.cField.AUTONAV_Barrel_Race_8, Constants.cField.AUTONAV_Barrel_Race_9, Constants.cField.AUTONAV_Barrel_Race_10, Constants.cField.AUTONAV_Barrel_Race_11, 
      Constants.cField.AUTONAV_Barrel_Race_12, Constants.cField.AUTONAV_Barrel_Race_13, Constants.cField.AUTONAV_Barrel_Race_14, Constants.cField.AUTONAV_Barrel_Race_15, 
      Constants.cField.AUTONAV_Barrel_Race_16, Constants.cField.AUTONAV_Barrel_Race_17, Constants.cField.AUTONAV_Barrel_Race_18, Constants.cField.AUTONAV_Barrel_Race_19,
      Constants.cField.AUTONAV_Barrel_Race_20, Constants.cField.AUTONAV_Barrel_Race_21, Constants.cField.AUTONAV_Barrel_Race_22, Constants.cField.AUTONAV_Barrel_Race_23),
      Constants.cField.AutoNav_Barrel_Race_END, forwardTrajectoryConfig);

      public static Trajectory AutoNav_Bounce_forwardA = TrajectoryGenerator.generateTrajectory(Constants.cField.BounceForwardA_START,
      Arrays.asList(Constants.cField.BounceForwardA_firstPoint, Constants.cField.BounceForwardA_secondPoint),
       Constants.cField.BounceForwardA_END, forwardTrajectoryConfig);

       public static Trajectory AutoNav_Bounce_backwardA = TrajectoryGenerator.generateTrajectory(Constants.cField.BounceBackwardsA_START,
      Arrays.asList(Constants.cField.BounceBackwardsA_Setup, Constants.cField.BounceBackwardsA_firstPoint, Constants.cField.BounceBackwardsA_secondPoint),
       Constants.cField.BounceBackwardsA_END, reverseTrajectoryConfig);

       public static Trajectory AutoNav_Bounce_forwardsB = TrajectoryGenerator.generateTrajectory(Constants.cField.BounceForwardB_START,
      Arrays.asList(Constants.cField.BounceForwardsB_firstPoint, Constants.cField.BounceForwardsB_secondPoint,
      Constants.cField.BounceForwardsB_thirdPoint, Constants.cField.BounceForwardsB_fourthPoint, Constants.cField.BounceForwardsB_fifthPoint),
       Constants.cField.BounceForwardsB_END, forwardTrajectoryConfig);

       public static Trajectory AutoNav_Bounce_backwardB = TrajectoryGenerator.generateTrajectory(Constants.cField.BounceBackwardsB_START,
      Arrays.asList(Constants.cField.BounceBackwardsB_firstPoint, Constants.cField.BounceBackwardsB_secondPoint),
       Constants.cField.BounceBackwardsB_END, reverseTrajectoryConfig);




}
