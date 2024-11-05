// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeOutakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RevEncoder;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import javax.management.RuntimeErrorException;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer { 
    
    /* ----------------------------- INITIALIZE AND DECLARE SUBSYSTEMS ETC ----------------------------- */

    
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    private final IntakeOutakeSubsystem m_robotIntakeOutake = new IntakeOutakeSubsystem();
    private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
    private final RevEncoder m_encoderSubsystem = new RevEncoder();

    ParallelCommandGroup p = new ParallelCommandGroup(getAutonomousCommandShoot(500, 800),
     getAutonomousCommandWrist(Constants.DriveConstants.wristVolts * 2, 500));
        
    // The driver's controller
    public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    public static XboxController m_commandController = new XboxController(OIConstants.kCommandControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

     public Command getAutonomousCommand(double armVelocity, int armDuration, double wristVelocity, int wristDuration) {
        // Create the commands for arm and wrist
        Command armCommand = Commands.runOnce(() -> autonomousArm(armVelocity, armDuration));
        Command wristCommand = Commands.runOnce(() -> autonomousWrist(wristVelocity, wristDuration));

        // Create and return a ParallelCommandGroup
        return new ParallelCommandGroup(armCommand, wristCommand);
    }
    
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, true),
                m_robotDrive));

        //LED on
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {


        /* ----------------------------- ALL BUTTONS ARE BOUND HERE ----------------------------- */
        /*
         * DRIVER CONTROLLER COMMANDS
         * SET X = RT
         * SPIN = LT
         * ELEVATOR UP = Y
         * ELEVATOR DOWN = A
         */
        //set wheels to X formation [Driver Controller RT]
        new JoystickButton(m_driverController, Axis.kRightTrigger.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));
        //spin [Driver Controller LT]
        JoystickButton btnSpin = new JoystickButton(m_driverController, Axis.kLeftTrigger.value);
        btnSpin.whileTrue(new RunCommand(
            () -> m_robotDrive.drive(0, 0, 1, true, true),
            m_robotDrive));

        //elevator up [Driver Controller Y]
        JoystickButton btnElevatorUp = new JoystickButton(m_driverController, Button.kY.value);
        btnElevatorUp.whileTrue(new RunCommand(
            () -> m_robotElevator.setElevator(-Constants.DriveConstants.elevatorVolts),
            m_robotElevator));

        btnElevatorUp.whileFalse(new RunCommand(
            () -> m_robotElevator.setElevator(0),
            m_robotElevator));

        //elevator down [Driver Controller A]
        JoystickButton btnElevatorDown = new JoystickButton(m_driverController, Button.kA.value);
        btnElevatorDown.whileTrue(new RunCommand(
            () -> m_robotElevator.setElevator(Constants.DriveConstants.elevatorVolts),
            m_robotElevator));
        btnElevatorDown.whileFalse(new RunCommand(
            () -> m_robotElevator.setElevator(0),
            m_robotElevator));
        
        /*
         * COMMAND CONTROLLER
         * INTAKE IN = RB
         * INTAKE OUT (SHOOT) = LEFT STICK 
         * INSTANT SHOOT = LS
         * WRIST UP = Y
         * WRIST DOWN = A
         * ARM UP = RT
         * ARM DOWN = RT
         */
        //intake in [Command Controller RB]
        JoystickButton btnIntakeIn = new JoystickButton(m_commandController, Button.kRightBumper.value);
        btnIntakeIn.whileTrue(new RunCommand(
            () -> m_robotIntakeOutake.intakeOn(Constants.DriveConstants.intakeOutakeVolts),
            m_robotIntakeOutake));
        btnIntakeIn.whileFalse(new RunCommand(
            () -> m_robotIntakeOutake.intakeOn(0),
            m_robotIntakeOutake));

        //shoot [Command Controller LB]
        JoystickButton btnShoot = new JoystickButton(m_commandController, Button.kLeftBumper.value);
        btnShoot.whileTrue(new RunCommand(
            () -> m_robotIntakeOutake.shoot(Constants.DriveConstants.intakeOutakeVolts),
            m_robotIntakeOutake));
        btnShoot.whileFalse(new RunCommand(
            () -> m_robotIntakeOutake.shoot(0),
            m_robotIntakeOutake));   
        
        //shoot instant [Command Controller LS]
        // JoystickButton btnFShoot = new JoystickButton(m_commandController, Button.kLeftStick.value);
        // btnFShoot.whileTrue(new RunCommand(
        //     () -> m_robotIntakeOutake.shootInstant(Constants.DriveConstants.intakeOutakeVolts),
        //     m_robotIntakeOutake));
        // btnFShoot.whileFalse(new RunCommand(
        //     () -> m_robotIntakeOutake.shootInstant(0.1), 
        //     m_robotIntakeOutake));

        //wrist down [Command Controller Y]
        JoystickButton btnWristUp = new JoystickButton(m_commandController, Button.kY.value);
        btnWristUp.whileTrue(new RunCommand(
            () -> m_robotIntakeOutake.setWrist(Constants.DriveConstants.wristVolts),
            m_robotIntakeOutake));
        btnWristUp.whileFalse(new RunCommand(
            () -> m_robotIntakeOutake.setWrist(0.1),
            m_robotIntakeOutake));

        //wrist up [Command Controller A]
        JoystickButton btnWristDown = new JoystickButton(m_commandController, Button.kA.value);
        btnWristDown.whileTrue(new RunCommand(
            () -> m_robotIntakeOutake.setWrist(-Constants.DriveConstants.wristVolts),
            m_robotIntakeOutake));
        btnWristDown.whileFalse(new RunCommand(
            () -> m_robotIntakeOutake.setWrist(0.1),
            m_robotIntakeOutake));        

        //arm up [Command Controller RT]
        JoystickButton btnArmUp = new JoystickButton(m_commandController, Button.kB.value);
        btnArmUp.whileTrue(new RunCommand(
            () -> m_armSubsystem.setArm(-Constants.DriveConstants.armVolts),
            m_armSubsystem));
        btnArmUp.whileFalse(new RunCommand(
            () -> m_armSubsystem.setArm(-0.2),
            m_armSubsystem));

        //arm down [Command Controller LT]
        JoystickButton btnArmDown = new JoystickButton(m_commandController, Button.kX.value);
        btnArmDown.whileTrue(new RunCommand(
            () -> m_armSubsystem.setArm(Constants.DriveConstants.armVolts),
            m_armSubsystem));
        btnArmDown.whileFalse(new RunCommand(
            () -> m_armSubsystem.setArm(-0.2),
            m_armSubsystem));

        // //wrist preset [Command Controller LSB]
        // JoystickButton btnWristPreset = new JoystickButton(m_commandController, Button.kLeftStick.value);
        // btnWristPreset.whileTrue(new RunCommand(
        //     () -> m_encoderSubsystem.wristPreset(0.23)));
        // btnWristPreset.whileFalse(new RunCommand(
        //     () -> m_robotIntakeOutake.setWrist(0.1)));

        // //arm preset [Command Controller RSB]
        // JoystickButton btnArmPreset = new JoystickButton(m_commandController, Button.kRightStick.value);
        // btnArmPreset.whileTrue(new RunCommand(
        //    () -> m_encoderSubsystem.armPreset(-0.145)));
        // btnArmPreset.whileFalse(new RunCommand(
        //     () -> m_armSubsystem.setArm(-0.2)));

        //speaker shooter preset [Command Controller RSB]
        JoystickButton btnShootSpeakerPreset = new JoystickButton(m_commandController, Button.kRightStick.value);
        btnShootSpeakerPreset.whileTrue(new RunCommand(
           () -> m_encoderSubsystem.shooterSpeakerPreset()));
        
        //amp shooter preset [Command Controller LSB] 
        JoystickButton btnShootAmpPreset = new JoystickButton(m_commandController, Button.kLeftStick.value);
        btnShootAmpPreset.whileTrue(new RunCommand( 
            () -> m_encoderSubsystem.shooterAmpPreset()));

       // intake preset [Driver Controller LSB]
       JoystickButton btnIntakePreset = new JoystickButton(m_driverController, Button.kLeftBumper.value);
       btnIntakePreset.whileTrue(new RunCommand( 
           () -> m_encoderSubsystem.intakePreset()));
           
        
        //Hanging Shoot preset [Driver Controller RSB]
        JoystickButton btnHangingShoot = new JoystickButton(m_driverController, Button.kRightBumper.value);
        btnHangingShoot.whileTrue(new RunCommand( 
            () -> m_robotIntakeOutake.shootInstant(DriveConstants.intakeOutakeVolts)));
             btnHangingShoot.whileFalse(new RunCommand( 
            () -> m_robotIntakeOutake.shootInstant(0),m_robotIntakeOutake));

        // JoystickButton btnShootPreset = new JoystickButton(m_driverController, Button.kRightStick.value);
        //     btnShootPreset.whileTrue(new RunCommand(
        //         () -> shootPreset(), m_robotIntakeOutake));
    }

    /* ----------------------------- METHODS FOR AUTONOMOUS COMMANDS ----------------------------- */

    private void wait(int ms) {
        try {Thread.sleep(ms);}
        catch(InterruptedException e) {throw new RuntimeErrorException(null);}
    }

    private void autonomousShoot(int ms, int charge) {
        m_robotIntakeOutake.autoShootButActuallyAuto(Constants.DriveConstants.intakeOutakeVolts * 3, charge);
        
        try {Thread.sleep(ms);}
        catch(InterruptedException e) {throw new RuntimeErrorException(null);}

        m_robotIntakeOutake.setBackIntake(0);
        m_robotIntakeOutake.setFrontIntake(0);
    }

    public void autonomousArm(double v, int ms) {
        m_armSubsystem.setArm(v);

        try {Thread.sleep(ms);}
        catch(InterruptedException e) {throw new RuntimeErrorException(null);}

        m_armSubsystem.setArm(0);
    }

    public void autoIntakePreset() {
        m_encoderSubsystem.intakePreset();
    }

    public void autoIntake(double v, int ms) {
         m_robotIntakeOutake.intakeOn(v);

        try {Thread.sleep(ms);}
        catch(InterruptedException e) {throw new RuntimeErrorException(null);}

        m_robotIntakeOutake.intakeOn(0);
    }


    public void autonomousWrist(double v, int ms) {
        m_robotIntakeOutake.setWrist(v);

        try {Thread.sleep(ms);}
        catch(InterruptedException e) {throw new RuntimeErrorException(null);}

        m_robotIntakeOutake.setWrist(0);
    }

    public void autonomousIntake(int ms) {
        m_robotIntakeOutake.intakeOn(Constants.DriveConstants.intakeOutakeVolts);

        try {Thread.sleep(ms);}
        catch(InterruptedException e) {throw new RuntimeErrorException(null);}

        m_robotIntakeOutake.intakeOn(0);
    }

    public void autonomousButNotDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, int ms, boolean stopAfter) {
        m_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit);

        try {Thread.sleep(ms);}
        catch(InterruptedException e) {throw new RuntimeErrorException(null);}

        if(stopAfter) m_robotDrive.drive(0, 0, 0, fieldRelative, rateLimit);
    }

    public void autonomousFRONTIntake(int ms) {
        m_robotIntakeOutake.frontIntakeOn(Constants.DriveConstants.intakeOutakeVolts);

        try {Thread.sleep(ms);}
        catch(InterruptedException e) {throw new RuntimeErrorException(null);}

        m_robotIntakeOutake.frontIntakeOn(0);
    }

   




    /* ----------------------------- AUTONOMOUS COMMANDS ----------------------------- */

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommandStart() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // trolololol
            List.of(),
            // End 3 meters straight ahead of where we starranted, facing forward
            new Pose2d(-3, 0.1, new Rotation2d(0)),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }

    public Command getAutonomousCommandGeneric(double startX, double startY, double startAngle, double endX, double endY, double endAngle) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(

            new Pose2d(startX, startY, new Rotation2d(startAngle)),
            // trolololol
            List.of(),
            // End 3 meters straight ahead of where we starranted, facing forward
            new Pose2d(endX, endY, new Rotation2d(endAngle)),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }

    public Command getAutonomousCommandShoot(int ms, int charge) {
        return Commands.runOnce(() -> autonomousShoot(ms, charge));
    }
    //These two should run parallel to each other, but they run sequentially
    public Command getAutonomousCommandArm(double v, int ms) {
        return Commands.runOnce(() -> autonomousArm(v, ms));
    }

    public Command getAutonomousCommandWrist(double v, int ms) {
        return Commands.runOnce(() -> autonomousWrist(v, ms));
    }

    public Command getAutonomousCommandIntake(int ms) {
        return Commands.runOnce(() -> autonomousIntake(ms));
    }

    public Command getAutonomousCommandFrontIntake(int ms) {
        return Commands.runOnce(() -> autonomousFRONTIntake(ms));
    }

    public Command getAutonomusCommandButNotDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, int ms, boolean stopAfter) {
        return Commands.runOnce(() -> autonomousButNotDrive(xSpeed, ySpeed, rot, fieldRelative, rateLimit, ms, stopAfter));
    }

    public Command setWait(int ms) {
        return Commands.runOnce(() -> wait(ms));
    }

    public Command parallel() {
        Command c1 = getAutonomousCommandWrist(2, 200);
        Command c2 = getAutonomousCommandArm(2, 300);

        return Commands.parallel(c1, c2);
    }

    //uses the "parallel" method to make them run parallel

    public Command getAutoIntakePreset() {
        return Commands.runOnce(() -> autoIntakePreset());
    }

    public Command getAutoIntake(double v, int ms) {
        return Commands.runOnce(() -> autoIntake(v, ms));
    }

    
    SequentialCommandGroup p2 = new SequentialCommandGroup();

    public SequentialCommandGroup gegfsdf() {
        p2.addCommands(
            new ParallelCommandGroup(
                getAutonomousCommandArm(Constants.DriveConstants.armVolts * 2, 300),
                getAutonomousCommandWrist(Constants.DriveConstants.wristVolts * 2, 400)
                ),
            new SequentialCommandGroup(getAutonomousCommandIntake(500)
            )
        );

        return p2;
    }


    public Command getAutonomousCommandEnd() {
        //System.out.println("Shooting ended.");
         // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(-3, 0.1, new Rotation2d(0)),
            // trolololol
            List.of(),
            // End 3 meters straight ahead of where we starranted, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }




    // ParallelCommandGroup pc;

    // public ParallelCommandGroup c() {
    //     pc = new ParallelCommandGroup(getAutonomousCommandArm(Constants.DriveConstants.armVolts * 2, 500),
    //     getAutonomousCommandShoot(100, 100));

    //     return pc;
    // }


    
    public void shootPreset() { 
        if (RobotContainer.m_commandController.getRightStickButtonPressed()) {
            m_armSubsystem.setArm(3.0);

            try {Thread.sleep(750);}
            catch(InterruptedException e) {throw new RuntimeErrorException(null);}

            m_armSubsystem.setArm(0);

            m_robotIntakeOutake.setWrist(3.0);
            try {Thread.sleep(750);}
            catch(InterruptedException e) {throw new RuntimeErrorException(null);}

            m_robotIntakeOutake.setWrist(0);

            m_robotIntakeOutake.autoShoot(3);
            try {Thread.sleep(750);}
            catch(InterruptedException e) {throw new RuntimeErrorException(null);}

            m_robotIntakeOutake.autoShoot(0);

        }

        else { 
            m_armSubsystem.setArm(0);
            m_robotIntakeOutake.setWrist(0);
            m_robotIntakeOutake.autoShoot(0);
        }
    }
}