package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Spark;

import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

    Joystick driver = new Joystick(0);
    Joystick operator = new Joystick(1);


    CANSparkMax driveLeft1 = new CANSparkMax(10, MotorType.kBrushless);     // Front Left drive motor
    CANSparkMax driveLeft2 = new CANSparkMax(2, MotorType.kBrushless);      // Back Left drive motor
    CANSparkMax driveRight1 = new CANSparkMax(3, MotorType.kBrushless);     // Front Right drive motor
    CANSparkMax driveRight2 = new CANSparkMax(1, MotorType.kBrushless);     // Back Right drive motor

    SpeedControllerGroup leftDrive = new SpeedControllerGroup(driveLeft1, driveLeft2);
    SpeedControllerGroup rightDrive = new SpeedControllerGroup(driveRight1, driveRight2);
    private final DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);

    Intake _intake;
    Pneumatics _pneumatics;

    Spark LED = new Spark(0);
    DigitalInput _LEDSwitch;

    private boolean limelightHasValidTarget = false;
    private double limelightDrive = 0;
    private double limelightSteer = 0;

    Timer t = new Timer();

    public void robotInit() {
        _LEDSwitch = new DigitalInput(1);
        _intake = new Intake();
        _pneumatics = new Pneumatics();

        _intake.robotInit();
    }

    public void drive() {
        double defaultDriveSpeed;
        boolean auto = operator.getRawButton(8);

        Update_Limelight_Tracking();

        if (driver.getRawButton(6)) {
            defaultDriveSpeed = 0.35;
        }
        else {
            defaultDriveSpeed = 1;
        }

        double leftT = (deadband(driver.getRawAxis(2) * defaultDriveSpeed)) * -1;
        double rightT = deadband(driver.getRawAxis(3) * defaultDriveSpeed);
        double rightX = deadband(driver.getRawAxis(0) * defaultDriveSpeed);

        double _drive = leftT + rightT;

        if (auto) {
            if (limelightHasValidTarget)
            {
                drive.arcadeDrive(limelightDrive, limelightSteer);
            }
            else {
                drive.arcadeDrive(0, 0);
            }

        }
        else {
            drive.arcadeDrive(_drive, rightX);
        }
        
    }

    public double deadband(double value) {
        double deadband = .15;
    
        if (Math.abs(value) < deadband) {
          return 0;
        }
        
        if (value > 0)
        {
          value = (value - deadband) / (1.0 - deadband);
        }
        
        else
        {
          value = -((-value - deadband) / (1.0 - deadband));
        }
    
        return value;
    }

    public void led() {
        if (_LEDSwitch.get()) {

            LED.set(-.01);
        }
        else {
            LED.set(.43);
        }
    }

    public void teleopPeriodic() {
        drive();
        led();

        _intake.modules();
        _pneumatics.modules();
    }

    double atDesiredTarget = 0;

    public void Update_Limelight_Tracking() {
        
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        @SuppressWarnings("unused")
        
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

            limelightHasValidTarget = false;
            limelightDrive = 0.0;
            limelightSteer = 0.0;
            return;
        }

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        limelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        limelightSteer = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
            drive_cmd = MAX_DRIVE;
        }
        
        limelightDrive = drive_cmd;
    }

    public void autonomousInit() {
        t.reset();
        t.start();

        _intake.autonomousInit();
    }

    public void autonomousPeriodic() {
        
        // If it has been less than 2 seconds since autonomous started, drive forwards
        if (t.get() < 2.0) {
            drive.arcadeDrive(0.5, 0.0);
        }

        // If it has been more than 2 seconds, stop the robot
        else {
            
            if (limelightHasValidTarget)
            {
                drive.arcadeDrive(limelightDrive, limelightSteer);
            }
            else {
                if (atDesiredTarget == 1) {
                    _intake.auton();
                }
                drive.arcadeDrive(0, 0);
            }
        }
    }

}