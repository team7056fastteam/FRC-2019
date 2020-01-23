package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import java.text.DecimalFormat;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SPI;

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
    
    private static final String auton1 = "Default Auton";
    private static final String auton2 = "Auton 2";
    private String autoSelected;
    private final SendableChooser<String> chooser = new SendableChooser<>();

    Timer at = new Timer();
    Timer it = new Timer();
    Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    double kP = 1;

    private static final AnalogInput sensor = new AnalogInput(0);
    private static final double VOLTS_TO_DIST = 3.5;

    public static double getVoltage() {
        return sensor.getVoltage();
    }
    
    public static double getDistance() {
        return getVoltage() * VOLTS_TO_DIST;
    }
      
    public static void updateDashboard() {
        SmartDashboard.putNumber("Distance (volts)", getVoltage());
        SmartDashboard.putNumber("Distance (real)", getDistance());

        DecimalFormat value = new DecimalFormat("#.#");
        double distance = getDistance();

        System.out.println(value.format(distance));
    }

    public void robotInit() {

        _LEDSwitch = new DigitalInput(1);
        _intake = new Intake();
        _pneumatics = new Pneumatics();
        _intake.robotInit();

        chooser.setDefaultOption("Default Auto", auton1);
        chooser.addOption("Auton 2", auton2);
        SmartDashboard.putData("Auton modes", chooser);

        gyro.calibrate();
        Shuffleboard.getTab("Gyro Alignment").add((Sendable) gyro);
    }

    public void robotPeriodic() {
        updateDashboard();
    }

    public void drive() {
        double defaultDriveSpeed;
        boolean auto = driver.getRawButton(8);

        Update_Limelight_Tracking();

        if (driver.getRawButton(6)) {
            defaultDriveSpeed = 0.35;
        }
        else {
            defaultDriveSpeed = 1;
        }

        double leftT = (driver.getRawAxis(2) * defaultDriveSpeed) * -1;
        double rightT = (driver.getRawAxis(3) * defaultDriveSpeed);
        double rightX = (driver.getRawAxis(0) * defaultDriveSpeed);

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
        final double DRIVE_K = 0.4;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 5;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.5;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        @SuppressWarnings("unused")
        
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
            limelightHasValidTarget = false;
            limelightDrive = 0.0;
            limelightSteer = 0.0;
            return;
        }

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
        autoSelected = chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + autoSelected);

        at.reset();
        at.start();
        it.reset();
    }

    public void autonomousPeriodic() {
        Update_Limelight_Tracking();

        switch (autoSelected) {

            // Autonomous 1:
            // Robot will rotate until a limelight target is found, drive towards it
            // and shoot the ball, then drive away from it and stop moving until
            // auton is disabled. 

            case auton1:
            default:

                if (limelightHasValidTarget) {

                    double already = 0;

                    // Check if the robot has driven towards the target already, if not drive towards it.
                    if (already == 0) {
                        drive.arcadeDrive(limelightDrive, limelightSteer);
                    }

                    // Check if the robot is 2 1/2 feet away, if so then shoot the ball and drive back.
                    if (getDistance() < 2.5) {
                        already = 1;
                        _intake.auton();
                        drive.arcadeDrive(-0.3, 0);
                    } else {
                        _intake.off();
                    }

                }
                // Rotate until a target is found, continuously resetting timer to not mess with further auton timing.
                else {
                    drive.arcadeDrive(0, 0.5);
                    at.reset();
                }

              break;

            // Autonomous 2
            // Robot will drive forward for 2 seconds then
            // stop moving until auton is disabled.

            case auton2:

                 double value = 0;

                // 0 > 1
                if (at.get() < 2.0) {
                    value = 0.4;
                }

                // End
                else {
                    value = 0;
                }

                drive.arcadeDrive(value, 0);

              break;
          }
    }

}