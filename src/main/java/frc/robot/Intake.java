package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Intake {

    Joystick operator = new Joystick(1);

    TalonSRX leftLift = new TalonSRX(4);
    TalonSRX rightLift = new TalonSRX(5);
    TalonSRX intakeLeft = new TalonSRX(6);
    TalonSRX intakeRight = new TalonSRX(7);
    TalonSRX intakeTilt = new TalonSRX(8);
    
    VictorSPX hatchTilt = new VictorSPX(9);
    DigitalInput hatchLimit;

    Timer t = new Timer();

    public void robotInit() {
        hatchLimit = new DigitalInput(0);
    }

    public void autonomousInit() {
        t.reset();
        t.start();
    }

    public void modules() {
        intake();
        lift();
        hatch();
    }

    public void intake() {
        double value;
        double tilt = operator.getRawAxis(1) * -0.75;   // Left Y

        if (operator.getRawButton(2)) {     // A
            value = 0.5;
        }
        else if (operator.getRawButton(1)) {        // B
            value = -0.5;
        }
        else {
            value = -0.1;
        }

        intakeLeft.set(ControlMode.PercentOutput, value);
        intakeRight.set(ControlMode.PercentOutput, -value);
        intakeTilt.set(ControlMode.PercentOutput, tilt);
    }

    public void auton() {
        double value;

        // If it has been more than 2 seconds since limelight tracking started, shoot the ball
        if (t.get() > 2.0) {
            value = 0.5;
        }

        // If it has been more than 3 seconds, stop
        else if (t.get() > 3.0) {
            value = 0;
        }

        else {
            value = 0;
        }

        intakeLeft.set(ControlMode.PercentOutput, value);
        intakeRight.set(ControlMode.PercentOutput, -value);
    }

    public void lift() {
        double left = operator.getRawAxis(5) * 0.5;     // Right Y
        double right = operator.getRawAxis(5) * -0.5;   // Right Y

        leftLift.set(ControlMode.PercentOutput, left);
        rightLift.set(ControlMode.PercentOutput, right);
    }

    public void hatch() {
        if (operator.getRawButton(5)) {

            if (hatchLimit.get()) {
                hatchTilt.set(ControlMode.PercentOutput, 0.5);
            }
            else {
                hatchTilt.set(ControlMode.PercentOutput, -0.5);
            }
        }
        else if (operator.getRawButton(6)) {
            hatchTilt.set(ControlMode.PercentOutput, -0.5);
        }
        else {
            hatchTilt.set(ControlMode.PercentOutput, 0);
        }

    }

}