package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

public class Pneumatics {

    Joystick driver = new Joystick(0);

    DoubleSolenoid front = new DoubleSolenoid(1, 4);
    DoubleSolenoid back = new DoubleSolenoid(2, 5);

    public void modules() {
        climb();
    }

    public void climb() {

        if (driver.getRawButton(2)) {
            front.set(DoubleSolenoid.Value.kForward);
        }
        else if (driver.getRawButton(1)) {
            front.set(DoubleSolenoid.Value.kReverse);
        }
        else {
            front.set(DoubleSolenoid.Value.kOff);
        }

        if (driver.getRawButton(4)) {
            front.set(DoubleSolenoid.Value.kForward);
        }
        else if (driver.getRawButton(3)) {
            front.set(DoubleSolenoid.Value.kReverse);
        }
        else {
            front.set(DoubleSolenoid.Value.kOff);
        }

    }

}