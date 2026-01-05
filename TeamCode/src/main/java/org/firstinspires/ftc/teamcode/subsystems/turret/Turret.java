package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    Servo turretServo1;
    Servo turretServo2;

    public AnalogInput analogInput1;
    public AnalogInput analogInput2;

    public Servo turret1;
    public Servo turret2;

    private PIDController controller;
    private double p = 0, i = 0, d = 0; //has to be tuned
    private double f = 0;

    public Turret(HardwareMap hwMap) {
        turretServo1 = hwMap.get(Servo.class, "turret1");
        turretServo2 = hwMap.get(Servo.class, "turret2");
        analogInput1 = hwMap.get(AnalogInput.class, "analogEncoder1");
        analogInput2 = hwMap.get(AnalogInput.class, "analogEncoder2");

        this.controller = new PIDController(p, i, d);
    }

    public double powerToPosition(double power) {
        return power * 2 - 1;
    }

    public void setPower(double power) {
        // TODO may have to invert one depending on hardware
        turret1.setPosition(powerToPosition(power));
        turret2.setPosition(powerToPosition(power));
    }

    // TODO may have to make 2 run to pos, one for each servo
    public void runToPos(double targetAngle) {
        controller.setPID(p, i, d);
        double pos = this.getPosition();
        double pid = controller.calculate(pos, targetAngle);
        double ff = Math.cos(Math.toRadians(targetAngle)) * f;
        double power = pid + ff;

        turret1.setPosition(powerToPosition(power));
        turret2.setPosition(powerToPosition(power));
    }

    public double getPosition() {
        return analogInput1.getVoltage() / 3.3 * 360;
    }

    public void changePIDF(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }
}
