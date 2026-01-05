package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public final DcMotorEx intake;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setVelocityPIDFCoefficients(0,0,0,0);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // constructor for JUnit
    public Intake(DcMotorEx intake) {
        this.intake = intake;
    }

//-------------------------------------------------------------------------------

    public void intakeOn() {
        intake.setPower(1);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void intakeReverse() {
        intake.setPower(-1);
    }

}



