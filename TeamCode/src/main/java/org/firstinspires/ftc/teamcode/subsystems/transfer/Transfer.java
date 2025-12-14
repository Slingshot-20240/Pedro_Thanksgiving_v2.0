package org.firstinspires.ftc.teamcode.subsystems.transfer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {
    public final CRServo backTransfer;
    public final CRServo frontTransfer;


    public Transfer(HardwareMap hardwareMap) {
        backTransfer = hardwareMap.get(CRServo.class, "transferB");
        frontTransfer = hardwareMap.get(CRServo.class, "transferF");

    }
// ------------------------------------------------------------------

    public void transferOn() {
        backTransfer.setPower(-1.0);
        frontTransfer.setPower(-1.0);
    }

    public void transferOff() {
        backTransfer.setPower(0);
        frontTransfer.setPower(0);
    }

    public void hotDog() {
        backTransfer.setPower(1);
        frontTransfer.setPower(-0.15);
    }

}

