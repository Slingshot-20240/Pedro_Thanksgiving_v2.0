package org.firstinspires.ftc.teamcode.subsystems.transfer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferM {
    public final CRServo backTransfer;
    public final DcMotorEx frontTransfer;


    public TransferM(HardwareMap hardwareMap) {
        backTransfer = hardwareMap.get(CRServo.class, "transferB");
        frontTransfer = hardwareMap.get(DcMotorEx.class, "transferF");

        frontTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public TransferM(DcMotorEx frontTransfer, CRServo backTransfer) {
        this.frontTransfer = frontTransfer;
        this.backTransfer = backTransfer;
    }
// ------------------------------------------------------------------

    public void transferOn() {
        backTransfer.setPower(-1);
        frontTransfer.setPower(-1);
    }

    public void transferOff() {
        backTransfer.setPower(0);
        frontTransfer.setPower(0);
    }

    public void hotDog() {
        backTransfer.setPower(1);
        frontTransfer.setPower(-0.08);
    }

}

