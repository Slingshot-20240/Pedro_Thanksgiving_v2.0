package org.firstinspires.ftc.teamcode.teleop.misc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.logi;

@Disabled
@TeleOp
public class logiTest extends OpMode {

    public logi cam;

    @Override
    public void init() {
        cam = new logi(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("AT angle: ", cam.getTargetArtifactTravelDistanceX());
        telemetry.addData("AT distance", cam.getTargetArtifactTravelDistanceX());
        telemetry.update();
    }
}
