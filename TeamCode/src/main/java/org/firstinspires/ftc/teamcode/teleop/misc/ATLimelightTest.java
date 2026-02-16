package org.firstinspires.ftc.teamcode.teleop.misc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.vision.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.subsystems.vision.PythonLimelight;

import dev.nextftc.core.units.Angle;

@Disabled
@Config
@TeleOp(group = "vision tests")
public class ATLimelightTest extends OpMode {

    public AprilTagLimelight limelight;
    GoBildaPinpointDriver pinpoint;
    @Override
    public void init() {
        limelight = new AprilTagLimelight(hardwareMap, true, telemetry);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));


    }

    @Override
    public void loop() {
        telemetry.addData("limelight angle ", Math.toDegrees(limelight.getAngle()));
        telemetry.addData("limelight dist ", Math.toDegrees(limelight.getDistance()));
        telemetry.addData("limelight botpose 1 ", (limelight.getBotPose()));
        telemetry.addData("limelight botpose 2 ", (limelight.getBotPose(pinpoint.getHeading(AngleUnit.DEGREES))));
        telemetry.addData("pinpoint ", pinpoint.getPosition());
        telemetry.update();
    }


    public void configurePinpoint(){
        pinpoint.setOffsets( -5.1, -4, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }
}
