//package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;
//
//import android.util.Size;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import dev.nextftc.core.subsystems.Subsystem;
//
//public class Loginf implements Subsystem {
//    public static final Loginf INSTANCE = new Loginf();
//    private Loginf() {}
//
//    private HardwareMap hardwareMap;
//    private AprilTagProcessor apriltagPipeline;
//    private VisionPortal portal;
//
//    public void init(HardwareMap hw) {
//        this.hardwareMap = hw;
//    }
//
//    @Override
//    public void initialize() {
//
//        apriltagPipeline = new AprilTagProcessor.Builder()
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .build();
//
//        portal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(apriltagPipeline)
//                .setCameraResolution(new Size(1920, 1080))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .setAutoStopLiveView(true)
//                .enableLiveView(true)
//                .build();
//
//        portal.setProcessorEnabled(apriltagPipeline, true);
//    }
//
//    @Override
//    public void periodic() {}
//
//    // ---------------- Public API ----------------
//
//    public void enableAT() {
//        portal.setProcessorEnabled(apriltagPipeline, true);
//    }
//
//    public void enableAT(boolean enabled) {
//        portal.setProcessorEnabled(apriltagPipeline, enabled);
//    }
//
//    public double getATdist() {
//        if (!portal.getProcessorEnabled(apriltagPipeline))
//            return 0.0;
//
//        for (AprilTagDetection detection : apriltagPipeline.getDetections()) {
//            if (detection.id == 20 || detection.id == 24) {
//                return detection.ftcPose.range;
//            }
//        }
//        return 0.0;
//    }
//
//    public double getATangle() {
//        if (!portal.getProcessorEnabled(apriltagPipeline))
//            return 0.0;
//
//        for (AprilTagDetection detection : apriltagPipeline.getDetections()) {
//            if (detection.id == 20 || detection.id == 24) {
//                return detection.ftcPose.bearing;
//            }
//        }
//        return 0.0;
//    }
//}
