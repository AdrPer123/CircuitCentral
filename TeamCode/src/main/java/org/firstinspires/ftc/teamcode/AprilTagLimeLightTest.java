package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Reports range and aim angle to BIOBUZZ AprilTags.
 *
 * No field localization here on purpose: BIOBUZZ tags are mounted on the CELLS and
 * move during a match, so they cannot anchor an absolute field position. Each tag
 * cluster's origin sits at the centre of the Cell opening, which makes the relative
 * pose below an aiming target rather than a landmark.
 */
@TeleOp(name = "Limelight AprilTag Aim", group = "Test")
public class AprilTagLimeLightTest extends OpMode {

    private static final int APRILTAG_PIPELINE = 8;

    /** Outtake height relative to the camera, in inches. Negative if it sits lower. */
    private static final double OUTTAKE_OFFSET_IN = 0.0;

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        telemetry.setMsTransmissionInterval(11);
        telemetry.addLine("Ready. Pipeline tag size must match the tag in use:");
        telemetry.addLine("  BIOBUZZ 3.25 in = 82.55 mm | DECODE 6.5 in = 165.1 mm");
    }

    @Override
    public void start() {
        super.start();
        limelight.start();
    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Limelight", "%s | pipeline %d (%s)",
                status.getName(), status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) {
            telemetry.addLine("no tag detected");
            return;
        }

        telemetry.addData("Target", "tx %.2f | ty %.2f | ta %.2f",
                result.getTx(), result.getTy(), result.getTa());

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            Pose3D tagPose = fiducial.getTargetPoseRobotSpace();
            Position position = tagPose.getPosition().toUnit(DistanceUnit.INCH);

            // Camera space is x = right, y = down, z = forward. Confirmed against the
            // camera's own output, where tx equals atan(x / z).
            double side = position.x;
            double vert = position.y;
            double forward = position.z;

            double range = Math.sqrt(side * side + vert * vert + forward * forward);
            double ground = Math.hypot(forward, side);
            double rise = -vert - OUTTAKE_OFFSET_IN;
            double aim = Math.toDegrees(Math.atan2(rise, ground));

            telemetry.addData("Tag " + fiducial.getFiducialId(),
                    "fwd %.1f | side %+.1f | vert %+.1f in | range %.1f in",
                    forward, side, vert, range);
            telemetry.addData("  aim", "tip %s %.1f deg | rise %+.1f in | ground %.1f in",
                    aim >= 0 ? "UP" : "DOWN", Math.abs(aim), rise, ground);

            YawPitchRollAngles facing = tagPose.getOrientation();
            telemetry.addData("  facing", "yaw %+.1f | pitch %+.1f | roll %+.1f deg",
                    facing.getYaw(AngleUnit.DEGREES),
                    facing.getPitch(AngleUnit.DEGREES),
                    facing.getRoll(AngleUnit.DEGREES));
        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
