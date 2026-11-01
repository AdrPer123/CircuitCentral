package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing setup for the robot. Every auto calls Constants.createFollower(hardwareMap).
 *
 * The numbers below are STARTING VALUES. Run the tuning OpModes (TeleOp list -> "Tuning")
 * and paste your results here before trusting the auto. Guide: https://pedropathing.com
 */
public class Constants {
    // Robot mass in kg, plus the PIDF values the tuners give you.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10);

    // Motor names must match your Robot Controller configuration.
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    // Localizer: uses the drive motor encoders, so it works on any mecanum robot.
    // If you have a goBILDA Pinpoint, swap this for PinpointConstants + .pinpointLocalizer(...)
    // (much more accurate).
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .robotWidth(16)
            .robotLength(16);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                .build();
    }
}
