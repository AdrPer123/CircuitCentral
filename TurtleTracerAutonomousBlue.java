/* ============================================================= *
 *                 Turtle Tracer — Auto-Generated                *
 *                                                               *
 *  Version: 2.2.1.                                              *
 *  Copyright (c) 2026 Matthew Allen                             *
 *                                                               *
 *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
 *  Changes will be overwritten when regenerated.                *
 * ============================================================= */

package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.turtletracerlib.pathing.NamedCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;

@Autonomous(name = "Turtle Tracer Autonomous", group = "Autonomous")
@Configurable // Panels
public class TurtleTracerAutonomous extends OpMode {

  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private ElapsedTime pathTimer; // Timer for path state machine
  private Paths paths; // Paths defined in the Paths class

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    // ...
    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);

    follower = PedroConstants.createFollower(hardwareMap);
    // Determine starting heading: prefer geometric heading when a path exists, otherwise fall back to explicit startPoint values
    follower.setStartingPose(new Pose(135.000, 72.000, Math.toRadians(0.000)));

    pathTimer = new ElapsedTime();
    paths = new Paths(follower); // Build paths
  }

  @Override
  public void loop() {
    follower.update(); // Update follower
    pathState = autonomousPathUpdate(); // Update autonomous state machine

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.update(telemetry);
  }

  public static class Paths {

    public PathChain ScorePreload;
    public PathChain ToGarden;
    public PathChain SweepGarden;
    public PathChain ScoreGarden;
    public PathChain ToRightFlower;
    public PathChain ToBottomFlower;
    public PathChain ScoreBottomFlower;
    public PathChain Park;

    public Paths(Follower follower) {
      ScorePreload = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(135.000, 72.000),

            new Pose(124.000, 72.000)
          )
        )
        .setConstantHeadingInterpolation(Math.toRadians(0))
        .addParametricCallback(
          0.000,
          NamedCommands.getCommand("launcherSpinUp")
        )
        .build();

      ToGarden = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(124.000, 72.000),
            new Pose(130.000, 104.000),
            new Pose(114.000, 133.000)
          )
        )
        .setConstantHeadingInterpolation(Math.toRadians(0))
        .addParametricCallback(0.800, NamedCommands.getCommand("intakeOn"))
        .build();

      SweepGarden = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(114.000, 133.000),

            new Pose(131.000, 133.000)
          )
        )
        .setConstantHeadingInterpolation(Math.toRadians(0))
        .build();

      ScoreGarden = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(131.000, 133.000),

            new Pose(122.000, 114.000)
          )
        )
        .setConstantHeadingInterpolation(Math.toRadians(0))
        .addParametricCallback(
          0.200,
          NamedCommands.getCommand("launcherSpinUp")
        )
        .build();

      ToRightFlower = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(122.000, 114.000),

            new Pose(127.000, 95.000)
          )
        )
        .setConstantHeadingInterpolation(Math.toRadians(0))
        .addParametricCallback(0.300, NamedCommands.getCommand("intakeOn"))
        .build();

      ToBottomFlower = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(127.000, 95.000),
            new Pose(128.000, 44.000),
            new Pose(114.000, 10.000),
            new Pose(83.000, 10.000)
          )
        )
        .setConstantHeadingInterpolation(Math.toRadians(0))
        .addParametricCallback(0.700, NamedCommands.getCommand("intakeOn"))
        .build();

      ScoreBottomFlower = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(83.000, 10.000),

            new Pose(118.000, 32.000)
          )
        )
        .setConstantHeadingInterpolation(Math.toRadians(0))
        .addParametricCallback(
          0.000,
          NamedCommands.getCommand("launcherSpinUp")
        )
        .build();

      Park = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(118.000, 32.000),

            new Pose(131.000, 40.000)
          )
        )
        .setConstantHeadingInterpolation(Math.toRadians(0))
        .build();
    }
  }

  public int autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        follower.followPath(paths.ScorePreload, true);
        setPathState(1);
        break;
      case 1:
        if (!follower.isBusy()) {
          setPathState(2);
        }
        break;
      case 2:
        setPathState(3);
        break;
      case 3:
        if (pathTimer.getMilliseconds() > 1200) {
          setPathState(4);
        }
        break;
      case 4:
        follower.followPath(paths.ToGarden, true);
        setPathState(5);
        break;
      case 5:
        if (!follower.isBusy()) {
          setPathState(6);
        }
        break;
      case 6:
        follower.followPath(paths.SweepGarden, true);
        setPathState(7);
        break;
      case 7:
        if (!follower.isBusy()) {
          setPathState(8);
        }
        break;
      case 8:
        setPathState(9);
        break;
      case 9:
        if (pathTimer.getMilliseconds() > 300) {
          setPathState(10);
        }
        break;
      case 10:
        follower.followPath(paths.ScoreGarden, true);
        setPathState(11);
        break;
      case 11:
        if (!follower.isBusy()) {
          setPathState(12);
        }
        break;
      case 12:
        setPathState(13);
        break;
      case 13:
        if (pathTimer.getMilliseconds() > 1200) {
          setPathState(14);
        }
        break;
      case 14:
        follower.followPath(paths.ToRightFlower, true);
        setPathState(15);
        break;
      case 15:
        if (!follower.isBusy()) {
          setPathState(16);
        }
        break;
      case 16:
        setPathState(17);
        break;
      case 17:
        if (pathTimer.getMilliseconds() > 2000) {
          setPathState(18);
        }
        break;
      case 18:
        follower.followPath(paths.ToBottomFlower, true);
        setPathState(19);
        break;
      case 19:
        if (!follower.isBusy()) {
          setPathState(20);
        }
        break;
      case 20:
        setPathState(21);
        break;
      case 21:
        if (pathTimer.getMilliseconds() > 800) {
          setPathState(22);
        }
        break;
      case 22:
        follower.followPath(paths.ScoreBottomFlower, true);
        setPathState(23);
        break;
      case 23:
        if (!follower.isBusy()) {
          setPathState(24);
        }
        break;
      case 24:
        setPathState(25);
        break;
      case 25:
        if (pathTimer.getMilliseconds() > 1200) {
          setPathState(26);
        }
        break;
      case 26:
        follower.followPath(paths.Park, true);
        setPathState(27);
        break;
      case 27:
        if (!follower.isBusy()) {
          setPathState(28);
        }
        break;
      case 28:
        requestOpModeStop();
        pathState = -1;
        break;
    }
    return pathState;
  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.reset();
  }
}
