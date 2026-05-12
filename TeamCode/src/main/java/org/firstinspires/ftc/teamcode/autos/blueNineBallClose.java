package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

@Autonomous(name = "blueNineBallClose", group = "Autonomous")
public class blueNineBallClose extends OpMode {
    public Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState = 1;
    private blueNineBallClose.Paths paths;
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private Kicker kicker;
    private TelemetryDebug telemetryDebug;

    @Override
    public void init () {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(27.033, 128, Math.toRadians(143)));

        paths = new Paths(follower); // Build paths


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetryDebug = new TelemetryDebug(telemetry);
        turret = new Turret(hardwareMap, false, telemetryDebug);
        shooter = new Shooter(hardwareMap, false, telemetryDebug);
        kicker = new Kicker(hardwareMap, telemetryDebug);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop () {
        autonomousPathUpdate();

        kicker.update();
        follower.update();

        for (TelemetryDebug.watcher w : telemetryDebug.watchers) {
            telemetry.addData(w.getName(), w.getValue());
        }
    }

    public void autonomousPathUpdate () {
        switch (pathState) {
            case 1:
                shooter.accelerate(3);
                follower.followPath(paths.Path1);
                setPathState(2);
                break;

            case 2: // WAIT FOR PATH 1
                shooter.accelerate(3);
                if (!follower.isBusy()) {
                    setPathState(25);
                }
                break;
            case 25:
                shooter.accelerate();
                turret.aim();
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(3);
                }
                break;
            case 3: // SHOOT THREE BALLS
                kicker.startSequence();
                setPathState(31); // Move to a "waiting" state immediately
                break;

            case 31: // WAIT FOR KICKER
                if (!kicker.isBusy()) {
                    setPathState(4); // Move to next path only after kicker is done
                }
                break;
            case 4: // 3. DO PATH 2
                follower.followPath(paths.Path2);
                setPathState(5);
                break;

            case 5: // WAIT FOR PATH 2
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6: // 4. INTAKE WHILE DOING PATH 3
                intake.intake();
                follower.followPath(paths.Path3);
                setPathState(7);
                break;

            case 7: // WAIT FOR PATH 3
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        intake.idle();
                        setPathState(8);
                    }
                }
                break;

            case 8: // 5. DO PATH 4
                shooter.accelerate(3);
                follower.followPath(paths.Path4);
                setPathState(9);
                break;

            case 9: // WAIT FOR PATH 4
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                shooter.accelerate();
                turret.aim();
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(11);
                }
                break;
            case 11: // 6. SHOOT THREE BALLS
                kicker.startSequence();
                setPathState(12);
                break;
            case 12:
                if (!kicker.isBusy()) {
                    setPathState(0);
                }
        }
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(27.039, 128.161),
                                    new Pose(54.476, 86.046)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))
                    .build();
            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(54.476, 86.046),
                                    new Pose(41.019, 82.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();
            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(41.019, 82.400),
                                    new Pose(17.908, 82.047)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(17.908, 82.047),
                                    new Pose(54.493, 85.750)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();
        }
    }

}
