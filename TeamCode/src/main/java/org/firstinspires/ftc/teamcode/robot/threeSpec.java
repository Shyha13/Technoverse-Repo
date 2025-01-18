package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "ThreeSpec")
public class threeSpec extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static double kP_o = 0.01;
    private DcMotor back;


    private int pathState;

    private final Pose startPose = new Pose(8, 63, Math.toRadians(180));

    private final Pose holdPre = new Pose(34.8, 63, Math.toRadians(180));
    private final Pose scorePre = new Pose(37.8, 63, Math.toRadians(180));

    private final Pose lineup1 = new Pose(60, 23, Math.toRadians(0));
    private final Pose push1 = new Pose(15, 23, Math.toRadians(0));

    private final Pose lineup2 = new Pose(60, 12.4, Math.toRadians(0));
    private final Pose push2 = new Pose(15, 12.4, Math.toRadians(0));

    private final Pose lineup3 = new Pose(60, 8.9, Math.toRadians(0));
    private final Pose push3 = new Pose(15, 8.9, Math.toRadians(0));

    private final Pose lineSpec = new Pose(14,26,Math.toRadians(0));
    private final Pose getSpec = new Pose(5.75, 26, Math.toRadians(0));

    private final Pose spec1 = new Pose(37.8, 61, Math.toRadians(180));
    private final Pose spec2 = new Pose(37.8, 65, Math.toRadians(180));
    private final Pose spec3 = new Pose(37.8, 67, Math.toRadians(180));
    private final Pose spec4 = new Pose(37.8, 69, Math.toRadians(180));

    private final Pose park = new Pose(10.2, 13, Math.toRadians(90));

    private Path preloadH, end;
    private PathChain preloadS, samp1L, samp1, samp2L, samp2, samp3L, samp3,line, get, hang1, line2, get2, hang2, line3, get3, hang3, line4, get4, hang4;

    public void buildPaths() {


        preloadH = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(holdPre)));
        preloadH.setLinearHeadingInterpolation(startPose.getHeading(), scorePre.getHeading());


        preloadS = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(holdPre),
                                new Point(scorePre)))
                .setLinearHeadingInterpolation(holdPre.getHeading(), scorePre.getHeading())
                .build();

        samp1L = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePre),
                                new Point(0.000, 29.000, Point.CARTESIAN),
                                new Point(79.600, 37.800, Point.CARTESIAN),
                                new Point(lineup1)))
                .setLinearHeadingInterpolation(scorePre.getHeading(), lineup1.getHeading())
                .build();

        samp1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(lineup1),
                                new Point(push1)))
                .setLinearHeadingInterpolation(lineup1.getHeading(), push1.getHeading())
                .build();

        samp2L = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(push1),
                                new Point(51.200, 36.700, Point.CARTESIAN),
                                new Point(lineup2)))
                .setLinearHeadingInterpolation(push1.getHeading(), lineup2.getHeading())
                .build();

        samp2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(lineup2),
                                new Point(push2)))
                .setLinearHeadingInterpolation(lineup2.getHeading(), push2.getHeading())
                .build();

        samp3L = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(push2),
                                new Point(40.900, 31.900, Point.CARTESIAN),
                                new Point(lineup3)))
                .setLinearHeadingInterpolation(push1.getHeading(), lineup2.getHeading())
                .build();

        samp3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(lineup3),
                                new Point(push3)))
                .setLinearHeadingInterpolation(lineup3.getHeading(), push3.getHeading())
                .build();


        line = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(push3),
                                new Point(29.500, 19.800, Point.CARTESIAN),
                                new Point(lineSpec)))
                .setLinearHeadingInterpolation(push3.getHeading(), lineSpec.getHeading())
                .build();

        get = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(lineSpec),
                                new Point(getSpec)))
                .setLinearHeadingInterpolation(lineSpec.getHeading(), getSpec.getHeading())
                .build();

        hang1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(getSpec),
                                new Point(15.500, 59.000, Point.CARTESIAN),
                                new Point(spec1)))
                .setLinearHeadingInterpolation(getSpec.getHeading(), spec1.getHeading())
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(spec1),
                                new Point(24.500, 35.800, Point.CARTESIAN),
                                new Point(lineSpec)))
                .setLinearHeadingInterpolation(spec1.getHeading(), lineSpec.getHeading())
                .build();
        get2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(lineSpec),
                                new Point(getSpec)))
                .setLinearHeadingInterpolation(lineSpec.getHeading(), getSpec.getHeading())
                .build();

        hang2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(getSpec),
                                new Point(15.500, 59.000, Point.CARTESIAN),
                                new Point(spec2)))
                .setLinearHeadingInterpolation(getSpec.getHeading(), spec2.getHeading())
                .build();

        line3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(spec2),
                                new Point(24.500, 35.800, Point.CARTESIAN),
                                new Point(lineSpec)))
                .setLinearHeadingInterpolation(spec2.getHeading(), lineSpec.getHeading())
                .build();
        get3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(lineSpec),
                                new Point(getSpec)))
                .setLinearHeadingInterpolation(lineSpec.getHeading(), getSpec.getHeading())
                .build();

        hang3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(getSpec),
                                new Point(15.500, 59.000, Point.CARTESIAN),
                                new Point(spec3)))
                .setLinearHeadingInterpolation(getSpec.getHeading(), spec3.getHeading())
                .build();

        line4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(spec2),
                                new Point(24.500, 35.800, Point.CARTESIAN),
                                new Point(lineSpec)))
                .setLinearHeadingInterpolation(spec3.getHeading(), lineSpec.getHeading())
                .build();

        get4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(lineSpec),
                                new Point(getSpec)))
                .setLinearHeadingInterpolation(lineSpec.getHeading(), getSpec.getHeading())
                .build();


        hang4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(getSpec),
                                new Point(15.500, 59.000, Point.CARTESIAN),
                                new Point(spec4)))
                .setLinearHeadingInterpolation(getSpec.getHeading(), spec4.getHeading())
                .build();

        end = new Path(
                new BezierLine(
                        new Point(spec4),
                        new Point(park)));
        end.setLinearHeadingInterpolation(spec4.getHeading(), park.getHeading());
    }

    public double outtakePid(double target, double current) {
        return (target - current) * kP_o;
    }

    public void outtake(int targetPosition) {
        back.setTargetPosition(targetPosition);
        back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back.setPower(outtakePid(targetPosition, back.getCurrentPosition()));
    }
    public void autonomousPathUpdate() {
        double delay = 0.5;
        double pause = 0.15;
        switch (pathState) {

            case 0:
                outtake(2600);
                follower.followPath(preloadH);
                setPathState(1,0);
                break;

            case 1: if(follower.atParametricEnd() && back.getCurrentPosition() >= 2575) {
                follower.followPath(preloadS,true);
                outtake(0);
                setPathState(2,0);
            }
                break;

            case 2:
                if(follower.atParametricEnd()) {
                    outtake(0);
                    follower.followPath(samp1L,true);
                    setPathState(3,0);
                }
                break;

            case 3:

                if(follower.atParametricEnd()) {
                    follower.followPath(samp1,true);
                    setPathState(4,0);
                }
                break;

            case 4:

                if(follower.atParametricEnd()) {
                    follower.followPath(samp2L,true);
                    setPathState(5,0);
                }
                break;

            case 5:

                if(follower.atParametricEnd()) {
                    follower.followPath(samp2,true);
                    setPathState(8,0);
                }
                break;

//            case 6:
//
//                if(follower.atParametricEnd()) {
//                    follower.followPath(samp3L,true);
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                if(follower.atParametricEnd()) {
//                    follower.followPath(samp3, true);
//                    setPathState(8);
//                }
//                break;

            case 8:
                if(follower.atParametricEnd()) {
                    follower.followPath(line,true);
                    setPathState(9, delay);
                }
            case 9:
                if(follower.atParametricEnd()&& pathTimer.getElapsedTime()>delay) {
                    follower.followPath(get,true);
                    setPathState(10,pause);
                }
                break;

            case 10:
                if(follower.atParametricEnd() && pathTimer.getElapsedTime() >pause ) {
                    outtake(2600);
                    follower.followPath(hang1,true);
                    setPathState(11,pause);
                }
                break;

            case 11:
                if(follower.atParametricEnd()&& pathTimer.getElapsedTime()>pause) {
                    outtake(0);
                    follower.followPath(line2,true);
                    setPathState(12,delay);
                }

            case 12:
                if(follower.atParametricEnd()&& pathTimer.getElapsedTime()>delay) {
                    follower.followPath(get2,true);
                    setPathState(13,pause);
                }
                break;

            case 13:
                if(follower.atParametricEnd()&& pathTimer.getElapsedTime()>pause) {
                    outtake(2600);
                    follower.followPath(hang2,true);
                    setPathState(20,pause);
                }
                break;
//            case 14:
//                if(follower.atParametricEnd()) {
//                    follower.followPath(line3,true);
//                    setPathState(15);
//                }
//
//            case 15:
//                if(follower.atParametricEnd()) {
//                    outtake(0);
//                    follower.followPath(get3,true);
//                    setPathState(16);
//                }
//                break;
//
//            case 16:
//                if(follower.atParametricEnd()) {
//                    outtake(2600);
//                    follower.followPath(hang3,true);
//                    setPathState(17);
//                }
//                break;
//
//            case 17:
//                if(follower.atParametricEnd()) {
//                    follower.followPath(line4,true);
//                    setPathState(18);
//                }
//
//            case 18:
//                if(follower.atParametricEnd()) {
//                    outtake(0);
//                    follower.followPath(get4,true);
//                    setPathState(19);
//                }
//                break;
//
//            case 19:
//                if(follower.atParametricEnd()) {
//                    outtake(2600);
//                    follower.followPath(hang4,true);
//                    setPathState(20);
//                }
//                break;
//

            case 20:
                if(follower.atParametricEnd()&& pathTimer.getElapsedTime()>pause) {
                    outtake(0);
                    follower.followPath(end,true);
                    setPathState(-1,0);
                }
                break;

        }
    }

    public void setPathState(int pState, double delay) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();


        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        back = hardwareMap.get(DcMotor.class, "back");
        back.setDirection(DcMotorSimple.Direction.REVERSE);
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

    }


    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0,0);
    }

    @Override
    public void stop() {
    }
}