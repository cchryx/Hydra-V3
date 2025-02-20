package opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import components.HardwareInitializer;
import components.IntakeControl;
import components.OuttakeControl;
import components.Values;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Specimen_4", group = "Auto")
public class Specimen_4 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private HardwareInitializer hardwareInitializer;

    private OuttakeControl outake;
    private IntakeControl intake;

    DcMotor VSLIDES_L, VSLIDES_R;
    Servo OUTPIVOT_L, OUTPIVOT_R, OUTROTATE, OUTWRIST, OUTCLAW, INTURRET, INPIVOT, INROTATE, INWRIST, INCLAW, HSLIDES_F, HSLIDES_B;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState, actionState;
    private String actionProcess = "none";

    /** Start Pose of our robot */
    private final double heading = 0;
    private final Pose startPose = new Pose(9, 64, Math.toRadians(0));

    public double waitPickup = 0.25;

    private Path scorePreload, pushPreset1, pushPreset2, pushPreset3, align1, align2, align3, align4, scorePreset1, grabPreset2, scorePreset2, grabPreset3, scorePreset3, grabPreset4, scorePreset4, park;
    private PathChain pushPresets;

    public void buildPaths() {

        scorePreload = new Path(
                // Line 1
                new BezierCurve(
                        new Point(9.000, 64.000, Point.CARTESIAN),
                        new Point(13.232, 73.196, Point.CARTESIAN),
                        new Point(40.540, 77.138, Point.CARTESIAN)
                )
        );
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(0));

        pushPresets = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(40.540, 77.138, Point.CARTESIAN),
                                new Point(24.774, 42.370, Point.CARTESIAN),
                                new Point(47.155, 32.094, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(47.155, 32.094, Point.CARTESIAN),
                                new Point(56.023, 33.783, Point.CARTESIAN),
                                new Point(71.226, 23.648, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(71.226, 23.648, Point.CARTESIAN),
                                new Point(33.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(33.000, 24.000, Point.CARTESIAN),
                                new Point(61.513, 30.968, Point.CARTESIAN),
                                new Point(71.226, 21.255, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(71.226, 21.255, Point.CARTESIAN),
                                new Point(30.264, 20.270, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(30.264, 20.270, Point.CARTESIAN),
                                new Point(67.566, 28.152, Point.CARTESIAN),
                                new Point(73.196, 19.144, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(73.196, 19.144, Point.CARTESIAN),
                                new Point(17.032, 15.906, Point.CARTESIAN),
                                new Point(29.560, 31.812, Point.CARTESIAN),
                                new Point(16.891, 47.015, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        align1 = new Path(
                // Line 9
                new BezierLine(
                        new Point(16.891, 47.015, Point.CARTESIAN),
                        new Point(10.276, 46.874, Point.CARTESIAN)
                )

        );
        align1.setConstantHeadingInterpolation(Math.toRadians(0));

        scorePreset1 = new Path(
                // Line 10
                new BezierCurve(
                        new Point(10.276, 46.874, Point.CARTESIAN),
                        new Point(28.293, 44.762, Point.CARTESIAN),
                        new Point(13.372, 82.205, Point.CARTESIAN),
                        new Point(41.525, 105.572, Point.CARTESIAN)
                )
        );
        scorePreset1.setConstantHeadingInterpolation(Math.toRadians(0));

        grabPreset2 = new Path(
                // Line 11
                new BezierCurve(
                        new Point(41.525, 105.572, Point.CARTESIAN),
                        new Point(25.619, 81.079, Point.CARTESIAN),
                        new Point(33.783, 47.718, Point.CARTESIAN),
                        new Point(16.328, 49.548, Point.CARTESIAN)
                )
        );
        grabPreset2.setConstantHeadingInterpolation(Math.toRadians(4));

        align2 = new Path(
                // Line 12
                new BezierLine(
                        new Point(16.328, 49.548, Point.CARTESIAN),
                        new Point(10.276, 49.548, Point.CARTESIAN)
                )
        );
        align2.setConstantHeadingInterpolation(Math.toRadians(4));

        scorePreset2 = new Path(
                // Line 13
                new BezierCurve(
                        new Point(10.276, 49.548, Point.CARTESIAN),
                        new Point(28.856, 44.481, Point.CARTESIAN),
                        new Point(13.935, 82.768, Point.CARTESIAN),
                        new Point(41.666, 103.601, Point.CARTESIAN)
                )
        );
        scorePreset2.setConstantHeadingInterpolation(Math.toRadians(5));

        grabPreset3 = new Path(
                // Line 14
                new BezierCurve(
                        new Point(41.666, 103.601, Point.CARTESIAN),
                        new Point(24.915, 80.798, Point.CARTESIAN),
                        new Point(33.220, 47.859, Point.CARTESIAN),
                        new Point(16.328, 49.830, Point.CARTESIAN)
                )
        );
        grabPreset3.setConstantHeadingInterpolation(Math.toRadians(5));

        align3 = new Path(
                // Line 15
                new BezierLine(
                        new Point(16.328, 49.830, Point.CARTESIAN),
                        new Point(10.276, 49.548, Point.CARTESIAN)
                )
        );
        align3.setConstantHeadingInterpolation(Math.toRadians(7));

        scorePreset3 = new Path(
                // Line 16
                new BezierCurve(
                        new Point(10.276, 49.548, Point.CARTESIAN),
                        new Point(28.716, 44.622, Point.CARTESIAN),
                        new Point(13.232, 82.065, Point.CARTESIAN),
                        new Point(41.384, 101.490, Point.CARTESIAN)
                )
        );
        scorePreset3.setConstantHeadingInterpolation(Math.toRadians(7));

        grabPreset4 = new Path(
                // Line 17
                new BezierCurve(
                        new Point(41.384, 101.490, Point.CARTESIAN),
                        new Point(25.337, 81.220, Point.CARTESIAN),
                        new Point(33.783, 47.437, Point.CARTESIAN),
                        new Point(16.610, 50.111, Point.CARTESIAN)
                )
        );
        grabPreset4.setConstantHeadingInterpolation(Math.toRadians(7));

//        align4 = new Path(
//                // Line 18
//                new BezierLine(
//                        new Point(16.751, 49.548, Point.CARTESIAN),
//                        new Point(9.009, 48.563, Point.CARTESIAN)
//                )
//        );
//        align4.setConstantHeadingInterpolation(Math.toRadians(7));
//
//        park = new Path(
//                // Line 19
//                new BezierLine(
//                        new Point(9.009, 48.563, Point.CARTESIAN),
//                        new Point(14.639, 53.208, Point.CARTESIAN)
//                )
//        );
//        park.setLinearHeadingInterpolation(Math.toRadians(7), Math.toRadians(45));

    }

    public int intakeOffset = 85;


    public void scoringUp() {
        outake.slidesTarget = Values.OUTSLIDES_HCHAM;
        outake.wristTarget = Values.OUTWRIST_MAX;
        outake.rotateTarget = Values.OUTROTATE_HCHAM;
        outake.pivotTarget = Values.OUTPIVOT_HCHAM;
    }
    public void scoringDown() {
        outake.pivotTarget = 0.85;
        outake.slidesTarget = outake.slidesPosition - 100;
    }
    public void scoringClawOpen() {
        outake.clawTarget = Values.CLAW_OPENED;
    }
    public void scoringHome() {
        outake.slidesTarget = Values.OUTSLIDES_MIN + intakeOffset;
        outake.clawTarget = Values.CLAW_OPENED;
        outake.wristTarget = Values.OUTWRIST_GRAB;
        outake.rotateTarget = Values.OUTROTATE_GRAB;
        outake.pivotTarget = Values.OUTPIVOT_GRAB;
    }

    public void intakeHome() {
        outake.clawTarget = Values.CLAW_OPENED;
        outake.wristTarget = Values.OUTWRIST_GRAB;
        outake.rotateTarget = Values.OUTROTATE_GRAB - 0.2;
        outake.pivotTarget = Values.OUTPIVOT_GRAB - 0.11;
    }

    public void intakeUp1() {
        outake.clawTarget = Values.CLAW_CLOSED;

    }

    public void intakeUp2() {
        outake.slidesTarget = Values.OUTSLIDES_GRAB + intakeOffset;
    }

    public void intakeExtend() {
        intake.wristTarget = Values.INWRIST_INIT;
        intake.clawTarget = Values.CLAW_OPENED;
        intake.turretTarget = Values.INTURRET_INIT;
        intake.pivotTarget = Values.INPIVOT_SUB;
        intake.rotateTarget = Values.INROTATE_SUB;
        intake.slidesTarget = Values.HSLIDES_MAX;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                scoringUp();
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    switch (actionState) {
                        case 0:
                            scoringDown();
                            setActionState(1);
                            break;
                        case 1:
                            if(actionTimer.getElapsedTimeSeconds() > 0) {
                                scoringClawOpen();
                                setActionState(2);
                            }
                            break;
                        case 2:
                            if(actionTimer.getElapsedTimeSeconds() > 0.2) {
                                scoringHome();
                                setActionState(3);
                            }
                            break;
                        case 3:
                            if(actionTimer.getElapsedTimeSeconds() > 0.1) {
                                setActionState(0);
                                follower.followPath(pushPresets);
                                setPathState(2);
                            }
                            break;
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(align1);
                    intakeHome();
                    setActionState(0);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    switch (actionState) {
                        case 0:
                            if (actionTimer.getElapsedTimeSeconds() > waitPickup) {
                                intakeUp1();
                                setActionState(1);
                            }
                            break;
                        case 1:
                            if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                                intakeUp2();
                            }
                            if (actionTimer.getElapsedTimeSeconds() > 0.7) {
                                follower.followPath(scorePreset1);
                                scoringUp();
                                setActionState(0);
                                setPathState(4);
                            }
                            break;
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    switch (actionState) {
                        case 0:
                            scoringDown();
                            setActionState(1);
                            break;
                        case 1:
                            if(actionTimer.getElapsedTimeSeconds() > 0.1) {
                                scoringClawOpen();
                                setActionState(2);
                            }
                            break;
                        case 2:
                            if(actionTimer.getElapsedTimeSeconds() > 0.2) {
                                scoringHome();
                                setActionState(3);
                            }
                            break;
                        case 3:
                            if(actionTimer.getElapsedTimeSeconds() > 0.1) {
                                setActionState(0);
                                follower.followPath(grabPreset2);
                                intakeHome();
                                setPathState(5);
                            }
                            break;
                    }
                }
                break;
            case 5:
                switch (actionState) {
                    case 0:
                        if (!follower.isBusy()) {
                            follower.followPath(align2);
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > waitPickup){
                            intakeUp1();
                            setActionState(2);
                        }
                        break;
                    case 2:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            intakeUp2();
                        }
                        if (actionTimer.getElapsedTimeSeconds() > 0.7) {
                            follower.followPath(scorePreset2);
                            scoringUp();
                            setActionState(0);
                            setPathState(6);
                        }
                        break;

                }
                break;
            case 6:
                switch (actionState) {
                    case 0:
                        if(!follower.isBusy()) {
                            scoringDown();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if(actionTimer.getElapsedTimeSeconds() > 0) {
                            scoringClawOpen();
                            setActionState(2);
                        }
                        break;
                    case 2:
                        if(actionTimer.getElapsedTimeSeconds() > 0.2) {
                            scoringHome();
                            setActionState(3);
                        }
                        break;
                    case 3:
                        if(actionTimer.getElapsedTimeSeconds() > 0.1) {
                            setActionState(0);
                            follower.followPath(grabPreset3);
                            intakeHome();
                            setPathState(7);
                        }
                        break;
                }
                break;
            case 7:
                switch (actionState) {
                    case 0:
                        if (!follower.isBusy()) {
                            follower.followPath(align3);
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > waitPickup){
                            intakeUp1();
                            setActionState(2);
                        }
                        break;
                    case 2:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            intakeUp2();
                        }
                        if (actionTimer.getElapsedTimeSeconds() > 0.7) {
                            follower.followPath(scorePreset3);
                            scoringUp();
                            setActionState(0);
                            setPathState(8);
                        }
                        break;

                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    switch (actionState) {
                        case 0:
                            if(!follower.isBusy()) {
                                scoringDown();
                                setActionState(1);
                            }
                            break;
                        case 1:
                            if(actionTimer.getElapsedTimeSeconds() > 0) {
                                scoringClawOpen();
                                setActionState(2);
                            }
                            break;
                        case 2:
                            if(actionTimer.getElapsedTimeSeconds() > 0.2) {
                                scoringHome();
                                setActionState(3);
                            }
                            break;
                        case 3:
                            if(actionTimer.getElapsedTimeSeconds() > 0.1) {
                                setActionState(0);
                                follower.followPath(grabPreset4);
                                intakeHome();
//                                setPathState(-1);
                            }
                            break;
                    }
                }
                break;
//            case 9:
//                switch (actionState) {
//                    case 0:
//                        if (!follower.isBusy()) {
//                            follower.followPath(align4);
//                            setActionState(1);
//                        }
//                        break;
//                    case 1:
//                        if (actionTimer.getElapsedTimeSeconds() > waitPickup){
//                            intakeUp1();
//                            setActionState(2);
//                        }
//                        break;
//                    case 2:
//                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
//                            intakeUp2();
//                        }
//                        if (actionTimer.getElapsedTimeSeconds() > 0.7) {
//                            follower.followPath(park);
//
//                            outake.slidesTarget = Values.OUTSLIDES_MIN;
//                            outake.clawTarget = Values.CLAW_CLOSED;
//                            outake.wristTarget = Values.OUTWRIST_GRAB;
//                            outake.rotateTarget = Values.OUTROTATE_GRAB;
//                            outake.pivotTarget = Values.OUTPIVOT_GRAB;
//                            setActionState(0);
//                            setPathState(-1);
//                        }
//                        break;
//                }
//                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setActionState(int pAction) {
        actionState = pAction;
        actionTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        outake.move();
        intake.move("none");

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        outake.telemetry();
        telemetry.addData("path state", pathState);
        telemetry.addData("action state", actionState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        //Test this
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize hardware
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        VSLIDES_L = hardwareInitializer.getMotor("VSLIDESL");
        VSLIDES_R = hardwareInitializer.getMotor("VSLIDESR");
        OUTPIVOT_L = hardwareInitializer.getServo("OUTPIVOTL");
        OUTPIVOT_R = hardwareInitializer.getServo("OUTPIVOTR");
        OUTROTATE = hardwareInitializer.getServo("OUTROTATE");
        OUTWRIST = hardwareInitializer.getServo("OUTWRIST");
        OUTCLAW = hardwareInitializer.getServo("OUTCLAW");

        INTURRET = hardwareInitializer.getServo("INTURRET");
        INPIVOT = hardwareInitializer.getServo("INPIVOT");
        INROTATE = hardwareInitializer.getServo("INROTATE");
        INWRIST = hardwareInitializer.getServo("INWRIST");
        INCLAW = hardwareInitializer.getServo("INCLAW");
        HSLIDES_F = hardwareInitializer.getServo("HSLIDESF");
        HSLIDES_B = hardwareInitializer.getServo("HSLIDESB");

        outake = new OuttakeControl(
                OUTPIVOT_L,
                OUTPIVOT_R,
                OUTROTATE,
                OUTWRIST,
                OUTCLAW,
                VSLIDES_L,
                VSLIDES_R,
                gamepad1,
                gamepad2,
                telemetry
        );

        intake = new IntakeControl(
                INTURRET,
                INPIVOT,
                INROTATE,
                INWRIST,
                INCLAW,
                HSLIDES_F,
                HSLIDES_B,
                gamepad1,
                gamepad2,
                telemetry
        );

        outake.init();
        intake.init();

        actionTimer = new Timer();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        // Feedback to Driver Hub
        outake.telemetry();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setActionState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
