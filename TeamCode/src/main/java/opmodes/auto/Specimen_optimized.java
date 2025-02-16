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

@Autonomous(name = "Specimen_5_Optimized", group = "Auto")
public class Specimen_optimized extends OpMode {
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

    private Path scorePreload, scorePreset1, grabPreset2, scorePreset2, grabPreset3, scorePreset3, grabPreset4, scorePreset4, park;
    private PathChain pushPresets;

    public void buildPaths() {

        scorePreload = new Path(
                // Line 1
                new BezierCurve(
                        new Point(9.000, 64.000, Point.CARTESIAN),
                        new Point(16.000, 60.000, Point.CARTESIAN),
                        new Point(44.000, 76.000, Point.CARTESIAN)
                )
        );
        scorePreload.setConstantHeadingInterpolation(heading);

        pushPresets = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(44.000, 76.000, Point.CARTESIAN),
                                new Point(30.000, 42.000, Point.CARTESIAN),
                                new Point(40.000, 36.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(40.000, 36.000, Point.CARTESIAN),
                                new Point(67.000, 32.000, Point.CARTESIAN),
                                new Point(67.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(67.000, 24.000, Point.CARTESIAN),
                                new Point(33.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(33.000, 24.000, Point.CARTESIAN),
                                new Point(69.000, 29.000, Point.CARTESIAN),
                                new Point(67.000, 17.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(67.000, 17.000, Point.CARTESIAN),
                                new Point(30.000, 17.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(30.000, 17.000, Point.CARTESIAN),
                                new Point(68.000, 20.000, Point.CARTESIAN),
                                new Point(67.000, 12.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(67.000, 12.000, Point.CARTESIAN),
                                new Point(18.000, 10.000, Point.CARTESIAN),
                                new Point(10.000, 22.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .build();

        scorePreset1 = new Path(
                // Line 9
                new BezierCurve(
                        new Point(10.000, 22.000, Point.CARTESIAN),
                        new Point(6.000, 60.000, Point.CARTESIAN),
                        new Point(50.000, 80.000, Point.CARTESIAN)
                )
        );
        scorePreset1.setConstantHeadingInterpolation(heading);

        grabPreset2 = new Path(
                // Line 10
                new BezierCurve(
                        new Point(50.000, 80.000, Point.CARTESIAN),
                        new Point(15.000, 60.000, Point.CARTESIAN),
                        new Point(9.000, 28.000, Point.CARTESIAN)
                )
        );
        grabPreset2.setConstantHeadingInterpolation(heading);

        scorePreset2 = new Path(
                // Line 11
                new BezierCurve(
                        new Point(9.000, 28.000, Point.CARTESIAN),
                        new Point(6.000, 60.000, Point.CARTESIAN),
                        new Point(50.000, 80.000, Point.CARTESIAN)
                )
        );
        scorePreset2.setConstantHeadingInterpolation(heading);

        grabPreset3 = new Path(
                // Line 12
                new BezierCurve(
                        new Point(50.000, 80.000, Point.CARTESIAN),
                        new Point(15.000, 60.000, Point.CARTESIAN),
                        new Point(9.000, 28.000, Point.CARTESIAN)
                )
        );
        grabPreset3.setConstantHeadingInterpolation(heading);

        scorePreset3 = new Path(
                // Line 13
                new BezierCurve(
                        new Point(9.000, 28.000, Point.CARTESIAN),
                        new Point(6.000, 60.000, Point.CARTESIAN),
                        new Point(50.000, 80.000, Point.CARTESIAN)
                )
        );
        scorePreset3.setConstantHeadingInterpolation(heading);

        grabPreset4 = new Path(
                // Line 14
                new BezierCurve(
                        new Point(50.000, 80.000, Point.CARTESIAN),
                        new Point(16.000, 60.000, Point.CARTESIAN),
                        new Point(9.000, 28.000, Point.CARTESIAN)
                )
        );
        grabPreset4.setConstantHeadingInterpolation(heading);

        scorePreset4 = new Path(
                // Line 15
                new BezierCurve(
                        new Point(9.000, 28.000, Point.CARTESIAN),
                        new Point(6.000, 60.000, Point.CARTESIAN),
                        new Point(50.000, 80.000, Point.CARTESIAN)
                )
        );
        scorePreset4.setConstantHeadingInterpolation(heading);

        park = new Path(
                // Line 16
                new BezierLine(
                        new Point(50.000, 80.000, Point.CARTESIAN),
                        new Point(30.000, 50.000, Point.CARTESIAN)
                )
        );
        park.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60));
    }

    public void scoringUp() {
        outake.slidesTarget = Values.OUTSLIDES_HCHAM;
        outake.wristTarget = Values.OUTWRIST_MAX;
        outake.rotateTarget = Values.OUTROTATE_HCHAM;
        outake.pivotTarget = Values.OUTPIVOT_HCHAM;
    }
    public void scoringDown() {
        outake.pivotTarget = Values.OUTPIVOT_HCHAM_S;
        outake.slidesTarget = outake.slidesPosition - 100;
    }
    public void scoringClawOpen() {
        outake.clawTarget = Values.CLAW_OPENED;
    }
    public void scoringHome() {
        outake.slidesTarget = Values.OUTSLIDES_MIN;
        outake.clawTarget = Values.CLAW_OPENED;
        outake.wristTarget = Values.OUTWRIST_GRAB;
        outake.rotateTarget = Values.OUTROTATE_GRAB;
        outake.pivotTarget = Values.OUTPIVOT_GRAB;
    }

    public void intakeHome() {
        outake.clawTarget = Values.CLAW_OPENED;
        outake.wristTarget = Values.OUTWRIST_GRAB;
        outake.rotateTarget = Values.OUTROTATE_GRAB;
        outake.pivotTarget = Values.OUTPIVOT_GRAB;
    }

    public void intakeUp() {
        outake.clawTarget = Values.CLAW_CLOSED;
        outake.slidesTarget = Values.OUTSLIDES_GRAB;
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
                            if(actionTimer.getElapsedTimeSeconds() > 0) {
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
                if(!follower.isBusy()) {
                    intakeHome();
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    switch (actionState) {
                        case 0:
                            intakeUp();
                            setActionState(1);
                            break;
                        case 1:
                            if (actionTimer.getElapsedTimeSeconds() > 0.5) {
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
                            if(actionTimer.getElapsedTimeSeconds() > 0) {
                                scoringClawOpen();
                                setActionState(2);
                            }
                            break;
                        case 2:
                            if(actionTimer.getElapsedTimeSeconds() > 0) {
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
                        intakeUp();
                        setActionState(1);
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            follower.followPath(scorePreset2);
                            scoringUp();
                            setActionState(0);
                            setPathState(6);
                        }
                        break;
                }
                break;
            case 6:
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
                            if(actionTimer.getElapsedTimeSeconds() > 0) {
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
                }
                break;
            case 7:
                switch (actionState) {
                    case 0:
                        intakeUp();
                        setActionState(1);
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
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
                            if(actionTimer.getElapsedTimeSeconds() > 0) {
                                scoringHome();
                                setActionState(3);
                            }
                            break;
                        case 3:
                            if(actionTimer.getElapsedTimeSeconds() > 0.1) {
                                setActionState(0);
                                follower.followPath(grabPreset4);
                                intakeHome();
                                setPathState(9);
                            }
                            break;
                    }
                }
                break;
            case 9:
                switch (actionState) {
                    case 0:
                        intakeUp();
                        setActionState(1);
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            follower.followPath(scorePreset4);
                            scoringUp();
                            setActionState(0);
                            setPathState(10);
                        }
                        break;
                }
                break;
            case 10:
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
                            if(actionTimer.getElapsedTimeSeconds() > 0) {
                                scoringHome();
                                setActionState(3);
                            }
                            break;
                        case 3:
                            if(actionTimer.getElapsedTimeSeconds() > 0.1) {
                                setActionState(0);
                                follower.followPath(park);
                                intakeHome();
                                setPathState(11);
                            }
                            break;
                    }
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
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

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        outake.telemetry();
        telemetry.addData("path state", pathState);
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
