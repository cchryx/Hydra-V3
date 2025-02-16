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

@Autonomous(name = "Sample_Optimized", group = "Auto")
public class Sample_Optimized extends OpMode {
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
    private final Pose startPose = new Pose(9, 85, Math.toRadians(0));

    private Path scorePreload, grabPreset1, scorePreset1, grabPreset2, scorePreset2, grabPreset3, scorePreset3, park;
    private PathChain pushPresets;

    public void buildPaths() {

        scorePreload = new Path(
                // Line 1
                new BezierCurve(
                        new Point(9.000, 85.000, Point.CARTESIAN),
                        new Point(12.000, 116.000, Point.CARTESIAN),
                        new Point(18.000, 125.000, Point.CARTESIAN)
                )
        );
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        grabPreset1 = new Path(
                // Line 2
                new BezierCurve(
                        new Point(18.000, 125.000, Point.CARTESIAN),
                        new Point(23.000, 120.000, Point.CARTESIAN),
                        new Point(35.000, 118.000, Point.CARTESIAN)
                )
        );
        grabPreset1.setConstantHeadingInterpolation(Math.toRadians(0));

        scorePreset1 = new Path(
                // Line 3
                new BezierCurve(
                        new Point(35.000, 118.000, Point.CARTESIAN),
                        new Point(22.232, 119.507, Point.CARTESIAN),
                        new Point(18.000, 125.000, Point.CARTESIAN)
                )
        );
        scorePreset1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        grabPreset2 = new Path(
                // Line 4
                new BezierCurve(
                        new Point(18.000, 125.000, Point.CARTESIAN),
                        new Point(26.503, 127.797, Point.CARTESIAN),
                        new Point(35.000, 130.000, Point.CARTESIAN)
                )
        );
        grabPreset2.setConstantHeadingInterpolation(Math.toRadians(0));

        scorePreset2 = new Path(
                // Line 5
                new BezierCurve(
                        new Point(35.000, 130.000, Point.CARTESIAN),
                        new Point(26.000, 128.174, Point.CARTESIAN),
                        new Point(20.000, 125.000, Point.CARTESIAN)
                )
        );
        scorePreset2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        grabPreset3 = new Path(
                // Line 6
                new BezierCurve(
                        new Point(20.000, 125.000, Point.CARTESIAN),
                        new Point(26.754, 126.792, Point.CARTESIAN),
                        new Point(40.000, 130.000, Point.CARTESIAN)
                )
        );
        grabPreset3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45));

        scorePreset3 = new Path(
                // Line 7
                new BezierCurve(
                        new Point(40.000, 130.000, Point.CARTESIAN),
                        new Point(29.517, 127.671, Point.CARTESIAN),
                        new Point(20.000, 125.000, Point.CARTESIAN)
                )
        );
        scorePreset3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        park = new Path(
                // Line 8
                new BezierCurve(
                        new Point(20.000, 125.000, Point.CARTESIAN),
                        new Point(47.228, 110.589, Point.CARTESIAN),
                        new Point(73.000, 90.000, Point.CARTESIAN)
                )
        );
        park.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90));
    }


    public void intake() {
        intake.clawTarget = Values.CLAW_OPENED;
        intake.pivotTarget = Values.INPIVOT_SUB;
        intake.rotateTarget = Values.INROTATE_SUB;
    }

    public void intakeDown() {
        intake.clawTarget = Values.CLAW_OPENED;
        intake.pivotTarget = Values.INPIVOT_SUB_G;
        intake.rotateTarget = Values.INROTATE_SUB_G;
    }

    public void transfer1 (){
        intake.clawTarget = Values.CLAW_CLOSED;

        outake.rotateTarget = Values.OUTROTATE_TRANSFER;
        outake.clawTarget = Values.CLAW_OPENED;
        outake.pivotTarget = Values.OUTPIVOT_TRANSFER;
    }

    public void transfer2 (){
        intake.wristTarget = Values.INWRIST_INIT;
        intake.rotateTarget = Values.INROTATE_INIT;
        intake.pivotTarget = Values.INPIVOT_TRANSFER;
        intake.turretTarget = Values.INTURRET_INIT;
        intake.slidesTarget = Values.HSLIDES_MIN;
    }

    public void transfer3 (){
        outake.clawTarget = Values.CLAW_CLOSED;
        intake.clawTarget = Values.CLAW_OPENED;
    }

    public void scoringUp() {
        outake.pivotTarget = Values.OUTPIVOT_HBASK;
        outake.slidesTarget = Values.OUTSLIDES_MAX;
        outake.wristTarget = Values.OUTWRIST_MAX;
    }

    public void scoringUp2(){
        outake.rotateTarget = Values.OUTROTATE_HBASK;
    }

    public void scoringClawOpen() {
        outake.clawTarget = Values.CLAW_OPENED;
    }

    public void outtakeHome1() {
        outake.pivotTarget = Values.OUTPIVOT_HBASK + 0.3;
    }

    public void outtakeHome2() {
        outake.clawTarget = Values.CLAW_OPENED;
        outake.wristTarget = Values.OUTWRIST_MAX;
        outake.rotateTarget = Values.OUTROTATE_TRANSFER;
        outake.pivotTarget = Values.OUTPIVOT_TRANSFER;
    }

    public void outtakeHome3() {
        outake.slidesTarget = Values.OUTSLIDES_MIN - 20;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                switch (actionState){
                    case 0:
                        scoringUp();
                        follower.followPath(scorePreload);
                        setActionState(1);
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                            scoringUp2();
                            setActionState(0);
                            setPathState(1);
                        }
                        break;
                }
                break;
            case 1:
                switch (actionState) {
                    case 0:
                        if(!follower.isBusy()) {
                            scoringClawOpen();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            outtakeHome1();
                            follower.followPath(grabPreset1);
                            intake();
                            setActionState(2);
                        }
                        break;
                    case 2:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5){
                            outtakeHome2();
                            setActionState(3);
                        }
                        break;
                    case 3:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5){
                            outtakeHome3();
                            setActionState(0);
                            setPathState(2);
                        }
                        break;
                }
                break;
            case 2:
                switch (actionState) {
                    case 0:
                        if(!follower.isBusy()) {
                            intakeDown();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                            transfer1();
                            setActionState(2);
                        }
                        break;
                    case 2:
                        if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                            transfer2();
                            setActionState(3);
                        }
                        break;
                    case 3:
                        if (actionTimer.getElapsedTimeSeconds() > 1) {
                            transfer3();
                            follower.followPath(scorePreset1);
                            setActionState(0);
                            setPathState(3);
                        }
                        break;

                }
                break;

            case 3:
                switch (actionState){
                    case 0:
                        scoringUp();
                        setActionState(1);
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                            scoringUp2();
                            setActionState(0);
                            setPathState(4);
                        }
                        break;
                }
                break;
            case 4:
                switch (actionState) {
                    case 0:
                        if(!follower.isBusy()) {
                            scoringClawOpen();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            outtakeHome1();
                            follower.followPath(grabPreset2);
                            intake();
                            setActionState(2);
                        }
                        break;
                    case 2:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5){
                            outtakeHome2();
                            setActionState(3);
                        }
                        break;
                    case 3:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5){
                            outtakeHome3();
                            setActionState(0);
                            setPathState(5);
                        }
                        break;
                }
                break;
            case 5:
                switch (actionState) {
                    case 0:
                        if(!follower.isBusy()) {
                            intakeDown();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                            transfer1();
                            setActionState(2);
                        }
                        break;
                    case 2:
                        if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                            transfer2();
                            setActionState(3);
                        }
                        break;
                    case 3:
                        if (actionTimer.getElapsedTimeSeconds() > 1) {
                            transfer3();
                            follower.followPath(scorePreset2);
                            setActionState(0);
                            setPathState(6);
                        }
                        break;

                }
                break;
            case 6:
                switch (actionState){
                    case 0:
                        scoringUp();
                        setActionState(1);
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                            scoringUp2();
                            setActionState(0);
                            setPathState(7);
                        }
                        break;
                }
                break;
            case 7:
                switch (actionState) {
                    case 0:
                        if(!follower.isBusy()) {
                            scoringClawOpen();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            outtakeHome1();
                            follower.followPath(grabPreset3);
                            intake();
                            setActionState(2);
                        }
                        break;
                    case 2:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5){
                            outtakeHome2();
                            setActionState(3);
                        }
                        break;
                    case 3:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5){
                            outtakeHome3();
                            setActionState(0);
                            setPathState(8);
                        }
                        break;
                }
                break;

            case 8:
                switch (actionState) {
                    case 0:
                        if(!follower.isBusy()) {
                            intakeDown();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                            transfer1();
                            setActionState(2);
                        }
                        break;
                    case 2:
                        if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                            transfer2();
                            setActionState(3);
                        }
                        break;
                    case 3:
                        if (actionTimer.getElapsedTimeSeconds() > 1) {
                            transfer3();
                            follower.followPath(scorePreset3);
                            setActionState(0);
                            setPathState(9);
                        }
                        break;
                }
                break;
            case 9:
                switch (actionState){
                    case 0:
                        scoringUp();
                        setActionState(1);
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                            scoringUp2();
                            setActionState(0);
                            setPathState(10);
                        }
                        break;
                }
                break;
            case 10:
                switch (actionState) {
                    case 0:
                        if(!follower.isBusy()) {
                            scoringClawOpen();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            outtakeHome1();
                            follower.followPath(park);
                            intake();
                            setActionState(2);
                        }
                        break;
                    case 2:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5){
                            outtakeHome2();
                            setActionState(3);
                        }
                        break;
                    case 3:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5){
                            outtakeHome3();
                            setActionState(0);
                            setPathState(-1);
                        }
                        break;
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
        intake.move("none");

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
