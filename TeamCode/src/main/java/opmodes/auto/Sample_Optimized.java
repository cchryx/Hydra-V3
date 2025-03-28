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

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(0));

    private double beforePIckupTime = 1.5;

    private double sideSample = 0.72;
    private Path scorePreload, grabPreset1, grabPreset11, scorePreset1, grabPreset2, grabPreset22, scorePreset2, grabPreset3, grabPreset33, scorePreset3, park;

    public void buildPaths() {

        scorePreload = new Path(
                // Line 1
                new BezierCurve(
                        new Point(9.000, 111.000, Point.CARTESIAN),
                        new Point(13.275, 114.482, Point.CARTESIAN),
                        new Point(14.217, 132.317, Point.CARTESIAN)
                )
        );
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        grabPreset1 = new Path(
                // Line 2
                new BezierLine(
                        new Point(14.217, 132.317, Point.CARTESIAN),
                        new Point(37.443, 109.372, Point.CARTESIAN)
                )
        );
        grabPreset1.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));

        grabPreset11 = new Path(
                // Line 3
                new BezierLine(
                        new Point(37.443, 109.372, Point.CARTESIAN),
                        new Point(36.500, 124.000, Point.CARTESIAN)
                )
        );
        grabPreset11.setConstantHeadingInterpolation(Math.toRadians(0));

        scorePreset1 = new Path(
                // Line 4
                new BezierLine(
                        new Point(36.500, 124.000, Point.CARTESIAN),
                        new Point(17.032, 139.214, Point.CARTESIAN)
                )
        );
        scorePreset1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        grabPreset2 = new Path(
                // Line 5
                new BezierLine(
                        new Point(17.032, 139.214, Point.CARTESIAN),
                        new Point(37.724, 119.507, Point.CARTESIAN)
                )
        );
        grabPreset2.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));

        grabPreset22 = new Path(
                // Line 6
                new BezierLine(
                        new Point(37.724, 119.507, Point.CARTESIAN),
                        new Point(37.865, 138.000, Point.CARTESIAN)
                )
        );
        grabPreset22.setConstantHeadingInterpolation(Math.toRadians(0));

        scorePreset2 = new Path(
                // Line 7
                new BezierLine(
                        new Point(37.865, 138.000, Point.CARTESIAN),
                        new Point(16.047, 139.214, Point.CARTESIAN)
                )
        );
        scorePreset2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        grabPreset3 = new Path(
                // Line 8
                new BezierCurve(
                        new Point(16.047, 139.214, Point.CARTESIAN),
                        new Point(25.337, 136.540, Point.CARTESIAN),
                        new Point(42.000, 140.700, Point.CARTESIAN)
                )
        );
        grabPreset3.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(45));

        scorePreset3 = new Path(
                // Line 9
                new BezierLine(
                        new Point(42.000, 140.700, Point.CARTESIAN),
                        new Point(15.484, 138.933, Point.CARTESIAN)
                )
        );
        scorePreset3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        park = new Path(
                // Line 10
                new BezierCurve(
                        new Point(15.484, 138.933, Point.CARTESIAN),
                        new Point(76.997, 141.889, Point.CARTESIAN),
                        new Point(76.434, 97.830, Point.CARTESIAN)
                )
        );
        park.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90));
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

    }

    public void transfer4 (){
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
        outake.slidesTarget = Values.OUTSLIDES_MIN - 10;
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
                        if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.8) {
                            scoringClawOpen();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            outtakeHome1();

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
                            setActionState(99);
                            setPathState(2);
                        }
                        break;
                }
                break;
            case 2:
                switch (actionState) {
                    case 99:
                        follower.followPath(grabPreset1);
                        setActionState(888);
                        break;
                    case 888:
                        if (!follower.isBusy()) {
                            setActionState(777);
                        }
                        break;
                    case 777:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            follower.followPath(grabPreset11);
                            setActionState(0);
                        }
                        break;
                    case 0:
                        if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > beforePIckupTime) {
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
                            setActionState(4);
                        }
                        break;
                    case 4:
                        if (actionTimer.getElapsedTimeSeconds() > 0.2) {
                            transfer4();
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
                        if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                            scoringUp();
                            setActionState(1);
                        }
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
                        if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.8) {
                            scoringClawOpen();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            outtakeHome1();
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
                            setActionState(99);
                            setPathState(5);
                        }
                        break;
                }
                break;
            case 5:
                switch (actionState) {
                    case 99:
                        follower.followPath(grabPreset2);
                        setActionState(888);
                        break;
                    case 888:
                        if (!follower.isBusy()) {
                            setActionState(777);
                        }
                        break;
                    case 777:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            follower.followPath(grabPreset22);
                            setActionState(0);
                        }
                        break;
                    case 0:
                        if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > beforePIckupTime) {
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
                            setActionState(4);
                        }
                        break;
                    case 4:
                        if (actionTimer.getElapsedTimeSeconds() > 0.2) {
                            transfer4();
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
                        if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                            scoringUp();
                            setActionState(1);
                        }
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
                        if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.8) {
                            scoringClawOpen();
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            outtakeHome1();

                            intake.clawTarget = Values.CLAW_OPENED;
                            intake.pivotTarget = Values.INPIVOT_SUB;
                            intake.rotateTarget = Values.INROTATE_SUB;
                            intake.wristTarget = sideSample;
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
                            setActionState(99);
                            setPathState(8);
                        }
                        break;
                }
                break;

            case 8:
                switch (actionState) {
                    case 99:
                        follower.followPath(grabPreset3);
                        setActionState(0);
                        break;
                    case 0:
                        if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 1.7) {
                            intake.clawTarget = Values.CLAW_OPENED;
                            intake.pivotTarget = Values.INPIVOT_SUB_G;
                            intake.rotateTarget = Values.INROTATE_SUB_G;
                            intake.wristTarget = sideSample;
                            setActionState(1);
                        }
                        break;
                    case 1:
                        if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                            intake.clawTarget = Values.CLAW_CLOSED;
                            outake.rotateTarget = Values.OUTROTATE_TRANSFER;
                            outake.clawTarget = Values.CLAW_OPENED;
                            outake.pivotTarget = Values.OUTPIVOT_TRANSFER;
                            setActionState(100);
                        }
                        break;
                    case 100:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            intake.turretTarget = 0.2;
                            follower.followPath(scorePreset3);
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
                        if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                            transfer3();
                            setActionState(4);
                        }
                        break;
                    case 4:
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            transfer4();
                            setActionState(0);
                            setPathState(9);
                        }
                        break;
                }
                break;
            case 9:
                switch (actionState){
                    case 0:
                        if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                            scoringUp();
                            setActionState(1);
                        }
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
                        if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.8) {
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