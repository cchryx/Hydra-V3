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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import components.HardwareInitializer;
import components.IntakeControl;
import components.OuttakeControl;
import components.Values;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Specimen_5_chris", group = "Auto")
public class Specimen_5_chris extends OpMode {
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

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final double heading = 0;
    private final Pose startPose = new Pose(9, 64, Math.toRadians(0));
    private final Pose score1Pose = new Pose(44, 76, Math.toRadians(0));
    private final Pose score1Ctrl1Pose = new Pose(16, 60, Math.toRadians(0));
    private final Pose move1_1Pose = new Pose(35.7, 17, Math.toRadians(0));
    private final Pose move1Ctrl1Pose = new Pose(28.42, 30, Math.toRadians(0));
    private final Pose move1_2Pose = new Pose(24, 17, Math.toRadians(0));
    private final Pose move2_1Pose = new Pose(35.7, 8, Math.toRadians(0));
    private final Pose move2_2Pose = new Pose(24, 8, Math.toRadians(0));


    private final Pose push2_1Pose = new Pose(32.2, 11.1, Math.toRadians(0));
    private final Pose push2_1Ctrl1Pose = new Pose(98.9, 12.25, Math.toRadians(0));
//    private final Pose push2_2Pose = new Pose(30, 10.5, Math.toRadians(0));
    private final Pose push3_1Pose = new Pose(32, 0.9, Math.toRadians(0));
    private final Pose push3_1Ctrl1Pose = new Pose(97, 0.9, Math.toRadians(0));
//    private final Pose push3_2Pose = new Pose(30, 2.5, Math.toRadians(0));

    // Grab pose can be reused
    private final Pose grabPose = new Pose(12, 30, Math.toRadians(0));
    private final Pose grabCtrl1Pose = new Pose(20.8, 34.31, Math.toRadians(0));
    private final Pose score2Pose = new Pose(44, 74, Math.toRadians(0));
    private final Pose score3Pose = new Pose(44, 72, Math.toRadians(0));
    private final Pose score4Pose = new Pose(44, 70, Math.toRadians(0));
    private final Pose score5Pose = new Pose(44, 68, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload,
            movePreset1_1, movePreset1_2,
            movePreset2_1, movePreset2_2,
            grabPreset1, scorePreset1, grabPreset2, scorePreset2, grabPreset3, scorePreset3, grabPreset4, scorePreset4;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire
         *      path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path
         * is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html>
         */


        scorePreload = new Path(
                new BezierCurve(
                    new Point(startPose),
                    new Point(score1Ctrl1Pose),
                    new Point(score1Pose)
                )
        );
        scorePreload.setConstantHeadingInterpolation(heading);

        movePreset1_1 = new Path(
                new BezierCurve(
                        new Point(score1Pose),
                        new Point(move1Ctrl1Pose),
                        new Point(move1_1Pose)
                )
        );
        movePreset1_1.setConstantHeadingInterpolation(heading);

        movePreset1_2 = new Path(
                new BezierLine(
                        new Point(move1_1Pose),
                        new Point(move1_2Pose)
                )
        );
        movePreset1_2.setConstantHeadingInterpolation(heading);

        movePreset2_1 = new Path(
                new BezierLine(
                        new Point(move1_2Pose),
                        new Point(move2_1Pose)
                )
        );
        movePreset2_1.setConstantHeadingInterpolation(heading);

        movePreset2_2 = new Path(
                new BezierLine(
                        new Point(move2_1Pose),
                        new Point(move2_2Pose)
                )
        );
        movePreset2_2.setConstantHeadingInterpolation(heading);


        grabPreset1 = new Path(
                new BezierLine(
                        new Point(push3_1Pose),
                        new Point(grabPose)
                )
        );
        grabPreset1.setConstantHeadingInterpolation(heading);

        scorePreset1 = new Path(
                new BezierLine(
                        new Point(grabPose),
                        new Point(score2Pose)
                )
        );
        scorePreset1.setConstantHeadingInterpolation(heading);

        grabPreset2 = new Path(
                new BezierCurve(
                        new Point(score2Pose),
                        new Point(grabCtrl1Pose),
                        new Point(grabPose)
                )
        );
        grabPreset2.setConstantHeadingInterpolation(heading);

        scorePreset2 = new Path(
                new BezierLine(
                        new Point(grabPose),
                        new Point(score3Pose)
                )
        );
        scorePreset2.setConstantHeadingInterpolation(heading);

        grabPreset3 = new Path(
                new BezierCurve(
                        new Point(score3Pose),
                        new Point(grabCtrl1Pose),
                        new Point(grabPose)
                )
        );
        grabPreset3.setConstantHeadingInterpolation(heading);

        scorePreset3 = new Path(
                new BezierLine(
                        new Point(grabPose),
                        new Point(score4Pose)
                )
        );
        scorePreset3.setConstantHeadingInterpolation(heading);

        grabPreset4 = new Path(
                new BezierCurve(
                        new Point(score4Pose),
                        new Point(grabCtrl1Pose),
                        new Point(grabPose)
                )
        );
        grabPreset4.setConstantHeadingInterpolation(heading);

        scorePreset4 = new Path(
                new BezierLine(
                        new Point(grabPose),
                        new Point(score5Pose)
                )
        );
        scorePreset4.setConstantHeadingInterpolation(heading);
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                outake.slidesTarget = Values.OUTSLIDES_HCHAM;
                outake.wristTarget = Values.OUTWRIST_MAX;
                outake.rotateTarget = Values.OUTROTATE_HCHAM;
                outake.pivotTarget = Values.OUTPIVOT_HCHAM;

                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    switch (actionState) {
                        case 0:
                            outake.pivotTarget = Values.OUTPIVOT_HCHAM_S;
                            outake.slidesTarget = outake.slidesPosition - 100;

                            setActionState(1);
                            break;
                        case 1:
                            if(actionTimer.getElapsedTimeSeconds() > 0.2) {
                                outake.clawTarget = Values.CLAW_OPENED;

                                setActionState(2);
                            }
                            break;
                        case 2:
                            if(actionTimer.getElapsedTimeSeconds() > 0.1) {
                                outake.slidesTarget = Values.OUTSLIDES_MIN;
                                outake.clawTarget = Values.CLAW_OPENED;
                                outake.wristTarget = Values.OUTWRIST_GRAB;
                                outake.rotateTarget = Values.OUTROTATE_GRAB + 0.1;
                                outake.pivotTarget = Values.OUTPIVOT_GRAB;

                                setActionState(3);
                            }
                            break;
                        case 3:
                            if(actionTimer.getElapsedTimeSeconds() > 0.3) {
                                intake.clawTarget = Values.CLAW_OPENED;
                                intake.pivotTarget = Values.INPIVOT_SUB;
                                intake.rotateTarget = Values.INROTATE_SUB;

                                setActionState(0);
                                actionProcess = "none";

                                follower.followPath(movePreset1_1, true);
                                setPathState(2);
                            }
                            break;
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    switch (actionState) {
                        case 0:
                            intake.clawTarget = Values.CLAW_OPENED;
                            intake.pivotTarget = Values.INPIVOT_SUB_G;
                            intake.rotateTarget = Values.INROTATE_SUB_G;

                            setActionState(1);
                            break;
                        case 1:
                            if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                                intake.clawTarget = Values.CLAW_CLOSED;

                                outake.rotateTarget = Values.OUTROTATE_TRANSFER;
                                outake.clawTarget = Values.CLAW_OPENED;
                                outake.pivotTarget = Values.OUTPIVOT_TRANSFER;

                                setActionState(2);
                            }
                            break;
                        case 2:
                            if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                                intake.wristTarget = Values.INWRIST_INIT;
                                intake.rotateTarget = Values.INROTATE_INIT;
                                intake.pivotTarget = Values.INPIVOT_TRANSFER;
                                intake.turretTarget = Values.INTURRET_INIT;
                                intake.slidesTarget = Values.HSLIDES_MIN;

                                setActionState(3);
                            }
                            break;
                        case 3:
                            if (actionTimer.getElapsedTimeSeconds() > 1) {
                                outake.clawTarget = Values.CLAW_CLOSED;
                                intake.clawTarget = Values.CLAW_OPENED;

                                setActionState(4);
                            }
                            break;
                        case 4:
                            if(actionTimer.getElapsedTimeSeconds() > 0.3) {
                                outake.wristTarget = Values.OUTWRIST_GRAB;
                                outake.rotateTarget = Values.OUTROTATE_GRAB;
                                outake.pivotTarget = Values.OUTPIVOT_GRAB;

                                intake.pivotTarget = Values.INPIVOT_SUB;
                                intake.rotateTarget = Values.INROTATE_SUB;

                                setActionState(0);
                                actionProcess = "none";

                                follower.followPath(movePreset1_2, true);
                                setPathState(3);
                            }
                            break;

                    }


                }
                break;
            case 3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outake.clawTarget = Values.CLAW_OPENED;

                    follower.followPath(movePreset2_1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(movePreset2_2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outake.clawTarget = Values.CLAW_OPENED;

                    setPathState(-1);
                }
                break;
//            case 6:
//                if(!follower.isBusy()) {
//                    follower.followPath(grabPreset3);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if(!follower.isBusy()) {
//                    follower.followPath(scorePreset3);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if(!follower.isBusy()) {
//                    follower.followPath(grabPreset4);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if(!follower.isBusy()) {
//                    follower.followPath(scorePreset4);
//                    setPathState(-1);
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
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
