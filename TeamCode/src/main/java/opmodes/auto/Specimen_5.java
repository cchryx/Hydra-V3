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
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Actual", group = "Specimen_5")
public class Specimen_5 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final double heading = 315;
    private final Pose startPose = new Pose(0, 65.5, Math.toRadians(315));
    private final Pose score1Pose = new Pose(28, 84, Math.toRadians(315));
    private final Pose move1Pose = new Pose(40, 43, Math.toRadians(315));
    private final Pose move1Ctrl1Pose = new Pose(14, 44, Math.toRadians(315));
    private final Pose push1_1Pose = new Pose(60, 40, Math.toRadians(315));
    private final Pose push1_1Ctrl1Pose = new Pose(14, 44, Math.toRadians(315));
    private final Pose push1_2Pose = new Pose(5, 34, Math.toRadians(315));
    private final Pose push2_1Pose = new Pose(50, 26, Math.toRadians(315));
    private final Pose push2_1Ctrl1Pose = new Pose(55, 36, Math.toRadians(315));
    private final Pose push2_2Pose = new Pose(5, 26, Math.toRadians(315));
    private final Pose push3_1Pose = new Pose(50, 16, Math.toRadians(315));
    private final Pose push3_1Ctrl1Pose = new Pose(60, 22, Math.toRadians(315));
    private final Pose push3_2Pose = new Pose(5, 16, Math.toRadians(315));

    // Grab pose can be reused
    private final Pose grabPose = new Pose(1, 33, Math.toRadians(315));
    private final Pose grabCtrl1Pose = new Pose(8, 34, Math.toRadians(315));
    private final Pose score2Pose = new Pose(28, 82, Math.toRadians(315));
    private final Pose score3Pose = new Pose(28, 80, Math.toRadians(315));
    private final Pose score4Pose = new Pose(28, 78, Math.toRadians(315));
    private final Pose score5Pose = new Pose(28, 78, Math.toRadians(315));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, grabPreset1, scorePreset1, grabPreset2, scorePreset2, grabPreset3, scorePreset3, grabPreset4, scorePreset4;
    private PathChain pushPresets;

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
                new BezierLine(
                    new Point(startPose),
                    new Point(score1Pose)
                )
        );
        scorePreload.setConstantHeadingInterpolation(heading);

        pushPresets = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(score1Pose),
                                new Point(move1Ctrl1Pose),
                                new Point(move1Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        new BezierCurve(
                                new Point(move1Pose),
                                new Point(push1_1Ctrl1Pose),
                                new Point(push1_1Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        new BezierLine(
                                new Point(push1_1Pose),
                                new Point(push1_2Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        new BezierCurve(
                                new Point(push1_2Pose),
                                new Point(push2_1Ctrl1Pose),
                                new Point(push2_1Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        new BezierLine(
                                new Point(push2_1Pose),
                                new Point(push2_2Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        new BezierCurve(
                                new Point(push2_2Pose),
                                new Point(push3_1Ctrl1Pose),
                                new Point(push3_1Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(
                        new BezierLine(
                                new Point(push3_1Pose),
                                new Point(push3_2Pose)
                        )
                )
                .build();

        grabPreset1 = new Path(
                new BezierCurve(
                        new Point(push3_2Pose),
                        new Point(grabCtrl1Pose),
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
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                follower.followPath(pushPresets,true);
                setPathState(2);
                break;
            case 2:
                follower.followPath(grabPreset1);
                setPathState(3);
                break;
            case 3:
                follower.followPath(scorePreset1);
                setPathState(4);
                break;
            case 4:
                follower.followPath(grabPreset2);
                setPathState(5);
                break;
            case 5:
                follower.followPath(scorePreset2);
                setPathState(6);
                break;
            case 6:
                follower.followPath(grabPreset3);
                setPathState(7);
                break;
            case 7:
                follower.followPath(scorePreset3);
                setPathState(8);
                break;
            case 8:
                follower.followPath(grabPreset4);
                setPathState(9);
                break;
            case 9:
                follower.followPath(scorePreset4);
                setPathState(-1);
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
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
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
