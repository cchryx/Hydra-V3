package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        FollowerConstants.leftFrontMotorName = "FL";
        FollowerConstants.leftRearMotorName = "BL";
        FollowerConstants.rightFrontMotorName = "FR";
        FollowerConstants.rightRearMotorName = "BR";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        FollowerConstants.mass = 13.97064;

        FollowerConstants.xMovement = 74.96014512606203;
        FollowerConstants.yMovement = (38.312479609182674 + 46.748898211873154 + 39.78102661806133 + 45.357528801057114) / 4;

        FollowerConstants.forwardZeroPowerAcceleration = -(72.84126303680323 + 69.22129448094135 + 68.7186813453993 + 67.15259367314722) / 4;
        FollowerConstants.lateralZeroPowerAcceleration = -(118.8437043109375 + 124.85636102655246 + 117.65806149399182 + 111.19720010296628) / 4;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.2,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.2,0,0.015,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.15,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.15,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.035,0,0.000001,0,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.02,0,0.000005,0,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 3;
        FollowerConstants.centripetalScaling = 0.00039;

        FollowerConstants.pathEndTimeoutConstraint = 0;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
