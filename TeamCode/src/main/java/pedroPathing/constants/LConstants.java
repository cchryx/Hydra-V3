package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 6.38764000079E-4;
        ThreeWheelConstants.strafeTicksToInches = 6.43310484811E-4;
        ThreeWheelConstants.turnTicksToInches = 5.34232118532E-4;

        ThreeWheelConstants.leftY = 6.456693;
        ThreeWheelConstants.rightY = -6.456693;
        ThreeWheelConstants.strafeX = 8.070866;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "LeftEncoder";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "RightEncoder";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "BL";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}