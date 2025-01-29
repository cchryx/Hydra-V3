package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .001989436789;
        ThreeWheelConstants.strafeTicksToInches = .001989436789;
        ThreeWheelConstants.turnTicksToInches = .001989436789;
        ThreeWheelConstants.leftY = 6.456693;
        ThreeWheelConstants.rightY = -6.456693;
        ThreeWheelConstants.strafeX = 8.070866;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "LeftEncoder";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "RightEncoder";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "FL";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




