package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 5.126341744e-4;
        ThreeWheelConstants.strafeTicksToInches = 5.338970378e-4;
        ThreeWheelConstants.turnTicksToInches = 5e-4;
        ThreeWheelConstants.leftY = 6.456693;
        ThreeWheelConstants.rightY = -6.456693;
        ThreeWheelConstants.strafeX = 8.070866;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "LeftEncoder";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "RightEncoder";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "FL";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




