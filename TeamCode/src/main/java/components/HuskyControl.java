package components;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class HuskyControl {

    private HuskyLens huskyLensDevice;
    private SampleColor state;

    public HuskyControl(HardwareMap hardwareMap, String state) {
        this.huskyLensDevice = hardwareMap.get(HuskyLens.class, "huskylens");
        if (state.equalsIgnoreCase("blue")) {
            this.state = SampleColor.BLUE;
        } else if (state.equalsIgnoreCase("red")) {
            this.state = SampleColor.RED;
        } else if (state.equalsIgnoreCase("redonly")) {
            this.state = SampleColor.REDONLY;
        } else if (state.equalsIgnoreCase("blueonly")) {
            this.state = SampleColor.BLUEONLY;
        } else if (state.equalsIgnoreCase("yellow")) {
            this.state = SampleColor.YELLOW;
        }
    }



    public double getServoPosition() {
        HuskyLens.Block[] blocks = huskyLensDevice.blocks();
        ArrayList<int[]> goodBlocks = new ArrayList<>();

        for (HuskyLens.Block block : blocks) {
            if (filter(block.id, block.width, block.height)) {
                int relativeDis = Math.abs(block.x - 160) + Math.abs(block.y - 120);
                goodBlocks.add(new int[]{block.id, block.width, block.height, block.x, block.y, relativeDis});
            }
        }

        int[] target = findClosestSample(goodBlocks);
        if (target != null) {
            double angle = getAngle(target[1], target[2]);
            return getServoAngle(angle);
        }
        return 0; // Return 0 if no valid block is found
    }

    private int[] findClosestSample(ArrayList<int[]> blocks) {
        if (blocks.isEmpty()) return null;
        int[] closestBlock = blocks.get(0);
        for (int[] block : blocks) {
            if (block[5] < closestBlock[5]) {
                closestBlock = block;
            }
        }
        return closestBlock;
    }

    private double getAngle(int width, int height) {
        double rad = Math.atan2(width, height);
        double radTilted = rad - Math.atan(1.5 / 3.5);
        return Math.toDegrees(radTilted);
    }

    private boolean filter(int id, int width, int height) {
        return state.isAllowed(id) && width * height > 5000 && width * height < 42000;
    }

    private double getServoAngle(double angle) {
        return angle / 180.0 + 0.47;
    }
}

enum SampleColor {
    RED(2, 3),
    BLUE(1, 2),
    REDONLY(3),
    BLUEONLY(1),
    YELLOW(2);

    private final int[] ids;

    SampleColor(int... ids) {
        this.ids = ids;
    }

    public boolean isAllowed(int id) {
        for (int validId : ids) {
            if (validId == id) {
                return true;
            }
        }
        return false;
    }
}



