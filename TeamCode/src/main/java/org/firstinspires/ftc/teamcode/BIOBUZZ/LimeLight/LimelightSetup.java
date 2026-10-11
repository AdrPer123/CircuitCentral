package org.firstinspires.ftc.teamcode.BIOBUZZ.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightSetup {
    private Limelight3A limelight;
    private double tx, ty, tz;
    public void init(HardwareMap HwMap) {
        limelight = HwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // april tag #11 pipeline
    }

    public double getTx() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTx();
        }
        else {
            return 0;
        }
    }

}
