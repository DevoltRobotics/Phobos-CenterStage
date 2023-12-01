package org.firstinspires.ftc.teamcode.vision;

import org.openftc.easyopencv.OpenCvPipeline;

public abstract class TeamElementDetectionPipeline extends OpenCvPipeline {

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public abstract Pattern getAnalysis();

}
