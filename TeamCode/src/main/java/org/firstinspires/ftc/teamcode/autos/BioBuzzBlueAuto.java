package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz BLUE Auto", group = "BioBuzz", preselectTeleOp = "")
public class BioBuzzBlueAuto extends BioBuzzAuto {
    @Override
    protected boolean isBlue() { return true; }
}
