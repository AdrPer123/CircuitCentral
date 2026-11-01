package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz RED Auto", group = "BioBuzz", preselectTeleOp = "")
public class BioBuzzRedAuto extends BioBuzzAuto {
    @Override
    protected boolean isBlue() { return false; }
}
