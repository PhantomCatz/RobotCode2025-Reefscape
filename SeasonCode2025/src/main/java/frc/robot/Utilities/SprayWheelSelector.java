// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SprayWheelSelector {
    private final CommandXboxController xboxAux;
    private final String REEFSIDE = "Reefside";
    private final double SELECTION_THRESHOLD = 0.3;

    private int previousSelected = 0;
    private int currentSelected = 0;
    private boolean isSelecting = false;

    public SprayWheelSelector(CommandXboxController aux){
        this.xboxAux = aux;

        for(int i = 0; i < 6; i++){
            SmartDashboard.putBoolean(REEFSIDE + i, false);
        }
    }

    public void updateSelected(){
        final double x = -xboxAux.getLeftY();
        final double y = -xboxAux.getLeftX();
        isSelecting = Math.hypot(x, y) > SELECTION_THRESHOLD;

        if(isSelecting){
            double angle = (Math.atan2(y, x) + 2*Math.PI) % (2 * Math.PI); //ensures angle is between 0-2pi (Dr. Eric Yuchen Lu (MD)'s idea)

            currentSelected = (int) Math.round(angle * 3.0 / Math.PI) % 6; //if angle is too close to 2pi, then it will return 6, but we want selected to be between 0-5

            if(currentSelected != previousSelected){
                setBoolean(previousSelected, false);
            }

            setBoolean(currentSelected, true);

            previousSelected = currentSelected;
        }else{
            setBoolean(currentSelected, false);
        }
    }

    public int getCurrentlySelected(){
        if(!isSelecting){
            return -1;
        }
        return currentSelected;
    }

    private void setBoolean(int selected, boolean val){
        SmartDashboard.putBoolean(REEFSIDE + selected, val);
    }
}
