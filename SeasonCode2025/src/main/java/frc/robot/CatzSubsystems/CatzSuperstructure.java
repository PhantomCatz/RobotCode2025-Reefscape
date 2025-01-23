// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems;

public class CatzSuperstructure {
    private CoralAlgae coralAlgae = CoralAlgae.CORAL;
    private int level = 1;

    public enum CoralAlgae{
        CORAL,
        ALGAE
    }

    public CoralAlgae getCoralAlgae(){
        return coralAlgae;
    }

    public int getLevel(){
        return level;
    }

    public void setCoralAlgae(CoralAlgae ca){
        this.coralAlgae = ca;
    }

    public void setLevel(int lvl){
        this.level = lvl;
    }
}
