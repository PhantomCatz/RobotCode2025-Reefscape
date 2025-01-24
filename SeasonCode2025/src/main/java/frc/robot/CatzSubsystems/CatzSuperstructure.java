// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems;

public class CatzSuperstructure {
    private Gamepiece chosenGamepiece = Gamepiece.CORAL;
    private int level = 1;

    public enum Gamepiece{
        CORAL,
        ALGAE
    }

    public Gamepiece getGamepieceSelection(){
        return chosenGamepiece;
    }

    public int getLevel(){
        return level;
    }

    public void setGamepieceChoice(Gamepiece choice){
        this.chosenGamepiece = choice;
    }

    public void setLevel(int lvl){
        this.level = lvl;
    }


    public enum LeftRight{
        LEFT(1),
        RIGHT(-1);
    
        public final int NUM;
    
        private LeftRight(int num){
          this.NUM  = num;
        }
      }
}
