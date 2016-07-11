/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.apl.UAV.gen.linear.uncertainty.LinearOpenStateUncertainty;
import ProOF.apl.UAV.gen.linear.uncertainty.LinearPerfectStateUncertainty;
import ProOF.apl.UAV.gen.linear.uncertainty.LinearStateFileUncertainty;
import ProOF.apl.UAV.gen.linear.uncertainty.LinearStateFileUncertaintyClosedLoop;
import ProOF.apl.UAV.gen.linear.uncertainty.LinearStateUncertainty;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearStateUncertainty;
import ProOF.com.language.Factory;

/**
 *
 * @author marcio
 */
public class fBlackmoreUncertainty extends Factory<pLinearStateUncertainty>{
    private final LinearSystem approach;
    public fBlackmoreUncertainty(LinearSystem approach) {
        this.approach = approach;
    }
    
    @Override
    public String name() {
        return "fBlackmoreUncertainty";
    }
    @Override
    public pLinearStateUncertainty build(int index) throws Exception {
        switch(index){
            case 0 : return new LinearStateUncertainty(approach);
            case 1 : return new RoverStateUncertainty(approach);
            case 2 : return new LinearStateFileUncertainty(approach);
            case 3 : return new LinearStateFileUncertaintyClosedLoop(approach);
            case 4 : return new LinearPerfectStateUncertainty(approach);
            case 5 : return new LinearOpenStateUncertainty(approach);
        }
        return null;
    }
}
