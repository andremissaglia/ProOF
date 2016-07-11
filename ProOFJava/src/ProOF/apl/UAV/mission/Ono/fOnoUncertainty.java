/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.apl.UAV.gen.linear.uncertainty.LinearStateUncertainty;
import ProOF.apl.UAV.gen.linear.LinearApproach;
import ProOF.apl.UAV.gen.linear.uncertainty.LinearPerfectStateUncertainty;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearStateUncertainty;
import ProOF.com.language.Factory;

/**
 *
 * @author marcio
 */
public class fOnoUncertainty extends Factory<pLinearStateUncertainty>{
    private final LinearApproach approach;
    public fOnoUncertainty(LinearApproach approach) {
        this.approach = approach;
    }
    
    @Override
    public String name() {
        return "fOnoUncertainty";
    }
    @Override
    public pLinearStateUncertainty build(int index) throws Exception {
        switch(index){
            case 0 : return new LinearStateUncertainty(approach);
            case 1 : return new LinearPerfectStateUncertainty(approach);
        }
        return null;
    }
}
