/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.HCCQSP.var;
import ProOF.com.language.Factory;

/**
 *
 * @author marcio
 */
public class fState extends Factory<oState>{
    public static final fState obj = new fState();
    @Override
    public String name() {
        return "fState";
    }
    @Override
    public oState build(int index) throws Exception {
        switch(index){
            case 0: return new oPosition("px");
            case 1: return new oPosition("py");
            case 2: return new oPosition("pz");
            case 3: return new oVelocity("vy");
            case 4: return new oVelocity("vy");
            case 5: return new oVelocity("vz");
            case 6: return new oState("fuel");
        }
        return null;
    }
}
