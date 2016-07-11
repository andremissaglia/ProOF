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
public class fControl extends Factory<oControl>{
    public static final fControl obj = new fControl();
    @Override
    public String name() {
        return "fControl";
    }
    @Override
    public oControl build(int index) throws Exception {
        switch(index){
            case 0: return new oControl("ax");
            case 1: return new oControl("ay");
            case 2: return new oControl("az");
            case 3: return new oControl("rf");  //rendimento do combustível
        }
        return null;
    }
}
