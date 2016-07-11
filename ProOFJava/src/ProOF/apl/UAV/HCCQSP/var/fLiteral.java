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
public class fLiteral extends Factory<oLiteral>{
    public static final fLiteral obj = new fLiteral();
    @Override
    public String name() {
        return "fLiteral";
    }
    @Override
    public oLiteral build(int index) throws Exception {
        switch(index){
            case 0: return new oLiteral("water");
            case 1: return new oLiteral("fire");
        }
        return null;
    }
}
