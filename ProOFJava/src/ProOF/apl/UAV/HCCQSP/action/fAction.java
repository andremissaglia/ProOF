/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.HCCQSP.action;
import ProOF.com.language.Factory;

/**
 *
 * @author marcio
 */
public class fAction extends Factory<oAction>{
    public static final fAction obj = new fAction();
    @Override
    public String name() {
        return "fAction";
    }
    @Override
    public oAction build(int index) throws Exception {
        switch(index){
            case 0: return new Fly();
            case 1: return new Fill();
            case 2: return new Drop();
        }
        return null;
    }

    private static class Fly extends oAction {
        @Override
        public String name() {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }
    }
    private static class Fill extends oAction {
        @Override
        public String name() {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }
    }
    private static class Drop extends oAction {
        @Override
        public String name() {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }
    }
}
