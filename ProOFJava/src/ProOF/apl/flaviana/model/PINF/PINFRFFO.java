/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.flaviana.model.PINF;

import ProOF.apl.advanced2.FMS.RFFO.RFFOModel;
import ProOF.apl.advanced2.FMS.RFFO.RelaxVar;
import ProOF.com.Linker.LinkerApproaches;
import java.util.ArrayList;

/**
 *
 * @author Flaviana
 */
public class PINFRFFO extends RFFOModel{

    private PINFInstance  inst = new PINFInstance();
    private PINFModel model;
    
    @Override
    public String name() {
        return "PINF-RFFO";
    }

    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        link.add(inst);
    }
    
    @Override
    public ArrayList<RelaxVar> relax_variables() throws Exception {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        ArrayList<RelaxVar> fifo = new ArrayList<RelaxVar>();
        //for(int i=0; i<inst.M;i++){
            for(int m=0; m<inst.M;m++){
                //if(i!=j){
                    fifo.add(new RelaxVar(model.Ym[m]));
                //}
            }
        //}
        //return value_wise_nearRef(fifo,0.5);
        return fifo;
    }

    @Override
    public ArrayList<RelaxVar> fix_variables() throws Exception {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        ArrayList<RelaxVar> fifo = new ArrayList<RelaxVar>();
        //for(int i=0; i<inst.M;i++){
            for(int m=0; m<inst.M;m++){
                //if(i!=m){
                    fifo.add(new RelaxVar(model.Ym[m]));
                //}
            }
        //}
//        for(int j=0; j<inst.N;j++){
//            for(int i=0; i<inst.N;i++){
//                if(i!=j){
//                    fifo.add(new RelaxVar(IloNumVarType.Bool, model.Xij[i][j]));
//                }
//            }
//        }
        return fifo;
    }

    @Override
    public void model() throws Exception {
        model = new PINFModel(cpx, inst);
    }

    @Override
    public void print() throws Exception {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
}
