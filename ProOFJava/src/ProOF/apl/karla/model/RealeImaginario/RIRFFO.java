/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.karla.model.RealeImaginario;

import ProOF.apl.advanced2.FMS.RFFO.RFFOModel;
import ProOF.apl.advanced2.FMS.RFFO.RelaxVar;
import ProOF.com.Linker.LinkerApproaches;
import java.util.ArrayList;

/**
 *
 * @author Administrador
 */
public class RIRFFO extends RFFOModel { 
    private RIModel model;
    private final RIInstance inst = new RIInstance();

    @Override
    public String name() {
        return "ri-RFFO"; //To change body of generated methods, choose Tools | Templates.
    }
    
     @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        link.add(inst);
    }
    
    @Override
    public void model() throws Exception {
        model = new RIModel(cpx, inst,true); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public ArrayList<RelaxVar> relax_variables() throws Exception {
        ArrayList<RelaxVar> fifo = new ArrayList<RelaxVar>();// fifo lista de variaveis a ser otimizada
        for(int i=0; i<inst.Nnos; i++){
            for(int j=0; j<inst.Nnos; j++){
              if (inst.Rr[i][j] > 0.0000001) {
                fifo.add(new RelaxVar( model.x[i][j]));
              }  
            }
        }
        return fifo;
        //return value_wise_nearRef(fifo, 0.5); // para esolher os proximos de 0.5 para relaxar
    }
    

    @Override
    public ArrayList<RelaxVar> fix_variables() throws Exception {
        ArrayList<RelaxVar> fifo = new ArrayList<RelaxVar>();// fifo lista de variaveis a ser otimizada
        for(int i=0; i<inst.Nnos; i++){
            for(int j=0; j<inst.Nnos; j++){
              if (inst.Rr[i][j] > 0.0000001) {
                fifo.add(new RelaxVar( model.x[i][j]));
              }  
            }
        }
        return fifo;
        //return value_wise_nearRef(fifo, 0.5); // para esolher os proximos de 0.5 para relaxar
    }
    
    
    @Override
    public void print() throws Exception {
        super.print(); //To change body of generated methods, choose Tools | Templates.
        model.print();
    }

    

  
    
}
