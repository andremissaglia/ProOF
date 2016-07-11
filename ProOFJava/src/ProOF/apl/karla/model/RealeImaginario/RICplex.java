/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.karla.model.RealeImaginario;

import ProOF.CplexOpt.CplexFull;
import ProOF.com.Linker.LinkerApproaches;
import ilog.concert.IloException;


/**
 *
 * @author Administrador
 */
public class RICplex extends CplexFull{
    public  RIModel model;
    public  RIInstance inst = new RIInstance();
    
    
    public RICplex() throws IloException {
       
    }
    
    
    @Override
    public String name() {
        return "RICplex";
    }
    
        
    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        link.add(inst);
    }
    

    @Override
    public void model() throws Exception {
        model = new RIModel(cpx, inst, false);
    }

    @Override
    public void print() throws Exception {
        super.print(); //To change body of generated methods, choose Tools | Templates
        model.print();
    }

    
    
}
