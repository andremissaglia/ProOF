/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.flaviana.model.PINF;

import ProOF.CplexOpt.CplexFull;
//import ProOF.apl.flaviana.model.PINFModel;
//import ProOF.apl.flaviana.problem.PINF.PINFInstance;
import ProOF.com.Linker.LinkerApproaches;
import ilog.concert.IloException;

/**
 *
 * @author Flaviana
 */
public class PINFCplex extends CplexFull{
    
    public PINFModel model;
    public PINFInstance inst = new PINFInstance();//opcional--deve ser  add  em  services
    
    public PINFCplex() throws IloException {
        
    }
     @Override
    public String name() {
        return "PINF-Cplex";
    }

    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        link.add(inst);//registra q o modelo receba a instancia
    }

    @Override
    public void model() throws Exception {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        model = new PINFModel(cpx, inst);
    }
}
