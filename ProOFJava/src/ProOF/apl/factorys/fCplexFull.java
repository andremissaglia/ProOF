/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.factorys;

import ProOF.apl.advanced2.problem.cplex.GCISTModelFull;
import ProOF.apl.pog.method.RFFO.PPDCP.PPDCPModelFull;
import ProOF.apl.sample2.problem.cplex.TSPFull;
import ProOF.apl.advanced2.problem.cplex.MLCLSPwBFull;
import ProOF.com.language.Factory;
import ProOF.CplexOpt.CplexFull;
import ProOF.apl.UAV.HCCQSP.HCCQSPFull;
import ProOF.apl.UAV.HCCQSP.HCCQSPMount;
import ProOF.apl.UAV.UAVfull;
import ProOF.apl.flaviana.model.PINF.PINFCplex;
import ProOF.apl.karla.model.RealeImaginario.RICplex;
import ilog.concert.IloException;

/**
 *
 * @author marcio
 */
public class fCplexFull extends Factory<CplexFull>{
    public static final fCplexFull obj = new fCplexFull();
    
    @Override
    public String name() {
        return "Models";
    }
    
    @Override
    public CplexFull build(int index) throws IloException {
        switch(index){
            case 0: return new TSPFull();
            case 1: return new GCISTModelFull();
            case 2: return new PPDCPModelFull();
            case 3: return new MLCLSPwBFull();
            case 4: return new UAVfull();
            case 5: return new PINFCplex();
            case 6: return new RICplex();
            case 7: return new HCCQSPFull();
            case 8: return new HCCQSPMount();
        }
        return null;
    }
}
