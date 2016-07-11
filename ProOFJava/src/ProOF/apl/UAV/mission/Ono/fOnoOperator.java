/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.CplexExtended.CplexExtended;
import ProOF.apl.UAV.oFull;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.language.Factory;
import ProOF.opt.abst.problem.meta.codification.Operator;

/**
 *
 * @author marcio
 */
public class fOnoOperator extends Factory<Operator>{
    public static final fOnoOperator obj = new fOnoOperator();
    @Override
    public String name() {
        return "Ono Operators";
    }
    @Override
    public Operator build(int index) {  //build the operators
        switch(index){
            case 0: return new RAA();
            case 1: return new FRR();
            case 2: return new FRT();
        }
        return null;
    }
    private abstract class Full extends oFull<OnoApproach, OnoModel>{
        protected oOnoAvoidance avoid;
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            avoid = link.get(fOnoAvoidance.obj, avoid);
        }
        public abstract OnoModel build(OnoApproach approach, CplexExtended cplex) throws Exception;
        @Override
        public final OnoModel build_model(OnoApproach approach, CplexExtended cplex, boolean isRelax) throws Exception {
            return build(approach, cplex);
        }
    }
    private class RAA extends Full{
        @Override
        public String name() {
            return "RAA";
        }
        @Override
        public OnoModel build(OnoApproach approach, CplexExtended cplex) throws Exception {
            return new OnoModelRiskAlloc(approach, id_name(), cplex, avoid);
        }
    }
    private class FRR extends Full{
        @Override
        public String name() {
            return "FRR";
        }
        @Override
        public OnoModel build(OnoApproach approach, CplexExtended cplex) throws Exception {
            return new OnoModelRiskFixed(approach, id_name(), cplex, avoid, OnoModelRiskFixed.RISK.FRR);
        }
    }
    private class FRT extends Full{
        @Override
        public String name() {
            return "FRT";
        }
        @Override
        public OnoModel build(OnoApproach approach, CplexExtended cplex) throws Exception {
            return new OnoModelRiskFixed(approach, id_name(), cplex, avoid, OnoModelRiskFixed.RISK.FRT);
        }
    }
}
