/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.com.language.Factory;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;

/**
 *
 * @author marcio
 */
public class fOnoAvoidance extends Factory<oOnoAvoidance>{
    public static final fOnoAvoidance obj = new fOnoAvoidance();
    @Override
    public String name() {
        return "Ono Avoidance";
    }
    @Override
    public oOnoAvoidance build(int index) {  //build the operators
        switch(index){
            case 0: return new Model1();
            case 1: return new Model2();
            case 2: return new Model6();    
        }
        return null;
    }
    private class Model1 extends oOnoAvoidance<OnoApproach, OnoModel>{
        @Override
        public String name() {
            return "Sum{Z} gt 1";
        }
        @Override
        public void N(OnoApproach approach, OnoModel model, IloNumExpr Y, IloNumExpr Zti[][], int t) throws Exception {
            model.cplex.addLe(Y, model.cplex.sum(Zti[t]), "O.sum(Z)");
        }
    }
    private class Model2 extends oOnoAvoidance<OnoApproach, OnoModel>{
        @Override
        public String name() {
            return "Sum{Z} eq 1 (fortalecimento 5)";
        }
        @Override
        public void N(OnoApproach approach, OnoModel model, IloNumExpr Y, IloNumExpr Zti[][], int t) throws Exception {
            model.cplex.addEq(Y, model.cplex.sum(Zti[t]), "O.sum(Z)");
        }
    }
    private class Model6 extends Model1{
        @Override
        public String name() {
            return "or{ Z(t) and Z(t-1) } = 1";
        }
        @Override
        public void N(OnoApproach approach, OnoModel model, IloNumExpr Y, IloNumExpr Zti[][], int t) throws Exception {
            //model.cplex.addLe(Y[t], model.cplex.sum(Zti[t]), "O.sum(Z)");
            
            //model.cplex.addEq(Y, model.cplex.sum(Zti[t]), "O.sum(Z)");
            
            if(t>0){
                IloNumExpr sum = null;
                for (int i = 0; i < Zti[t].length; i++) {
                    sum = model.cplex.SumProd(sum, 1.0, model.cplex.And("And("+i+")", Zti[t][i], Zti[t-1][i]));
                }
                //model.cplex.addEq(sum, 1, "Or6"); 
                model.cplex.addEq(sum, Y, "Or6"); 
            }else{
                //t=0
                model.cplex.addEq(Y, model.cplex.sum(Zti[t]), "O.sum(Z)");
            }
        }
    }
}

