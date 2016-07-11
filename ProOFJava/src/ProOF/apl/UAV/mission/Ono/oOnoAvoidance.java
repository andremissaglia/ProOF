/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.opt.abst.problem.meta.codification.Operator;
import ilog.concert.IloNumExpr;
/**
 *
 * @author marcio
 * @param <App>
 * @param <Model>
 */
public abstract class oOnoAvoidance<App extends OnoApproach, Model extends OnoModel> extends Operator{
//    public final boolean infeasible(int Y[], int Zti[][], int t) throws IloException {
//        for(int i=0; i<Zti[t].length; i++){
//            if( Zti[t][i]==1 && Zti[t-1][i]==1 ){
//                return false;
//            }
//        }
//        return true;
//    }

    public abstract void N(App approach, Model model, IloNumExpr Y, IloNumExpr Zti[][], int t) throws Exception;
}
