/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.opt.abst.problem.meta.codification.Operator;
import ilog.concert.IloException;

/**
 *
 * @author marcio
 * @param <App>
 * @param <Model>
 */
public abstract class oBlackmoreAvoidance<App extends BlackmoreSystem, Model extends BlackmoreModel> extends Operator{
    public abstract boolean O(App approach, Model model, int j, int s) throws Exception;
    public abstract boolean N(App approach, Model model, int j, int s) throws Exception;
    public abstract void OandN(App approach, Model model, int j, int s) throws Exception;
    
    public static final boolean infeasibleNs(int vZjti[][][], int j, int s, BlackmoreApproach approach) throws IloException {
        for(int i=0; i<approach.inst.Gj(j); i++){
            if( vZjti[j][s][i]==1 && vZjti[j][s-1][i]==1 ){
                return false;
            }
        }
        return true;
    }

    public static final boolean infeasibleOt(double vXt[][], int j, int t, final double fixed_delta, BlackmoreApproach approach) throws Exception {
        for (int i = 0; i < approach.inst.Gj(j); i++) {
            final double a[] = approach.inst.obstacles[j].hyperplans[i].a;
            final double distance_risk = approach.unc.risk_allocation(t, fixed_delta, a);
            //c_(i,t) (δ_jt ) + b_ji - a_ji^T*μ_t ≤ M*(1- Z_jti )
            //c_(i,t) (δ_jt ) ≤ a_ji^T*μ_t - b_ji

            double distance_real = approach.inst.obstacles[j].distance(i, vXt[t]);

            if(distance_risk < distance_real){    //out of the ith side of the obstacle
                return false;
            }
        }
        return true;
    }

}
