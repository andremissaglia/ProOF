/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.CplexExtended.CplexExtended;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.gen.linear.LinearState;
import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import java.awt.Color;
import jsc.distributions.Normal;

/**
 *
 * @author marcio
 */
public class BlackmoreModelRiskFixed extends BlackmoreModel{
    private final double fixed_delta;
    
    public BlackmoreModelRiskFixed(BlackmoreSystem approach, String name, CplexExtended cplex, double fixed_delta, boolean isRelax) throws IloException, Exception {
        super(approach, name, cplex, fixed_delta, isRelax);
        this.fixed_delta = fixed_delta;
    }
    @Override
    public void addRisckAllocationConstraint(int j, int t) throws Exception {
        for(int i=0; i<system.inst().Gj(j); i++){
            final double a[] = system.inst().obstacles[j].hyperplans[i].a;
            //final double fixed_risk = approach.unc.risk_allocation(t, fixed_delta);
            final double fixed_risk = system.unc().risk_allocation(t, fixed_delta, a);
            //System.out.println("fixed_risk = "+fixed_risk);
            //c_(i,t) (δ_jt ) + b_ji - a_ji^T*μ_t ≤ M*(1- Z_jti )
            //M*Z_jti - a_ji^T*μ_t ≤ M - b_ji - c_(i,t) (δ_jt )
            IloNumExpr exp = cplex.prod(system.inst().bigM(), Zt(j, t, i));
            for(int n=0; n<system.N(); n++){
                exp = cplex.SumProd(exp, -1.0, cplex.prod(system.inst().a(j,i,n), states[t].x[n]));
            }
            cplex.addLe(exp, system.inst().bigM()-system.inst().b(j, i)-fixed_risk, name);
        }
    }
    
    @Override
    public void paint(Graphics2DReal gr, double size) throws Exception {
        super.paint(gr, size);
        
        
        if(system.plot().type()==BlackmorePlot.ALLOC.ALL && fixed_delta < system.Delta()-1e-4 && isExtract()){
            for (LinearState state : states) {
                state.plot.drawRiskAllocation(gr, system.unc(), fixed_delta, Color.CYAN, system.inst().obstacles);
            }
        }
    }
    
    public final int[][][] getTrueZjsi() throws IloException, Exception{
        int vZjti[][][] = new int[system.inst().J()][system.Steps()+1][];
        for(int j=0; j<system.inst().J(); j++){
            for(int s=0; s<system.Steps()+1; s++){
                int t = system.Waypoint(s);
                vZjti[j][s] = new int[system.inst().Gj(j)];
                double max = -Integer.MAX_VALUE;
                boolean flag = false;
                for(int i=0; i<system.inst().Gj(j); i++){
                    double exp = system.inst().obstacles[j].distance(i, cplex.getValues(states[t].x));
                    
                    exp = exp/Math.sqrt(2*system.unc().sigma(t,system.inst().obstacles[j].hyperplans[i].a));

                    if(exp<=0){
                        vZjti[j][t][i] = 0;
                    }else{
                        double delta = (1-Normal.standardTailProb(exp, false))/2;
                        vZjti[j][t][i] = delta<=fixed_delta+1e-6 ? 1 : 0;
                        flag = true;
                    }
                    max = Math.max(max, exp);
                }
                if(!flag){
                    //throw new IloException("Sum_i Zjti = 0");
                }
                //double delta = (1-Normal.standardTailProb(max, false))/2;

            }
        }
        return vZjti;
    }
}
