/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.CplexExtended.CplexExtended;
import ProOF.apl.UAV.gen.linear.LinearControl;
import ProOF.apl.UAV.gen.linear.LinearState;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.apl.UAV.mission.Ono.OnoInstance.Episode;
import ProOF.apl.UAV.mission.Ono.OnoInstance.NonConvex;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumExpr;

/**
 *
 * @author marcio
 */
public class OnoModelRiskFixed extends OnoModel{
    public static enum RISK{ FRR, FRT }
    private final RISK type;
    private final double fixed_delta[];
    
    public OnoModelRiskFixed(OnoApproach approach, String name, CplexExtended cplex, oOnoAvoidance avoid, RISK type) throws IloException, Exception {
        super(approach, name, cplex);
        this.type = type;
        this.fixed_delta = new double[approach.inst.chance_constraints.length];
        for(int c=0; c<fixed_delta.length; c++){
            fixed_delta[c] = calc_delta(c);
            System.out.printf("fixed_delta[%d] = %10.8f\n", c, fixed_delta[c]);
        }
        super.addConstraints(avoid);
    }
    private double calc_delta(int c) throws Exception{
        if(type==RISK.FRR){
            return system.inst.chance_constraints[c];
        }else{
            int count = 0;
            for(LinearControl u : controls){
                count += u.FRT();
            }
            for(LinearState x : states){
                count += x.FRT();
            }
            for(int t=0; t<system.Waypoints()+1; t++){
                for(Episode ep : system.inst.episodes){
                    if(c==ep.c){
                        for(NonConvex non_convex : ep.ra.I){
                            for(int index: non_convex.C){
                                Obstacle obs = system.inst.regions[index];
                                count += obs.Gj();
                            }
                        }
                        for(NonConvex non_convex : ep.ra.O){
                            count += non_convex.C.size();
                        }
                    }
                }
            }
            return system.inst.chance_constraints[c] / count;
        }
    }
    
    @Override
    public void addWaypoint(Obstacle obs, int t, int c, IloNumExpr YC) throws Exception {
        for(int i=0; i<obs.Gj(); i++){
            //final double fixed_risk = approach.unc.risk_allocation(t, fixed_delta);
            final double fixed_risk = system.unc.risk_allocation(t, fixed_delta[c], obs.hyperplans[i].a);
            //c_(i,t) (δ_it) ≤ a_i^T*x_t - b_i 

            IloNumExpr exp = obs.hyperplans[i].scalProd(cplex, states[t].x);
            exp = cplex.sum(exp, -obs.hyperplans[i].b-fixed_risk);
            cplex.addIF_Y_Them_Ge(name, YC, exp);
        }
    }
    @Override
    public void addObstacle(Obstacle obs, int t, int c, IloNumExpr Z[], boolean det) throws Exception {
        for(int i=0; i<obs.Gj(); i++){
            //final double fixed_risk = approach.unc.risk_allocation(t, fixed_delta);
            final double fixed_risk = system.unc.risk_allocation(t, fixed_delta[c], obs.hyperplans[i].a);
            //c_(i,t) (δ_t) ≤ -a_i^T*x_t + b_i

            IloNumExpr exp = obs.hyperplans[i].scalProd(cplex, states[t].x);
            exp = cplex.sum(cplex.prod(-1, exp), +obs.hyperplans[i].b-fixed_risk);
            cplex.addIF_Y_Them_Ge(name, Z[i], exp);
        }
    }
}
