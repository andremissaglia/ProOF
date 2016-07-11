/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.CplexExtended.CplexExtended;
import ProOF.CplexExtended.CplexPrinter;
import ProOF.CplexExtended.iCplexExtract;
import ProOF.apl.UAV.gen.linear.LinearControl;
import ProOF.apl.UAV.gen.linear.LinearState;
import ProOF.apl.UAV.map.Obstacle;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;

/**
 *
 * @author marcio
 */
public class OnoModelRiskAlloc extends OnoModel{
    private final IloNumExpr Dc[];
    private final IloNumExpr erf_inv[][];
    
    public OnoModelRiskAlloc(OnoApproach approach, String name, CplexExtended cplex, oOnoAvoidance avoid) throws IloException, Exception {
        super(approach, name, cplex);
        Dc = new IloNumExpr[approach.inst.chance_constraints.length]; 
        
        if(system.strengthened[4]){
            IloNumVar delta[][] = cplex.numVarArray(approach.Waypoints()+1, approach.inst.chance_constraints.length, 0, Double.POSITIVE_INFINITY, "delta");
            erf_inv = new IloNumExpr[approach.inst.chance_constraints.length][approach.Waypoints()+1];
            for(int c=0; c<approach.inst.chance_constraints.length; c++){
                for(int t=0; t<approach.Waypoints()+1; t++){
                    erf_inv[c][t] = cplex.ERF_Inv(delta[t][c], approach.inst.chance_constraints[c], 2.0, approach.Naprox(), "erf_inv["+c+"]");
                    
                    Dc[c] = cplex.SumProd(Dc[c], 1.0, delta[t][c]);
                }
            }
            
            printers.add(new CplexPrinter() {
                @Override
                public void printer(iCplexExtract ext) throws IloException {
                    double vEt[][] = ext.getValues(erf_inv, true);
                    ext.print("erf-inv", "-%5.2f", vEt);
                    double vDt[][] = ext.getValues(delta, true);
                    ext.print("delta", "-%5.5f", vDt);
                }
            });
        }else{
            erf_inv = null;
        }
        
        super.addConstraints(avoid);
        
        
        
        
        //---------------------- Limitando a alocação do risco: ---------------------------------
        IloNumExpr dU = null;
        for(LinearControl u : controls){
            dU = cplex.SumProd(dU, 1.0, u.delta());
        }
        IloNumExpr dX = null;
        for(LinearState x : states){
            dX = cplex.SumProd(dX, 1.0, x.delta());
        }
        for(int c = 0; c<approach.inst.chance_constraints.length; c++){
            cplex.addLe(cplex.Sum(Dc[c], dU, dX), approach.inst.chance_constraints[c], "dU+dX+Dc <= Delta_c");
        }
    }
    
    /*
    @Override
    public void addWaypoint(Obstacle obs, int t, int c, IloIntVar YC) throws Exception {
        IloNumVar delta = cplex.numVar(0.0, Double.POSITIVE_INFINITY, "dI");
        for(int i=0; i<obs.Gj(); i++){
            

            double uncertainty = Math.sqrt(2*approach.unc.sigma(t, obs.hyperplans[i].a));
            //IloNumExpr risk = cplex.RiskAllocationCond(delta, uncertainty, approach.inst.chance_constraints[c], approach.Naprox(), YC, "C(i,t,I)");
            IloNumExpr risk = cplex.RiskAllocation(delta, uncertainty, approach.inst.chance_constraints[c], approach.Naprox(), "C(i,t,I)");

            //h * x <= b - c
            IloNumExpr exp = obs.hyperplans[i].scalProd(cplex, states[t].x);
            exp = cplex.sum(exp, -obs.hyperplans[i].b);
            exp = cplex.sum(exp, risk);
            cplex.addIF_Y_Them_Le(name, YC, exp);
            
            
//                                
//                                final double fixed_delta = 0.001;
//
//                                //final double fixed_risk = approach.unc.risk_allocation(t, fixed_delta);
//                                final double fixed_risk = approach.unc.risk_allocation(t, fixed_delta, obs.hyperplans[i].a);
//                                //c_(i,t) (δ_it) ≤ a_i^T*x_t - b_i 
//                                
//                                IloNumExpr exp = obs.hyperplans[i].scalProd(cplex, states[t].x);
//                                exp = cplex.sum(exp, -obs.hyperplans[i].b-fixed_risk);
//                                cplex.addIF_Y_Them_Ge(name, YC[k], exp);


        }
        Dc[c] = cplex.SumProd(Dc[c], 1.0, delta);
    }*/
    
    private double[] Unc(Obstacle obs, int t) throws Exception{
        double unc[] = new double[obs.Gj()];
        boolean isUnc = false;
        for(int i=0; i<obs.Gj(); i++){
            unc[i] = Math.sqrt(2*system.unc.sigma(t, obs.hyperplans[i].a));
            if(unc[i]>1e-6){
                isUnc = true;
            }
        }
        return isUnc ? unc : null;
    }
    
    @Override
    public void addWaypoint(Obstacle obs, int t, int c, IloNumExpr YC) throws Exception {
        double unc[] = Unc(obs, t);
        if(unc!=null){
            if(system.strengthened[4]){
                for(int i=0; i<obs.Gj(); i++){
                    cplex.addWaypointUnc(obs.hyperplans[i], states[t].x, YC, erf_inv[c][t], unc[i]);
                }
            }else{
                for(int i=0; i<obs.Gj(); i++){
                    IloNumVar delta = cplex.numVar(0.0, Double.POSITIVE_INFINITY, "temp");
                    cplex.addWaypointUnc(obs.hyperplans[i], states[t].x, YC, delta, unc[i], system.inst.chance_constraints[c], system.Naprox());
                    Dc[c] = cplex.SumProd(Dc[c], 1.0, delta);
                }
            }
        }else{
            for(int i=0; i<obs.Gj(); i++){
                cplex.addWaypointDet(obs.hyperplans[i], states[t].x, YC);
            }
        }
    }

    @Override
    public void addObstacle(Obstacle obs, int t, int c, IloNumExpr Z[], boolean det) throws Exception {
        double unc[] = Unc(obs, t);
        if(unc!=null && !det){
            if(system.strengthened[4]){
                for(int i=0; i<obs.Gj(); i++){
                    cplex.addObstacleUnc(obs.hyperplans[i], states[t].x, Z[i], erf_inv[c][t], unc[i]);
                }
            }else{
                IloNumVar delta = cplex.numVar(0.0, Double.POSITIVE_INFINITY, "temp");
                for(int i=0; i<obs.Gj(); i++){
                    cplex.addObstacleUnc(obs.hyperplans[i], states[t].x, Z[i], delta, unc[i], system.inst.chance_constraints[c], system.Naprox());
                }
                Dc[c] = cplex.SumProd(Dc[c], 1.0, delta);
            }
        }else{
            for(int i=0; i<obs.Gj(); i++){
                cplex.addObstacleDet(obs.hyperplans[i], states[t].x, Z[i]);
            }
        }
    }
}
