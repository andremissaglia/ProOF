/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.CplexExtended.CplexExtended;
import ProOF.CplexExtended.iCplexExtract;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.gen.linear.LinearState;
import ProOF.com.Linker.LinkerResults;
import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import java.awt.Color;
import java.io.PrintStream;

/**
 *
 * @author marcio
 */
public class BlackmoreModelRiskAlloc extends BlackmoreModel{
    protected final IloNumVar Djt[][];
    protected final IloNumExpr Cjti[][][];
    //private final IloNumVar Djti[][][];
    
    public BlackmoreModelRiskAlloc(BlackmoreSystem approach, String name, CplexExtended cplex, double delta_to_cut, boolean isRelax) throws IloException, Exception {
        super(approach, name, cplex, delta_to_cut, isRelax);
        Djt = cplex.numVarArray(approach.inst().J(), approach.Waypoints()+1, 0, Double.POSITIVE_INFINITY, "Djt");
//        Djti = new IloNumVar[approach.inst.J()][approach.Waypoints()+1][];
//        for(int j=0; j<approach.inst.J(); j++){
//            for(int t=0; t<approach.Waypoints()+1; t++){
//                Djti[j][t] = cplex.numVarArray(approach.inst.Gj(j), 0, Double.POSITIVE_INFINITY, "Djti");
//            }
//        }
//        for(int j=0; j<approach.inst.J(); j++){
//            for(int t=0; t<approach.Waypoints()+1; t++){
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    cplex.addGe(Djt[j][t], Djti[j][t][i], "EqDelta");
//                }
//            }
//        }
        Cjti = new IloNumExpr[approach.inst().J()][approach.Waypoints()+1][];
        for(int j=0; j<approach.inst().J(); j++){
            for(int t=0; t<approach.Waypoints()+1; t++){
                Cjti[j][t] = new IloNumExpr[approach.inst().Gj(j)];
            }
        }
            
        IloNumExpr exp = null;
        for(int j=0; j<approach.inst().J(); j++){
            for(int t=0; t<approach.Waypoints()+1; t++){
                exp = cplex.SumProd(exp, 1.0, Djt[j][t]);
            }
        }
        for(int t=0; t<approach.Waypoints()+1; t++){
            exp = cplex.SumProd(exp, 1.0, states[t].delta());
        }
        cplex.addLe(exp, approach.Delta(), "SumDelta");
    }
    
    @Override
    public void addRisckAllocationConstraint(int j, int t) throws Exception {
        
        IloNumExpr erf_inv = cplex.ERF_Inv(Djt[j][t], system.Delta(), 2.0, system.Naprox(), true, "Erf"); 
        
        for(int i=0; i<system.inst().Gj(j); i++){
            //c_(i,t) (δ_jt ) + b_ji - a_ji^T*μ_t ≤ M*(1- Z_jti )
            //c_(i,t) (δ_jt ) + M*Z_jti - a_ji^T*μ_t ≤ M - b_ji
            double a[] = system.inst().obstacles[j].hyperplans[i].a;
            double sigma = system.unc().sigma(t, a);
            double uncertainty = Math.sqrt(2*sigma);
            
            //IloNumExpr risk = cplex.RiskAllocation(Djti[j][t][i], uncertainty, approach.Delta(), approach.Naprox(), "Cit");
            //Cjti[j][t][i] = cplex.RiskAllocation(Djt[j][t], uncertainty, approach.Delta(), approach.Naprox(), "Cit");

            if(uncertainty<1e-6){
                Cjti[j][t][i] = cplex.constant(0.0);
            }else{
                Cjti[j][t][i] = cplex.prod(erf_inv, uncertainty);
            }
//            
            IloNumExpr exp = cplex.sum(Cjti[j][t][i], cplex.prod(system.inst().bigM(), Zt(j, t, i)));
            for(int n=0; n<system.N(); n++){
                exp = cplex.SumProd(exp, -1.0, cplex.prod(system.inst().a(j,i,n), states[t].x[n]));
            }
            cplex.addLe(exp, system.inst().bigM()-system.inst().b(j, i), name);
            
            //for 2D xt:=[px,py,vx,vy] ut:=[ax,ay]; vmax = 3.0, umax = 1.0, T = 20, dt=1.0
            //final double dynamicCorrection = 0.25f;
            //cplex.addLe(exp, approach.inst.bigM()-approach.inst.b(j, i)-dynamicCorrection, name);
        }
    }
    
    private double delta;
    private double Dt[];
    private void extractRisk(iCplexExtract ext) throws IloException, Exception {
        Dt = new double[system.Waypoints()+1];
        delta = 0;
        for(int t=0; t<system.Waypoints()+1; t++){
            double max = Double.NEGATIVE_INFINITY;
            for(int j=0; j<system.inst().J(); j++){
                max = Math.max(max,ext.getValue(Djt[j][t]));
                delta += ext.getValue(Djt[j][t]);
            }
            Dt[t] = max;
        }
    }
    
    @Override
    public void extract(iCplexExtract ext, Callback type) throws IloException, Exception {
        super.extract(ext, type); //To change body of generated methods, choose Tools | Templates.
        extractRisk(ext);
    }

    @Override
    public void paint(Graphics2DReal gr, double size) throws Exception {
        super.paint(gr, size); //To change body of generated methods, choose Tools | Templates.
        if(system.plot().type()==BlackmorePlot.ALLOC.ALL && isExtract()){
            for(int t=0; t<states.length; t++){
                states[t].plot.drawRiskAllocation(gr, system.unc(), Dt[t], Color.BLUE, system.inst().obstacles);
            }
        }
    }

    @Override
    public void results(LinkerResults link) throws Exception {
        super.results(link); //To change body of generated methods, choose Tools | Templates.
        link.writeDbl("delta-raa", delta);
        link.writeDbl("true_alloaction", true_alloaction);
    }

    
    
    @Override
    public void save() throws Exception {
        if(isFeasible()){
            PrintStream out = new PrintStream("./output.txt");
            out.printf("%d\n", states.length);
            for(LinearState state : states){
                double x[] = cplex.getValues(state.x);
                for(int i=0; i<x.length; i++){
                    out.printf("%1.14f ", x[i]);
                }
                out.println();
            }
            
            out.println("<objective value>");
            out.printf("%g\n", cplex.getObjValue());
            
            out.println("<risk allocation for each time step>");
            for(double d : Dt){
                out.printf("%g, ", d);
            }
            out.println();
            
            system.unc().print_covariance(out);
            
            out.close();
            
            PrintStream route = new PrintStream("./route.txt");
            route.printf("%d\n", states.length);
            for(LinearState state : states){
                double x[] = cplex.getValues(state.x);
                if(system.N()==2){
                    for(int i=0; i<2; i++){
                        route.printf("%1.14f ", x[i]);
                    }
                    if(system.inst() instanceof fBlackmoreInstance.Instance2DEmterprise){
                        route.printf("%1.0f\n", ((fBlackmoreInstance.Instance2DEmterprise)system.inst()).fly_altitude);
                    }else{
                        route.printf("%1.0f\n", 0.0);
                    }
                }else{
                    for(int i=0; i<2; i++){
                        route.printf("%1.14f ", x[i]);
                    }
                    route.printf("%1.14f ", x[2]);
                }
            }
            
            route.println("<objective value>");
            route.printf("%g\n", cplex.getObjValue());
            
            route.println("<risk allocation for each time step>");
            for(double d : Dt){
                route.printf("%g, ", d);
            }
            route.println();
            
            system.unc().print_covariance(route);
            
            route.close();
            
            
        }
    }
}
