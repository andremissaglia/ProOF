/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.vehicle.parts;

import ProOF.apl.UAV.abst.uncertainty.Delta;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearStateUncertainty;
import ProOF.apl.UAV.gen.linear.LinearState;
import ProOF.apl.UAV.gen.linear.LinearApproach;
import ProOF.apl.UAV.gen.linear.LinearModel;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.language.Factory;
import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;

/**
 *
 * @author marcio
 */
public class fLinearStateBound extends Factory<oLinearStateBound>{
    public static final fLinearStateBound obj = new fLinearStateBound();
    @Override
    public String name() {
        return "State Bound";
    }
    @Override
    public oLinearStateBound build(int index) {  //build the operators
        switch(index){
            case 0: return new DetNorm1VtUB();
            case 1: return new UncNorm1VtUB();
            case 2: return new DetNorm2VtUB();
            case 3: return new UncNorm2VtUB();
            case 4: return new DetXtRn();
            //case 2: return new DetFixedWings();
            //case 5: return new UncFixedWings();
            //case 6: return new DetRealShapeVt();
            //case 7: return new UncRealShapeVt();
        }
        return null;
    }
    private class DetNorm1VtUB extends oLinearStateBound<LinearSystem, LinearModel>{
        
        @Override
        public String name() {
            return "DetNorm1(Vt)";
        }
        @Override
        public LinearState[] build_states(LinearSystem approach, final LinearModel model) throws Exception {
            LinearState states[] = new LinearState[approach.Waypoints()+1];
            for(int t=0; t<approach.Waypoints()+1; t++){
                states[t] = new LinearState(model.cplex, approach.N()*2,t){
                    @Override
                    public IloNumExpr delta() throws IloException {
                        return null;
                    }
                    @Override
                    public IloNumExpr risk() throws IloException {
                        return null;
                    }
                    @Override
                    public int FRT() throws Exception {
                        return 0;
                    }
                };
                for(int i=approach.N(); i<2*approach.N(); i++){
                    states[t].x[i].setLB(-approach.maxVelocity());
                    states[t].x[i].setUB(+approach.maxVelocity());
                }
            }
            return states;
        }
    }
    private class UncNorm1VtUB extends oLinearStateBound<LinearSystem, LinearModel>{
        private pLinearStateUncertainty unc;
        private Delta delta;
        private int Naprox;
        @Override
        public String name() {
            return "UncNorm1(Vt)";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            unc = link.need(pLinearStateUncertainty.class, unc);
            delta = link.need(Delta.class, delta);
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            Naprox  = link.Int("Velocity-N-risk", 12, 1, 64, 
                    "Number of picewise restriction to aproximate the "+
                    "inverse of erf(x) to calculate the risk in Norm1 velocity upper bound");
        }
        @Override
        public LinearState[] build_states(LinearSystem approach, final LinearModel model) throws Exception {
            LinearState states[] = new LinearState[approach.Waypoints()+1];
            for(int t=0; t<approach.Waypoints()+1; t++){
                final IloNumVar In[] = model.cplex.numVarArray(2*approach.N(), 0, delta.Delta(), "V("+t+").Risk");
                final IloNumExpr c[] = new IloNumExpr[2*approach.N()];
                for(int n=0; n<2*approach.N(); n++){
                    int i = approach.N()+n/2;
                    double uncertainty = Math.sqrt(2*(unc.Sigma(t, i, i)));    //vetores orgotonais resultam nessa simplificação
                    c[n] = model.cplex.RiskAllocation(In[n], uncertainty, delta.Delta(), Naprox, "V("+t+").Alloc");
                }
                
                states[t] = new LinearState(model.cplex, approach.N()*2,t){
                    @Override
                    public IloNumExpr delta() throws IloException {
                        return model.cplex.sum(In);
                    }
                    @Override
                    public IloNumExpr risk() throws IloException {
                        return model.cplex.sum(c);
                    }
                    @Override
                    public int FRT() throws Exception {
                        return In.length;
                    }
                };
                for(int i=approach.N(); i<2*approach.N(); i++){ //vx,vy,vz,...
                    states[t].x[i].setLB(-approach.maxVelocity());
                    states[t].x[i].setUB(+approach.maxVelocity());
                }
                for(int n=0; n<2*approach.N(); n++){
                    int i = approach.N()+n/2;
                    double a = 1-(n%2)*2;
                    model.cplex.addLe(
                        model.cplex.sum(model.cplex.prod(a, states[t].x[i]), c[n]), +approach.maxVelocity()
                    );
                }
            }
            return states;
        }
    }

    private class DetNorm2VtUB extends oLinearStateBound<LinearSystem, LinearModel>{
        private final int N = 16;
        @Override
        public String name() {
            return "DetNorm2(Vt)";
        }
        @Override
        public LinearState[] build_states(LinearSystem approach, final LinearModel model) throws Exception {
            LinearState states[] = new LinearState[approach.Waypoints()+1];
            for(int t=0; t<approach.Waypoints()+1; t++){
                states[t] = new LinearState(model.cplex, approach.N()*2,t){
                    @Override
                    public IloNumExpr delta() throws IloException {
                        return null;
                    }
                    @Override
                    public IloNumExpr risk() throws IloException {
                        return null;
                    }
                    @Override
                    public int FRT() throws Exception {
                        return 0;
                    }
                };
                for(int i=approach.N(); i<2*approach.N(); i++){ //vx,vy,vz,...
                    states[t].x[i].setLB(-approach.maxVelocity());
                    states[t].x[i].setUB(+approach.maxVelocity());
                }
                
                for(int n=0; n<N; n++){ //only vx,vy
                    IloNumExpr exp = null;
                
                    IloNumVar vx = states[t].x[approach.N()];
                    exp = model.cplex.SumProd(exp, Math.cos((2*Math.PI*n)/N), vx);
                    
                    IloNumVar vy = states[t].x[approach.N()+1];
                    exp = model.cplex.SumProd(exp, Math.sin((2*Math.PI*n)/N), vy);
                    
                    model.cplex.addLe(exp, approach.maxVelocity(), "Vmax");
                }
            }
            return states;
        }
    }
    private class UncNorm2VtUB extends oLinearStateBound<LinearSystem, LinearModel>{
        private pLinearStateUncertainty unc;
        private Delta delta;
        private int Naprox;
        private final int N = 16;
        @Override
        public String name() {
            return "UncNorm2(Vt)";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            unc = link.need(pLinearStateUncertainty.class, unc);
            delta = link.need(Delta.class, delta);
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            Naprox  = link.Int("Velocity-N-risk", 12, 1, 64, 
                    "Number of picewise restriction to aproximate the "+
                    "inverse of erf(x) to calculate the risk in Norm1 velocity upper bound");
        }
        @Override
        public LinearState[] build_states(LinearSystem approach, final LinearModel model) throws Exception {
            if(approach.N()!=2){
                throw new Exception("The class "+this.class_name()+" needs a 2D map");
            }
            
            LinearState states[] = new LinearState[approach.Waypoints()+1];
            for(int t=0; t<approach.Waypoints()+1; t++){
                final IloNumVar In[] = model.cplex.numVarArray(N, 0, delta.Delta(), "V("+t+").Risk");
                final IloNumExpr c[] = new IloNumExpr[N];
                for(int n=0; n<N; n++){ //only vx,vy
                    double a[] = new double[]{0, 0, Math.cos((2*Math.PI*n)/N), Math.sin((2*Math.PI*n)/N)};
                    double uncertainty = Math.sqrt(2*(unc.sigma(t, a)));
                    c[n] = model.cplex.RiskAllocation(In[n], uncertainty, delta.Delta(), Naprox, "V("+t+").Alloc");
                }
                
                states[t] = new LinearState(model.cplex, approach.N()*2,t){
                    @Override
                    public IloNumExpr delta() throws IloException {
                        return model.cplex.sum(In);
                    }
                    @Override
                    public IloNumExpr risk() throws IloException {
                        return model.cplex.sum(c);
                    }
                    @Override
                    public int FRT() throws Exception {
                        return In.length;
                    }
                };
                for(int i=approach.N(); i<2*approach.N(); i++){ //vx,vy,vz,...
                    states[t].x[i].setLB(-approach.maxVelocity());
                    states[t].x[i].setUB(+approach.maxVelocity());
                }
                
                for(int n=0; n<N; n++){ //only vx,vy
                    IloNumExpr exp = c[n];
                
                    IloNumVar vx = states[t].x[approach.N()];
                    exp = model.cplex.SumProd(exp, Math.cos((2*Math.PI*n)/N), vx);
                    
                    IloNumVar vy = states[t].x[approach.N()+1];
                    exp = model.cplex.SumProd(exp, Math.sin((2*Math.PI*n)/N), vy);
                    
                    model.cplex.addLe(exp, approach.maxVelocity(), "Vmax");
                }
            }
            return states;
        }
    }
    private class DetXtRn extends oLinearStateBound<LinearApproach, LinearModel>{
        
        @Override
        public String name() {
            return "DetXtRn";
        }
        @Override
        public LinearState[] build_states(LinearApproach approach, final LinearModel model) throws Exception {
            LinearState states[] = new LinearState[approach.Waypoints()+1];
            for(int t=0; t<approach.Waypoints()+1; t++){
                states[t] = new LinearState(model.cplex, approach.N(),t){
                    @Override
                    public IloNumExpr delta() throws IloException {
                        return null;
                    }
                    @Override
                    public IloNumExpr risk() throws IloException {
                        return null;
                    }
                    @Override
                    public int FRT() throws Exception {
                        return 0;
                    }
                };
            }
            return states;
        }
    }
}
