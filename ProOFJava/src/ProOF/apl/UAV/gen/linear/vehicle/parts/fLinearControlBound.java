/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.vehicle.parts;

import ProOF.apl.UAV.abst.uncertainty.Delta;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearControlUncertainty;
import ProOF.apl.UAV.gen.linear.LinearControl;
import ProOF.apl.UAV.gen.linear.LinearApproach;
import ProOF.apl.UAV.gen.linear.LinearModel;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.language.Factory;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;

/**
 *
 * @author marcio
 */
public class fLinearControlBound extends Factory<oLinearControlBound>{
    public static final fLinearControlBound obj = new fLinearControlBound();
    @Override
    public String name() {
        return "Control Bound";
    }
    @Override
    public oLinearControlBound build(int index) {  //build the operators
        switch(index){
            case 0: return new DetNorm1UtUB();
            case 1: return new UncNorm1UtUB();
            case 2: return new DetNorm2UtUB();
            case 3: return new UncNorm2UtUB();
            case 4: return new UncNorm2UtUB_deltaU();
            //case 4: return new DetRealShapeUt();
            //case 5: return new UncRealShapeUt();
                
        }
        return null;
    }
    private class DetNorm1UtUB extends oLinearControlBound<LinearSystem, LinearModel>{
        @Override
        public String name() {
            return "DetNorm1(Ut)";
        }
        @Override
        public LinearControl[] build_controls(LinearSystem approach, final LinearModel model) throws Exception {
            LinearControl controls[] = new LinearControl[approach.Waypoints()];
            for(int t=0; t<approach.Waypoints(); t++){
                controls[t] = new LinearControl(model.cplex, approach.N()){
                    @Override
                    public IloNumExpr delta() throws Exception {
                        return null;
                    }
                    @Override
                    public IloNumExpr risk() throws Exception {
                        return null;
                    }
                    @Override
                    public int FRT() throws Exception {
                        return 0;
                    }
                };
                for(int i=0; i<approach.N(); i++){
                    controls[t].u[i].setLB(-approach.maxControl());
                    controls[t].u[i].setUB(+approach.maxControl());
                }
            }
            return controls;
        }
    }
    private class UncNorm1UtUB extends oLinearControlBound<LinearSystem, LinearModel>{
        private pLinearControlUncertainty unc;
        private Delta delta;
        private int Naprox;
        @Override
        public String name() {
            return "UncNorm1(Ut)";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            unc = link.need(pLinearControlUncertainty.class, unc);
            delta = link.need(Delta.class, delta);
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            Naprox  = link.Int("Control-N-risk", 12, 1, 64, 
                    "Number of picewise restriction to aproximate the "+
                    "inverse of erf(x) to calculate the risk in Norm1 control upper bound");
        }
        @Override
        public LinearControl[] build_controls(LinearSystem approach, final LinearModel model) throws Exception {
            LinearControl controls[] = new LinearControl[approach.Waypoints()];
            for(int t=0; t<approach.Waypoints(); t++){
                final IloNumVar In[] = model.cplex.numVarArray(2*approach.N(), 0, delta.Delta(), "U("+t+").Risk");
                
                final IloNumExpr c[] = new IloNumExpr[2*approach.N()];
                for(int n=0; n<2*approach.N(); n++){
                    int i = n/2;
                    double uncertainty = Math.sqrt(2*(unc.Sigma(t, i, i)));    //vetores orgotonais resultam nessa simplificação
                    c[n] = model.cplex.RiskAllocation(In[n], uncertainty, delta.Delta(), Naprox, "U("+t+").Alloc");
                }
                
                controls[t] = new LinearControl(model.cplex, approach.N()){
                    @Override
                    public IloNumExpr delta() throws Exception {
                        return model.cplex.sum(In);
                    }
                    @Override
                    public IloNumExpr risk() throws Exception {
                        return model.cplex.sum(c);
                    }
                    @Override
                    public int FRT() throws Exception {
                        return In.length;
                    }
                };
                for(int n=0; n<2*approach.N(); n++){
                    int i = n/2;
                    double a = 1-(n%2)*2;
                    model.cplex.addLe(
                        model.cplex.sum(model.cplex.prod(a, controls[t].u[i]), c[n]), +approach.maxControl()
                    );
                }
            }
            return controls;
        }
    }
    
    private class DetNorm2UtUB extends oLinearControlBound<LinearSystem, LinearModel>{
        private final int N = 16;
        @Override
        public String name() {
            return "DetNorm2(Ut)";
        }
        @Override
        public LinearControl[] build_controls(LinearSystem approach, final LinearModel model) throws Exception {
            LinearControl controls[] = new LinearControl[approach.Waypoints()];
            for(int t=0; t<approach.Waypoints(); t++){
                controls[t] = new LinearControl(model.cplex, approach.N()){
                    @Override
                    public IloNumExpr delta() throws Exception {
                        return null;
                    }
                    @Override
                    public IloNumExpr risk() throws Exception {
                        return null;
                    }
                    @Override
                    public int FRT() throws Exception {
                        return 0;
                    }
                };
                for(int i=0; i<approach.N(); i++){
                    controls[t].u[i].setLB(-approach.maxControl());
                    controls[t].u[i].setUB(+approach.maxControl());
                }
                
                for(int n=0; n<N; n++){ //only ux,uy
                    IloNumExpr exp = null;
                
                    IloNumVar ux = controls[t].u[0];
                    exp = model.cplex.SumProd(exp, Math.cos((2*Math.PI*n)/N), ux);
                    
                    IloNumVar uy = controls[t].u[1];
                    exp = model.cplex.SumProd(exp, Math.sin((2*Math.PI*n)/N), uy);
                    
                    model.cplex.addLe(exp, approach.maxControl(), "Umax");
                }
                
            }
            return controls;
        }
    }
    
    private class UncNorm2UtUB extends oLinearControlBound<LinearSystem, LinearModel>{
        private final int N = 16;
        private pLinearControlUncertainty unc;
        private Delta delta;
        private int Naprox;
        @Override
        public String name() {
            return "UncNorm2(Ut)";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            unc = link.need(pLinearControlUncertainty.class, unc);
            delta = link.need(Delta.class, delta);
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            Naprox  = link.Int("Control-N-risk", 12, 1, 64, 
                    "Number of picewise restriction to aproximate the "+
                    "inverse of erf(x) to calculate the risk in Norm1 control upper bound");
        }
        @Override
        public LinearControl[] build_controls(LinearSystem approach, final LinearModel model) throws Exception {
            if(approach.N()!=2){
                throw new Exception("The class "+this.class_name()+" needs a 2D map");
            }
            
            LinearControl controls[] = new LinearControl[approach.Waypoints()];
            for(int t=0; t<approach.Waypoints(); t++){
                final IloNumVar In[] = model.cplex.numVarArray(N, 0, delta.Delta(), "U("+t+").Risk");
                
                final IloNumExpr c[] = new IloNumExpr[N];
                for(int n=0; n<N; n++){ //only ux,uy
                    double a[] = new double[]{Math.cos((2*Math.PI*n)/N), Math.sin((2*Math.PI*n)/N)};
                    double uncertainty = Math.sqrt(2*(unc.sigma(t, a)));
                    c[n] = model.cplex.RiskAllocation(In[n], uncertainty, delta.Delta(), Naprox, "U("+t+").Alloc");
                }
                
                controls[t] = new LinearControl(model.cplex, approach.N()){
                    @Override
                    public IloNumExpr delta() throws Exception {
                        return model.cplex.sum(In);
                    }
                    @Override
                    public IloNumExpr risk() throws Exception {
                        return model.cplex.sum(c);
                    }
                    @Override
                    public int FRT() throws Exception {
                        return In.length;
                    }
                };
//                for(int n=0; n<2*approach.N(); n++){
//                    int i = n/2;
//                    double a = 1-(n%2)*2;
//                    model.cplex.addLe(
//                        model.cplex.sum(model.cplex.prod(a, controls[t].u[i]), c[n]), +approach.maxControl()
//                    );
//                }
                
                for(int i=0; i<approach.N(); i++){ //ux,uy,uz,...
                    controls[t].u[i].setLB(-approach.maxControl());
                    controls[t].u[i].setUB(+approach.maxControl());
                }
                
                for(int n=0; n<N; n++){ //only vx,vy
                    IloNumExpr exp = c[n];
                
                    IloNumVar ux = controls[t].u[0];
                    exp = model.cplex.SumProd(exp, Math.cos((2*Math.PI*n)/N), ux);
                    
                    IloNumVar uy = controls[t].u[1];
                    exp = model.cplex.SumProd(exp, Math.sin((2*Math.PI*n)/N), uy);
                    
                    model.cplex.addLe(exp, approach.maxControl(), "Umax");
                }
            }
            return controls;
        }
    }
    
    /**
     * This approach represent that only one variable delta is needed for each time of control for all dimensions
     */
    private class UncNorm2UtUB_deltaU extends oLinearControlBound<LinearSystem, LinearModel>{
        private final int N = 16;
        private pLinearControlUncertainty unc;
        private Delta delta;
        private int Naprox;
        @Override
        public String name() {
            return "UncNorm2(Ut)(fortalecimento 6)";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            unc = link.need(pLinearControlUncertainty.class, unc);
            delta = link.need(Delta.class, delta);
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            Naprox  = link.Int("Control-N-risk", 12, 1, 64, 
                    "Number of picewise restriction to aproximate the "+
                    "inverse of erf(x) to calculate the risk in Norm1 control upper bound");
        }
        @Override
        public LinearControl[] build_controls(final LinearSystem approach, final LinearModel model) throws Exception {
            if(approach.N()!=2){
                throw new Exception("The class "+this.class_name()+" needs a 2D map");
            }
            
            LinearControl controls[] = new LinearControl[approach.Waypoints()];
            for(int t=0; t<approach.Waypoints(); t++){
                final IloNumVar In = model.cplex.numVar(0, delta.Delta(), "U("+t+").Risk");
                final IloNumExpr erf_inv = model.cplex.ERF_Inv(In, delta.Delta(), 2.0, N, "U("+t+").erf_inv");
                final IloNumExpr c[] = new IloNumExpr[N];
                
                for(int n=0; n<N; n++){ //only ux,uy
                    double a[] = new double[]{Math.cos((2*Math.PI*n)/N), Math.sin((2*Math.PI*n)/N)};
                    double uncertainty = Math.sqrt(2*(unc.sigma(t, a)));
                    
                    //c[n] = model.cplex.RiskAllocation(In, uncertainty, delta.Delta(), Naprox, "U("+t+").Alloc");
                    c[n] = model.cplex.RiskAllocation(erf_inv, uncertainty);
                }
                
                controls[t] = new LinearControl(model.cplex, approach.N()){
                    @Override
                    public IloNumExpr delta() throws Exception {
                        return In;
                    }
                    @Override
                    public IloNumExpr risk() throws Exception {
                        return model.cplex.sum(c);
                    }
                    @Override
                    public int FRT() throws Exception {
                        return approach.N();
                    }
                };

                for(int i=0; i<approach.N(); i++){ //ux,uy,uz,...
                    controls[t].u[i].setLB(-approach.maxControl());
                    controls[t].u[i].setUB(+approach.maxControl());
                }
                
                for(int n=0; n<N; n++){ //only vx,vy
                    IloNumExpr exp = c[n];
                
                    IloNumVar ux = controls[t].u[0];
                    exp = model.cplex.SumProd(exp, Math.cos((2*Math.PI*n)/N), ux);
                    
                    IloNumVar uy = controls[t].u[1];
                    exp = model.cplex.SumProd(exp, Math.sin((2*Math.PI*n)/N), uy);
                    
                    model.cplex.addLe(exp, approach.maxControl(), "Umax");
                }
            }
            return controls;
        }
    }
}