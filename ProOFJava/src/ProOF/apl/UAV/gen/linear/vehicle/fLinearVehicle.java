/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.vehicle;

import ProOF.apl.UAV.gen.linear.LinearControl;
import ProOF.apl.UAV.gen.linear.LinearState;
import ProOF.apl.UAV.gen.linear.LinearApproach;
import ProOF.apl.UAV.gen.linear.LinearModel;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.apl.UAV.gen.linear.vehicle.parts.fLinearControlBound;
import ProOF.apl.UAV.gen.linear.vehicle.parts.fLinearDynamic;
import ProOF.apl.UAV.gen.linear.vehicle.parts.fLinearSpecific;
import ProOF.apl.UAV.gen.linear.vehicle.parts.fLinearStateBound;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearControlBound;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearSpecific;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearStateBound;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.language.Factory;
import ilog.concert.IloNumExpr;

/**
 *
 * @author marcio
 */
public class fLinearVehicle extends Factory<oLinearVehicle>{
    public static final fLinearVehicle obj = new fLinearVehicle();
    @Override
    public String name() {
        return "Vehicle";
    }
    @Override
    public oLinearVehicle build(int index) {  //build the operators
        switch(index){
            case 0: return new PartsVehicle();
            case 1: return new FullVehicle();
        }
        return null;
    }
    private class PartsVehicle extends oLinearVehicle<LinearSystem, LinearModel>{
        private oLinearStateBound state;
        private oLinearControlBound control;
        private oLinearDynamic dynamic;
        private oLinearSpecific specific;
        
        @Override
        public String name() {
            return "Parts";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            state = link.get(fLinearStateBound.obj, state);
            control = link.get(fLinearControlBound.obj, control);
            dynamic = link.get(fLinearDynamic.obj, dynamic);
            specific = link.get(fLinearSpecific.obj, specific);
        }
        @Override
        public void start(LinearSystem approach) throws Exception {
            dynamic.start(approach); 
        }
        @Override
        public LinearState[] build_states(LinearSystem approach, LinearModel model) throws Exception {
            return state.build_states(approach, model);
        }
        @Override
        public LinearControl[] build_controls(LinearSystem approach, LinearModel model) throws Exception {
            return control.build_controls(approach, model);
        }
        @Override
        public void addConstraints(LinearSystem approach, LinearModel model) throws Exception {
            dynamic.addConstraints(approach, model);
            specific.addConstraints(approach, model);
        }
    }
    private class FullVehicle extends oLinearVehicle<LinearApproach, LinearModel>{
        private double A[][];
        private double B[][];
    
        @Override
        public String name() {
            return "FullPV";
        }
        @Override
        public void start(LinearApproach approach) throws Exception {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }
        @Override
        public LinearState[] build_states(LinearApproach approach, LinearModel model) throws Exception {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public LinearControl[] build_controls(LinearApproach approach, LinearModel model) throws Exception {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public void addConstraints(LinearApproach approach, LinearModel model) throws Exception {
            for(int t=0; t<model.controls.length; t++){
                for(int i=0; i<A.length; i++){
                    IloNumExpr exp = null;
                    for(int j=0; j<A[i].length; j++){
                        exp = model.cplex.SumProd(exp, A[i][j], model.states[t].x[j]);
                    }
                    for(int j=0; j<B[i].length; j++){
                        exp = model.cplex.SumProd(exp, B[i][j], model.controls[t].u[j]);
                    }
                    model.cplex.addEq(model.states[t+1].x[i], exp, "DynamicLinear["+(i+1)+"]");
                }
            }
        }
        
        
        public void print(){
            System.out.println("-------------------------[A]-------------------------");
            for(int i=0; i<A.length; i++){
                for(int j=0; j<A[i].length; j++){
                    System.out.printf("%8.3f ", A[i][j]);
                }
                System.out.println();
            }
            System.out.println("-------------------------[B]-------------------------");
            for(int i=0; i<B.length; i++){
                for(int j=0; j<B[i].length; j++){
                    System.out.printf("%8.3f ", B[i][j]);
                }
                System.out.println();
            }
        }
    }
}
