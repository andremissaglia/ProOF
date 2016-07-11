/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.mission.parts;

import ProOF.apl.UAV.gen.linear.LinearApproach;
import ProOF.apl.UAV.gen.linear.LinearModel;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.apl.UAV.gen.linear.vehicle.parts.pLinearWaypoints;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.language.Factory;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;

/**
 *
 * @author marcio
 */
public class fLinearObjective extends Factory<oLinearObjective>{
    public static final fLinearObjective obj = new fLinearObjective();
    @Override
    public String name() {
        return "Objective";
    }
    @Override
    public oLinearObjective build(int index) {  //build the operators
        switch(index){
            case 0: return new Norm1Ut();
            case 1: return new Norm2Ut2D();
            case 2: return new ScalProdUtAprox();  
            case 3: return new ScalProdUtCplex();
        }
        return null;
    }
    public static class Norm1Ut extends oLinearObjective<LinearSystem, LinearModel>{
        private pLinearWaypoints waypoint;
        @Override
        public String name() {
            return "|u|*dt";
        }

        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            waypoint = link.need(pLinearWaypoints.class, waypoint);
        }
        @Override
        public void addObjective(LinearSystem approach, LinearModel model) throws Exception {
            IloNumExpr obj = null;
            for (int t = 0; t < approach.Waypoints(); t++) {
                obj = model.cplex.SumNumNorm1(obj, "|u|", approach.maxControl(), waypoint.dt(), model.controls[t].u);
            }
            model.cplex.addMinimize(obj);
        }
    }
    public static class Norm2Ut2D extends oLinearObjective<LinearSystem, LinearModel>{
        private pLinearWaypoints waypoint;
        @Override
        public String name() {
            return "||u||*dt";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            waypoint = link.need(pLinearWaypoints.class, waypoint);
        }
        @Override
        public void addObjective(LinearSystem approach, LinearModel model) throws Exception {
            IloNumVar UtNorm[][] = model.cplex.numVarArray(approach.Waypoints(), approach.N(), 0, Double.POSITIVE_INFINITY, "Ut.norm");
            //------------------------ (NOM-2)  -----------------------------
            for (int t = 0; t < approach.Waypoints(); t++) {
                for (int j = 0; j < approach.N(); j++) {
                    model.cplex.addGe(UtNorm[t][j], model.cplex.prod(+1, model.controls[t].u[j]));
                    model.cplex.addGe(UtNorm[t][j], model.cplex.prod(-1, model.controls[t].u[j]));
                }
            }
            IloNumExpr obj = null;
            for (int t = 0; t < approach.Waypoints(); t++) {
                obj = model.cplex.SumNumNorm2_2D(obj, "‖ux+uy‖", 32, approach.maxControl(), waypoint.dt(), model.controls[t].u[0], model.controls[t].u[1]);
                if(approach.N()==3){
                    obj = model.cplex.SumNumNorm1(obj, "|uz|", approach.maxControl(), waypoint.dt(), model.controls[t].u[2]);
                }
            }
            model.cplex.addMinimize(obj);
        }
    }
    
    public static class ScalProdUtAprox extends oLinearObjective<LinearSystem, LinearModel>{
        private pLinearWaypoints waypoint;
        private int Naprox;
        @Override
        public String name() {
            return "aprox{ u*u }*dt";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            waypoint = link.need(pLinearWaypoints.class, waypoint);
        }

        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            Naprox = link.Int("N-aprox", 32, 1, 128);
        }
        @Override
        public void addObjective(LinearSystem approach, LinearModel model) throws Exception {
            IloNumExpr obj = null;
            //System.out.println("maxControl = "+approach.maxControl());
            for (int t = 0; t < approach.Waypoints(); t++) {
                obj = model.cplex.SumNumScalProd(obj, "u*u", Naprox, approach.maxControl(), waypoint.dt(), model.controls[t].u);
            }
            model.cplex.addMinimize(obj);
        }
    }
    public static class ScalProdUtCplex extends oLinearObjective<LinearSystem, LinearModel>{
        private pLinearWaypoints waypoint;
        @Override
        public String name() {
            return "cplex{ u*u }*dt";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            waypoint = link.need(pLinearWaypoints.class, waypoint);
        }
        @Override
        public void addObjective(LinearSystem approach, LinearModel model) throws Exception {
            IloNumExpr obj = null;
            for (int t = 0; t < approach.Waypoints(); t++) {
                obj = model.cplex.Sum(obj, model.cplex.prod(waypoint.dt(), model.cplex.scalProd(model.controls[t].u, model.controls[t].u)));
            }
            model.cplex.addMinimize(obj);
        }
    }
}
