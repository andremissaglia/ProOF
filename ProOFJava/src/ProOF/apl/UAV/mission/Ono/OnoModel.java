/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.apl.UAV.gen.linear.LinearParticleControl;
import ProOF.CplexExtended.CplexExtended;
import ProOF.CplexExtended.CplexPrinter;
import ProOF.CplexExtended.iCplexExtract;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.gen.linear.LinearControl;
import ProOF.apl.UAV.gen.linear.LinearModel;
import ProOF.apl.UAV.gen.linear.LinearState;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.apl.UAV.mission.Ono.OnoInstance.Episode;
import ProOF.apl.UAV.mission.Ono.OnoInstance.NonConvex;
import ProOF.apl.UAV.mission.Ono.OnoInstance.TemporalConstraints;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import java.awt.BasicStroke;
import java.awt.Color;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.LinkedList;

/**
 *
 * @author marcio
 */
public abstract class OnoModel extends LinearModel<OnoApproach>{
    protected final IloNumVar Ke[];
    protected final IloNumVar Te[];
    private final IloNumVar[][] Yat;
    //private final ArrayList<IloIntVar> obstacles = new ArrayList<IloIntVar>();
    //private final ArrayList<IloIntVar> waypoints = new ArrayList<IloIntVar>();
    //private final ArrayList<ArrayList<IloNumVar[][]> > sets_ti = new ArrayList<>();
    protected final ArrayList<CplexPrinter> printers = new ArrayList<CplexPrinter>();
    public OnoModel(OnoApproach approach, String name, CplexExtended cplex) throws IloException, Exception {
        super(approach, name, cplex);
        if(system.strengthened[6]){
            System.out.println("fortalecimento - [implies] parte 1");
            Ke = cplex.numVarArray(approach.inst.n_events-1, 0, Double.POSITIVE_INFINITY, "Ke");
        }else{
            Ke = cplex.intVarArray(approach.inst.n_events-1, 0, Integer.MAX_VALUE, "Ke");
        }
        Te = cplex.numVarArray(approach.inst.n_events, 0, Double.MAX_VALUE, "Te");
        Yat = cplex.numVarArray(approach.inst.episodes.length, approach.Waypoints()+1, 0, 1, "Yat");
        
        //cplex.addMinimize(cplex.minus(Te[approach.inst.n_events-1], cplex.prod(10,Ke[4])));
        approach.objective.addObjective(approach, this);
    }
    public abstract void addWaypoint(Obstacle obs, int t, int c, IloNumExpr YC) throws Exception ;
    public abstract void addObstacle(Obstacle obs, int t, int c, IloNumExpr Z[], boolean det) throws Exception;

    
    public final void addConstraints(oOnoAvoidance avoid) throws Exception{
        //---------------------- Start state ---------------------------------
        for(int i=0; i<system.inst.start_state.length; i++){
            int t = 0;
            states[t].x[i].setLB(system.inst.start_state[i]);
            states[t].x[i].setUB(system.inst.start_state[i]);
        
            //end state
//            t = system.Waypoints();
//            states[t].x[i].setLB(system.inst.start_state[i] + (i==0 ? +24 : 0));
//            states[t].x[i].setUB(system.inst.start_state[i] + (i==0 ? +24 : 0));
        }
        
        //system.strengthened[0] = true;
        
        //---------------------- Time Fluxe ---------------------------------
        Te[0].setUB(0);
        for(int e=0; e<system.inst.n_events-1; e++){
            cplex.addEq(Te[e+1], cplex.sum(Te[e],cplex.prod(system.dt(), Ke[e])), "TimeFluxe");
        }
 
        //---------------------- Temporal constraints ---------------------------------
        for(TemporalConstraints r : system.inst.temporal_constraints){
            if(r.eE == r.eS+1){
                Ke[r.eS].setUB((int) (r.ub/system.dt()+0.000001));
                Ke[r.eS].setLB((int) (r.lb/system.dt()+0.999999));
            }else{
                IloNumExpr exp = cplex.minus(Te[r.eE], Te[r.eS]);
                //cplex.addLe(exp, r.ub ,"T.ub");
                //cplex.addGe(exp, r.lb ,"T.lb");
                cplex.addRange(r.lb, exp, r.ub, "T");
            }
        }
        //------------ Melhoria dos limitantes de Te e Ke utilizando o tempo maximo de planejamento -----------
        for(TemporalConstraints r : system.inst.temporal_constraints){
            if(r.eS==0 && r.eE==system.inst.n_events-1){  //link the frist event to the last one
                Te[r.eE].setUB(Math.min(Te[r.eE].getUB(), r.ub));   //set max time to plan
                for(int e=0; e<system.inst.n_events-1; e++){
                    Ke[e].setUB(Math.min(Ke[e].getUB(), (int) (r.ub/system.dt()+0.000001)));
                }
            }
        }
        
        //---------------------- Episodios ---------------------------------
        System.out.println("T  = "+system.Waypoints());
        System.out.println("dt = "+system.dt());
        System.out.println("V  = "+system.maxVelocity());
        System.out.println("U  = "+system.maxControl());
        for(boolean v : system.strengthened){
            System.out.println("streng = "+v);
        }
        
        //----- Capitura intantes dos episódios START, REMAIN e END --------
        System.out.println("----- Capitura intantes dos episódios START, REMAIN e END --------");
        for(Episode ep : system.inst.episodes){
            switch (ep.type) {
                case Episode.START_IN:
                    // Start-In:
                    for(int t=0; t<system.Waypoints()+1; t++){
                        IloNumExpr exp = cplex.sum(Te[ep.eS],   -t*system.dt());
                        if(system.strengthened[6]){
                            System.out.println("fortalecimento - [implies] parte 2 (start)");
                            // Yat ==> Te(Start) = t*dt 
                            cplex.addIF_Y_Them_Eq("Y", Yat[ep.index][t], exp);
                        }else{
                            // Te(Start) = t*dt  ==>  Yat
                            cplex.addIF_Eq_Them_Y("Y", Yat[ep.index][t], exp);
                        }
                    }   break;
                case Episode.REMAIN_IN:
                    // Remain-In:
                    if(system.strengthened[2] && ep.eS==0 && ep.eE==system.inst.n_events-1){
                        System.out.println("fortalecimento - 3 parte 1");
                    }else{
                        for(int t=0; t<system.Waypoints()+1; t++){
                            if(system.strengthened[6]){
                                System.out.println("fortalecimento - [implies] parte 3 (remain)");
                                // Yat ==> Te(Start) <= t*dt <= Te(End) 
                                //cplex.addIF_Y_Them_Eq("Y", Yat[ep.index][t], exp);
                                cplex.addIF_Y_Them_Between("Y", Yat[ep.index][t], Te[ep.eS], t*system.dt(), Te[ep.eE]);
                               
                                cplex.addIF_Between_Them_Y("Y", Yat[ep.index][t], Te[ep.eS], t*system.dt(), Te[ep.eE]);
                                cplex.addEq(cplex.sum(Yat[ep.index]), Ke[ep.eS], "Y_Ke");
                            }else{
                                // Te(Start) <= t*dt <= Te(End) ==>  Yat
                                cplex.addIF_Between_Them_Y("Y", Yat[ep.index][t], Te[ep.eS], t*system.dt(), Te[ep.eE]);
                            }
                        }
                    }   break;
                case Episode.END_IN:
                    // End-In:
                    for(int t=0; t<system.Waypoints()+1; t++){
                        IloNumExpr exp = cplex.sum(Te[ep.eE],   -t*system.dt());
                        if(system.strengthened[6]){
                            System.out.println("fortalecimento - [implies] parte 4 (end)");
                            // Yat ==> Te(End) = t*dt 
                            cplex.addIF_Y_Them_Eq("Y", Yat[ep.index][t], exp);
                            //cplex.addIF_Eq_Them_Y("Y", Yat[ep.index][t], exp);
                        }else{
                            // Te(End) = t*dt  ==>  Yat
                            cplex.addIF_Eq_Them_Y("Y", Yat[ep.index][t], exp);
                        }
                    }   break;
                default:
                    throw new Exception("episode type = '"+ep.type+"' doesn't exist");
            }
        }
        
        System.out.println("---------------------- Pontos de passagem ------------------------");
        
        //---------------------- Pontos de passagem ------------------------
//        for(Episode ep : system.inst.episodes){
//            int c = ep.c;
//            if(ep.ra.I.size()>0){
//                for(int t=0; t<system.Waypoints()+1; t++){
//                    IloNumVar YN[];
//                    if(system.strengthened[3]){
//                        System.out.println("fortalecimento - 5 parte 1");
//                        YN = cplex.numVarArray(ep.ra.I.size(), 0, 1, "YI");
//                    }else{
//                        YN = cplex.boolVarArray(ep.ra.I.size(), "YI");
//                    }
//                    int j = 0;
//                    for(NonConvex non_convex : ep.ra.I){
//                        IloIntVar YC[] = cplex.boolVarArray(non_convex.C.size(), "YC");
//                        int k=0;
//                        for(int index: non_convex.C){
//                            Obstacle obs = system.inst.regions[index];
//
//                            //waypoints.add(YC[k]);
//                            
//                            addWaypoint(obs, t, c, YC[k]);
//                            
//                            k++;
//                        }
//                        if(system.strengthened[3]){
//                            System.out.println("fortalecimento - 5 parte 2");
//                            cplex.addEq(YN[j], cplex.sum(YC), "I.sum(YC)");
//                        }else{
//                            cplex.addLe(YN[j], cplex.sum(YC), "I.sum(YC)");
//                        }
//                        
//                        j++;
//                    }
//                    if(system.strengthened[3]){
//                        System.out.println("fortalecimento - 5 parte 3");
//                        cplex.addEq(Yat[ep.index][t], cplex.sum(YN), "I.sum(YN)");
//                    }else{
//                        cplex.addLe(Yat[ep.index][t], cplex.sum(YN), "I.sum(YN)");
//                    }
//                    
//                }
//            }
//        }

        for(Episode ep : system.inst.episodes){
            int c = ep.c;
            if(ep.ra.I.size()>0){
                
                IloNumVar YNt[][];
                if(system.strengthened[3]){
                    System.out.println("fortalecimento - 5 parte 1");
                    YNt = cplex.numVarArray(system.Waypoints()+1, ep.ra.I.size(), 0, 1, "YNt");
                }else{
                    YNt = cplex.boolVarArray(system.Waypoints()+1, ep.ra.I.size(), "YNt");
                }
                int j = 0;
                for(NonConvex non_convex : ep.ra.I){
                    IloIntVar YCt[][] = cplex.boolVarArray(system.Waypoints()+1, non_convex.C.size(), "YCt");
                    int k=0;
                    for(int index: non_convex.C){
                        Obstacle obs = system.inst.regions[index];

                        //waypoints.add(YC[k]);
                        for(int t=0; t<system.Waypoints()+1; t++){
                            addWaypoint(obs, t, c, YCt[t][k]);
                        }
                        printers.add(new CplexPrinter() {
                            @Override
                            public void printer(iCplexExtract ext) throws IloException {
                                double vYCt[][] = ext.getValues(YCt, true);
                                ext.print("way["+index+"]", "-%5.2f", vYCt);
                            }
                        });

                        k++;
                    }
                    if(system.strengthened[3]){
                        System.out.println("fortalecimento - 5 parte 2");
                        for(int t=0; t<system.Waypoints()+1; t++){
                            cplex.addEq(YNt[t][j], cplex.sum(YCt[t]), "I.sum(YC)");
                        }
                    }else{
                        for(int t=0; t<system.Waypoints()+1; t++){
                            cplex.addLe(YNt[t][j], cplex.sum(YCt[t]), "I.sum(YC)");
                        }
                    }
                    j++;
                }
                if(system.strengthened[3]){
                    System.out.println("fortalecimento - 5 parte 3");
                    for(int t=0; t<system.Waypoints()+1; t++){
                        cplex.addEq(Yat[ep.index][t], cplex.sum(YNt[t]), "I.sum(YN)");
                    }
                }else{
                    for(int t=0; t<system.Waypoints()+1; t++){
                        cplex.addLe(Yat[ep.index][t], cplex.sum(YNt[t]), "I.sum(YN)");
                    }
                }
            }
        }

        System.out.println("--------------------------- Obstaculos ---------------------------");
        
        //--------------------------- Obstaculos ---------------------------
        LinkedList<IloNumVar> list = new LinkedList<>();        
        for(Episode ep : system.inst.episodes){
            int c = ep.c;
            if(ep.ra.O.size()>0){
                for(NonConvex non_convex : ep.ra.O){
                    for(int index: non_convex.C){
                        Obstacle obs = system.inst.regions[index];
                        IloNumVar Zt[][];
                        if(system.strengthened[7]){
                            System.out.println("fortalecimento - [bin St] parte 1");
                            if(system.strengthened[8]){
                                System.out.println("fortalecimento - [step] parte 1");
                                Zt = cplex.numVarArray(system.Steps()+1, obs.Gj(), 0, 1, "Z");
                            }else{
                                Zt = cplex.numVarArray(system.Waypoints()+1, obs.Gj(), 0, 1, "Z");
                            }
                        }else{
                            if(system.strengthened[8]){
                                System.out.println("fortalecimento - [step] parte 1");
                                Zt = cplex.boolVarArray(system.Steps(), obs.Gj(), "Z");
                            }else{
                                Zt = cplex.boolVarArray(system.Waypoints()+1, obs.Gj(), "Z");
                            }
                        }
                        
                        if(system.strengthened[5]){
                            System.out.println("fortalecimento - [swaps] parte 1");
                            IloNumVar St[][];
                            if(system.strengthened[7]){
                                System.out.println("fortalecimento - [bin St] parte 1");
                                St = cplex.boolVarArray(system.Waypoints(), obs.Gj(), "S");
                                for(int t=0; t<system.Waypoints(); t++){
                                    for(int i=0; i<obs.Gj(); i++){
                                        //Z(t+1) >= St
                                        cplex.addGe(Zt[t+1][i], St[t][i]);
                                        //Zt <= 1 - St
                                        cplex.addLe(Zt[t][i], cplex.sum(1, cplex.prod(-1,St[t][i])));
                                    }
                                    IloNumExpr sum = null;
                                    for(int i=0; i<obs.Gj(); i++){
                                        sum = cplex.SumProd(sum, 1, St[t][i]);
                                    }
                                    cplex.addLe(sum, 1, "sum St");
                                    for(int i=0; i<obs.Gj(); i++){
                                        //[And_i St ] = 0 ==> Z(t+1) = Zt
                                        cplex.addIF_Y_Them_Eq("eq", -1, +1, 0, cplex.Not(sum), cplex.minus(Zt[t+1][i], Zt[t][i]));
                                    }
                                }
                            }else{
                                if(system.strengthened[8]){
                                    System.out.println("fortalecimento - [step] parte 3");
                                    St = cplex.numVarArray(system.Steps()-1, obs.Gj(), 0, 1, "S");
                                    for(int s=0; s<system.Steps()-1; s++){
                                        for(int i=0; i<obs.Gj(); i++){
                                            cplex.addGe(St[s][i], cplex.minus(Zt[s+1][i], Zt[s][i]));
                                        }
                                    }
                                }else{
                                    St = cplex.numVarArray(system.Waypoints(), obs.Gj(), 0, 1, "S");
                                    for(int t=0; t<system.Waypoints(); t++){
                                        for(int i=0; i<obs.Gj(); i++){
                                            cplex.addGe(St[t][i], cplex.minus(Zt[t+1][i], Zt[t][i]));
                                        }
                                    }
                                }
                            }
                            
                            printers.add(new CplexPrinter() {
                                @Override
                                public void printer(iCplexExtract ext) throws IloException {
                                    double vZt[][] = ext.getValues(Zt, true);
                                    double vSt[][] = ext.getValues(St, true);
                                    ext.print("obs["+index+"]: Zt | St", "-%5.2f", vZt, vSt);
                                }
                            });
                            
                            int h = obs.indexNear(system.inst.start_state);
                            Zt[0][h].setLB(1);
                            System.out.println("index "+index+" = " +h);
                            for(int k=0; k<obs.Gj(); k++){
                                if(k!=h){
                                    Zt[0][k].setUB(0);
                                }
                            }
                            for(int t=0; t<St.length; t++){
                                for(int i=0; i<obs.Gj(); i++){
                                    list.addLast(St[t][i]);
                                }
                            }
                            
                            
//                            IloNumExpr sum = null;
//                            
//                            for(int t=0; t<system.Waypoints(); t++){
//                                for(int i=0; i<obs.Gj(); i++){
//                                    sum = cplex.SumProd(sum, 1, St[t][i]);
//                                }
//                            }
//
//                            //IloNumVar swap = cplex.numVar(0, obs.Gj(), "swap");
//                            IloNumVar swap = cplex.numVar(0, Double.POSITIVE_INFINITY, "swap");
//                            cplex.addLe(sum, swap);
//                            
//                            list.addLast(swap);
                            
//                            for(int t=1; t<system.Waypoints()+1; t++){
//                                addObstacle(obs, t, c, St[t-1], false);
//                            }
//                            for(int t=0; t<system.Waypoints()+1; t++){
//                                addObstacle(obs, t, c, Zt[t], true);
//                            }
                        }
                        if(system.strengthened[8]){
                            System.out.println("fortalecimento - [step] parte 3");
                            for(int t=0; t<system.Waypoints()+1; t++){
                                int s = system.Step(t);
                                System.out.println(" s = "+s+" -> t = "+t);
                                addObstacle(obs, t, c, Zt[s], false);
                                if(t<system.Waypoints() && system.Step(t+1)>s){ //has change
                                    System.out.println("*s = "+s+" -> t = "+(t+1));
                                    addObstacle(obs, t+1, c, Zt[s], false);
                                }
                            }
//                            System.out.println("fortalecimento - [step] parte 4");
//                            for(int s=0; s<system.Steps(); s++){
//                                int t = system.Waypoint(s);
//                                System.out.println("s = "+s+" -> t = "+(t-1));
//                                addObstacle(obs, t-1, c, Zt[s], false);
//                            }
                            for(int t=0; t<system.Waypoints()+1; t++){
                                int s = system.Step(t);
                                cplex.addEq(cplex.sum(Zt[s]), Yat[ep.index][t], "O.sum(Z)");
                            }
//                            for(int s=0; s<system.Steps(); s++){
//                                cplex.addEq(cplex.sum(Zt[s]), 1, "O.sum(Z)");
//                            }
                            
//                            for(int t=0; t<system.Waypoints()+1; t++){
//                                int s = system.Step(t);
//                                avoid.N(system, this, Yat[ep.index][t], Zt, s);
//                            }
                        }else{
                            for(int t=0; t<system.Waypoints()+1; t++){
                                addObstacle(obs, t, c, Zt[t], false);
                            }
                            for(int t=0; t<system.Waypoints()+1; t++){
                                avoid.N(system, this, Yat[ep.index][t], Zt, t);
                            }
                        }
                    }
                }
            }
        }
        if(system.strengthened[5]){
            System.out.println("fortalecimento - [swaps] parte 2");
            int J = 0;
            for(Episode ep : system.inst.episodes){
                for(NonConvex n : ep.ra.O){
                    J += n.C.size();
                }
            }
            System.out.println("Swaps = "+2*J);
            cplex.addLe(cplex.sum(list.toArray(new IloNumExpr[list.size()])), 2*J);
        }
        System.out.println("--------------------------- Fortalecimentos ---------------------------");
        
        //---------------------- Fortalecimentos ---------------------------------
        
        for(Episode ep : system.inst.episodes){
            if(ep.type==Episode.START_IN){ // Start-In:
                if(system.strengthened[0]){
                    System.out.println("fortalecimento - 1 parte 1");
                    cplex.addEq(cplex.sum(Yat[ep.index]), 1, "sum(Yat|S)=1");
                }
            }else if(ep.type==Episode.REMAIN_IN){ // Remain-In:
                if(system.strengthened[2] && ep.eS==0 && ep.eE==system.inst.n_events-1){
                    System.out.println("fortalecimento - 3 parte 2");
                    for(int t=0; t<system.Waypoints()+1; t++){
                        Yat[ep.index][t].setLB(1);
                        Yat[ep.index][t].setUB(1);
                    }
                }else if(system.strengthened[0]){
                    boolean added = false;
                    for(TemporalConstraints r : system.inst.temporal_constraints){
                        if(ep.eS == r.eS && ep.eE == r.eE){
                            int min = Math.max(1, (int) (r.lb/system.dt()+0.999999));
                            int max = Math.min(system.Waypoints(), (int) (r.ub/system.dt()+0.000001));
                            if(min>max){
                                throw new Exception("episode name = '"+ep.name+"' the temporal constraint ["+r.lb+","+r.ub+"] implies ["+min+"<="+max+"] false");
                            }else if(min==max){
                                System.out.println("fortalecimento - 1 parte 3");
                                cplex.addEq(cplex.sum(Yat[ep.index]), min, "(lb|ub)/dt==sum(Yat|R)");
                            }else if(max==system.Waypoints()){
                                System.out.println("fortalecimento - 1 parte 4");
                                cplex.addGe(cplex.sum(Yat[ep.index]), min, "lb/dt<=sum(Yat|R)");
                            }else{
                                System.out.println("fortalecimento - 1 parte 5");
                                cplex.addRange(min, cplex.sum(Yat[ep.index]), max, "lb/dt<=sum(Yat|R)<=ub/dt");
                            }
                            added = true;
                            break;
                        }
                    }
                    if(!added){
                        System.out.println("fortalecimento - 1 parte 2");
                        cplex.addGe(cplex.sum(Yat[ep.index]), 1, "sum(Yat|R)>=1");
                    }
                }
            }else if(ep.type==Episode.END_IN){ // End-In:
                if(system.strengthened[0]){
                    System.out.println("fortalecimento - 1 parte 3");
                    cplex.addEq(cplex.sum(Yat[ep.index]), 1, "sum(Yat|E)=1");
                }
            }else{
                throw new Exception("episode type = '"+ep.type+"' doesn't exist");
            }
        }
    }
    
    @Override
    public boolean addChanges() throws Exception {
        return false;
    }
    
    public final double[][] getXt() throws IloException {
        double [][] vXt = new double[system.Waypoints()+1][];
        for(int t=0; t<system.Waypoints()+1; t++){
            vXt[t] = cplex.getValues(states[t].x);
        }
        return vXt;
    }
    private int end;
    private LinearParticleControl crt[];
    @Override
    public void extract(iCplexExtract ext, Callback type) throws IloException, Exception {
        super.extract(ext, type); //To change body of generated methods, choose Tools | Templates.
        double vTe[] = ext.getValues(Te, true);
        end = (int)(vTe[vTe.length-1]/system.dt() + 0.5);
        end  = Math.min(end, system.Waypoints());
        end  = system.Waypoints();
        //System.out.println("end = "+end + " vTe = "+vTe[vTe.length-1]+" dt = "+system.dt());
        //System.out.println(type.toString() + " -> "+ isFeasible());
        if(type == Callback.FEASIBLE || type == Callback.OPTIMAL){
            crt = new LinearParticleControl[(int)(1.0/system.Delta())];
            for(int n=0; n<crt.length; n++){
                crt[n] = new LinearParticleControl(system.unc, system.dynamic, states, controls);
            }
            double vKe[] = ext.getValues(Ke, true);
            double vYat[][] = ext.getValues(Yat, true);
            ext.print("Ke", "%5.2f", vKe);
            ext.print("Te", "%5.2f", vTe);
            ext.print("Yat", "-%5.2f", vYat);
            System.out.println("------------------------ [ dt*Yat ] ------------------------");
            for (double[] v : vYat) { 
                for (int t = 0; t < v.length; t++) {
                    System.out.printf("%5.0f ", t*system.dt()*v[t]);
                } 
                System.out.println();
            }
            for(CplexPrinter p : printers){
                p.printer(ext);
            }
            System.out.println("------------------------ [ states ] ------------------------");
            for(LinearState state : states){
                double x[] = cplex.getValues(state.x, true);
                for(int i=0; i<x.length; i++){
                    System.out.printf("%5.2f ", x[i]);
                }
                System.out.println();
            }
            System.out.println("------------------------ [ controls ] ------------------------");
            for(LinearControl control : controls){
                double u[] = cplex.getValues(control.u, true);
                for(int i=0; i<u.length; i++){
                    System.out.printf("%5.2f ", u[i]);
                }
                System.out.println();
            }
        }
    }
    @Override
    public void paint(Graphics2DReal gr, double size) throws Exception {
        if(system.plot.type() == OnoPlot.ALLOC.ALL || system.plot.type() == OnoPlot.ALLOC.CRT){
            if(crt!=null){
                for(LinearParticleControl c : crt){
                    if(c!=null){
                        c.paint(gr, size, end);
                    }
                }
            }
        }
        
            
        gr.g2.setStroke(new BasicStroke(3));
        Color color = Color.BLACK;
        for(int t=0; t<states.length-1; t++){//plot lines
            if(t < end && states[t].plot!=null){
                states[t].plot.drawLine(gr, color, states[t+1].plot);
            }
        }
        for(int t=0; t<states.length; t++){
            if(t <= end && states[t].plot!=null){
                states[t].plot.fillPoint(gr, color, 0.005*size);
            }
        }
        gr.g2.setStroke(new BasicStroke(1));

        for(double delta : system.inst.chance_constraints){
            for(int t=0; t<=end; t++){
                if(states[t].plot!=null){
                    states[t].plot.drawRiskAllocation(gr, system.unc, delta, Color.BLUE);
                }
            }
        }

        for(int t=0; t<=end; t++){
            if(states[t].plot!=null){
                states[t].plot.drawRiskAllocation(gr, system.unc, system.Delta(), Color.RED);
            }
        }
        if(system.plot.type() == OnoPlot.ALLOC.ALL){
            for(int t=0; t<=end; t++){
                if(states[t].plot!=null){
                    states[t].plot.drawLabel(gr, Color.BLACK, 0.004*size);
                }
            }
        }
            
    }

    @Override
    public void save() throws Exception {
        PrintStream out = new PrintStream("./output.txt");
        out.printf("%d\n", states.length);
        for(LinearState state : states){
            double x[] = cplex.getValues(state.x);
            if(system.N()==2){
                for(int i=0; i<2; i++){
                    out.printf("%1.14f\t", x[i]);
                }
                out.printf("%1.0f\n", 0.0);
            }else{
                for(int i=0; i<2; i++){
                    out.printf("%1.14f\t", x[i]);
                }
                out.printf("%1.14f\t", x[2]);
            }
        }
        out.close();
    }

    
}
