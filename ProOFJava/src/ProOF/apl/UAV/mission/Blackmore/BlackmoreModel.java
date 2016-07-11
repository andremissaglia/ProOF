/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.CplexExtended.CplexExtended;
import ProOF.CplexExtended.iCplexExtract;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.gen.linear.LinearModel;
import ProOF.apl.UAV.gen.linear.LinearParticleControlContinuos;
import ProOF.apl.UAV.gen.linear.LinearState;
import ProOF.com.Linker.LinkerResults;
import ilog.concert.IloException;
import ilog.concert.IloNumVar;
import java.awt.Color;
import java.io.PrintStream;

/**
 *
 * @author marcio
 */
public abstract class BlackmoreModel extends LinearModel<BlackmoreSystem>{
    
    private final boolean isO[][];
    private final boolean isN[][];
    private final IloNumVar[][][] Zjsi;
    //private final IloNumVar Ys[];
    //private final IloNumVar Yjs[][];
    public final double delta_to_cut;
    
    public BlackmoreModel(BlackmoreSystem approach, String name, CplexExtended cplex, double delta_to_cut, boolean isRelax) throws IloException, Exception {
        super(approach, name, cplex);
        this.delta_to_cut = delta_to_cut;
        isO = new boolean[approach.inst().J()][approach.Steps()+1];
        isN = new boolean[approach.inst().J()][approach.Steps()+1];
        Zjsi = new IloNumVar[approach.inst().J()][approach.Steps()+1][];
        
        //isRelax = true;
        for(int j=0; j<approach.inst().J(); j++){
            for(int s=0; s<approach.Steps()+1; s++){
                if(isRelax){
                    Zjsi[j][s] = cplex.numVarArray(approach.inst().Gj(j), 0.0, 1.0, "Z"+(j+1)+""+(s));
                }else{
                    Zjsi[j][s] = cplex.boolVarArray(approach.inst().Gj(j), "Z"+(j+1)+""+(s));
//                    if(s==0){
//                        Zjsi[j][s] = cplex.boolVarArray(approach.inst.Gj(j), "Z"+(j+1)+""+(s));
//                    }else{
//                        Zjsi[j][s] = new IloNumVar[approach.inst.Gj(j)];
//                        //Zjsi[j][s] = cplex.numVarArray(approach.inst.Gj(j), 0.0, 1.0, "Z"+(j+1)+""+(s));
//                    }
                }
            }
        }
        
         //---------------------- Mission path planning from start state to end point ---------------------------------
        for(int i=0; i<approach.inst().start_state.length; i++){
            int t = 0;
            states[t].x[i].setLB(approach.inst().start_state[i]);
            states[t].x[i].setUB(approach.inst().start_state[i]);
        }
        for(int i=0; i<approach.inst().end_point.length; i++){
            int t = approach.Waypoints();
            states[t].x[i].setLB(approach.inst().end_point[i]);
            states[t].x[i].setUB(approach.inst().end_point[i]);
        }
        
        approach.addObjective(this);
        
//        //---------------------- Proposta-5: melhoria com base na subida e decida ---------------------------------------
//        //Zj,s+1,i = (Zjsd and YjsU) or (Zjsu and YjsD) or (Zjsi and not(YjsU) and not(YjsD)) 
//        for(int j=0; j<approach.inst.J(); j++){
//            IloNumExpr sum = null;
//            for(int i=0; i<approach.inst.Gj(j); i++){
//                sum = cplex.SumProd(sum, 1, Zjsi[j][0][i]);
//            }
//            cplex.addEq(sum, 1);
//        }
//        IloNumVar YjsU[][] = cplex.boolVarArray(approach.inst.J(), approach.Steps(), "YjsT");
//        IloNumVar YjsD[][] = cplex.boolVarArray(approach.inst.J(), approach.Steps(), "YjsD");
//        for(int j=0; j<approach.inst.J(); j++){
//           for(int s=0; s<approach.Steps(); s++){
//               cplex.addLe(cplex.sum(YjsU[j][s], YjsD[j][s]), 1);
//           }
//        }
//        for(int j=0; j<approach.inst.J(); j++){
//            for(int s=0; s<approach.Steps(); s++){
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    int d = (i-1+approach.inst.Gj(j))%approach.inst.Gj(j);
//                    int u = (i+1)%approach.inst.Gj(j);
//                    IloNumExpr up   = cplex.And(Zjsi[j][s][d], YjsU[j][s]);
//                    IloNumExpr down = cplex.And(Zjsi[j][s][u], YjsD[j][s]);
//                    IloNumExpr stay = cplex.And(Zjsi[j][s][i], cplex.Not(YjsU[j][s]), cplex.Not(YjsD[j][s]));
//                    Zjsi[j][s+1][i] = cplex.Or(up, down, stay);
//                    //cplex.addEq(Zjsi[j][s+1][i], cplex.Or(up, down));
//                }
//            }
//        }
////        IloNumExpr changes = null;
////        for(int s=0; s<approach.Steps(); s++){
////            for(int j=0; j<approach.inst.J(); j++){
////                changes = cplex.SumProd(changes, 1, YjsU[j][s]);
////                changes = cplex.SumProd(changes, 1, YjsD[j][s]);
////            }
////        }
////        cplex.addLe(changes, 7);
//         //---------------------- Proposta-4: melhoria com variável Ys ---------------------------------------
//        IloNumVar Yjs[][] = cplex.numVarArray(approach.inst.J(), approach.Steps(), 0, 1, "Ys");
//        //IloNumVar Ys[] = cplex.boolVarArray(approach.Steps(), "Ys");
//        //not Yjs ==> AND_i{ Zjis <=> Zji,s+1 }
//        //    Yjs ==> OR_i { Zjis xor Zji,s+1 }
//        for(int s=0; s<approach.Steps(); s++){
//            for(int j=0; j<approach.inst.J(); j++){
//                IloNumExpr eq[] = new IloNumExpr[approach.inst.Gj(j)];
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    eq[i] = cplex.IF_Only(Zjsi[j][s][i], Zjsi[j][s+1][i]);
//                }
//                cplex.addIF_Y_Them_Zi(null, cplex.Not(Yjs[j][s]), eq);
//                
//                IloNumExpr xor[] = new IloNumExpr[approach.inst.Gj(j)];
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    xor[i] = cplex.XOr(Zjsi[j][s][i], Zjsi[j][s+1][i]);
//                }
//                IloNumExpr ORi = cplex.Or(name, xor);
//                cplex.addIF_Y_Them_Zi(null, Yjs[j][s], ORi);
//                
//            }
//        }
//        IloNumExpr changes = null;
//        for(int s=0; s<approach.Steps(); s++){
//            for(int j=0; j<approach.inst.J(); j++){
//                changes = cplex.SumProd(changes, 1, Yjs[j][s]);
//            }
//        }
//        cplex.addLe(changes, approach.inst.J()*2);
        //cplex.addMinimizeExtra(changes);
        
        
////        //---------------------- Proposta-1: melhoria com variável Ys ---------------------------------------
//        IloNumVar Ys[] = cplex.numVarArray(approach.Steps(), 0, 1, "Ys");
//        //Ys = cplex.boolVarArray(approach.Steps(), "Ys");
//        //not Ys ==> Zjis == Zji,s+1
//        for(int s=0; s<approach.Steps(); s++){
//            for(int j=0; j<approach.inst.J(); j++){
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    cplex.addGe(Ys[s], cplex.minus(Zjsi[j][s][i], Zjsi[j][s+1][i]));
//                    cplex.addGe(Ys[s], cplex.minus(Zjsi[j][s+1][i], Zjsi[j][s][i]));
//                }
//            }
//        }
//        IloNumExpr changes = null;
//        for(int s=0; s<approach.Steps(); s++){
//            changes = cplex.SumProd(changes, 1, Ys[s]);
//        }
//        cplex.addLe(changes, 8);
        
//        for(int s=0; s<approach.Steps(); s++){
//            cplex.setPriority(Ys[s], 1);
//        }
        
//        //---------------------- Proposta-2: melhoria com variável Yjs ---------------------------------------
//        Yjs = cplex.numVarArray(approach.inst.J(), approach.Steps(), 0, 1, "Ys");
//        //Yjs = cplex.boolVarArray(approach.inst.J(), approach.Steps(), "Yjs");
//        //not Yjs ==> Zjis == Zji,s+1
//        for(int s=0; s<approach.Steps(); s++){
//            for(int j=0; j<approach.inst.J(); j++){
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    cplex.addGe(Yjs[j][s], cplex.minus(Zjsi[j][s][i], Zjsi[j][s+1][i]));
//                    cplex.addGe(Yjs[j][s], cplex.minus(Zjsi[j][s+1][i], Zjsi[j][s][i]));
//                }
//            }
//        }
//        IloNumExpr changes = null;
//        for(int s=0; s<approach.Steps(); s++){
//            for(int j=0; j<approach.inst.J(); j++){
//                changes = cplex.SumProd(changes, 1, Yjs[j][s]);
//            }
//        }
//        //cplex.addLe(changes, approach.inst.J()*2);
//        cplex.addMinimizeExtra(changes);
        
        
//        //---------------------- Proposta-3: melhoria com fluxo estilo HCCQSP ---------------------------------------
//        int N = 4;
//        IloNumVar Hjni[][][] = new IloNumVar[approach.inst.J()][N][];
//        for(int j=0; j<approach.inst.J(); j++){
//            for(int n=0; n<N; n++){
//                Hjni[j][n] = cplex.boolVarArray(approach.inst.Gj(j), "Hjni");
//            }
//        }
//        IloNumVar Tjn[][] = cplex.numVarArray(approach.inst.J(), N, 0, Double.MAX_VALUE, "Tjn");
//        for(int j=0; j<approach.inst.J(); j++){
//            Tjn[j][0].setLB(0); //start in zero
//            Tjn[j][0].setUB(0); //start in zero
//            Tjn[j][N-1].setLB(approach.Steps()); //ends in dt*T
//            Tjn[j][N-1].setUB(approach.Steps()); //ends in dt*T
//        }
//        for(int j=0; j<approach.inst.J(); j++){
//            for(int n=0; n<N-1; n++){
//                cplex.addGe(Tjn[j][n+1], Tjn[j][n]);
//            }
//        }
//        
//        for(int j=0; j<approach.inst.J(); j++){
//            for(int n=0; n<N; n++){
//                IloNumExpr sum = null;
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    sum = cplex.SumProd(sum, 1, Hjni[j][n][i]);
//                }
//                cplex.addEq(sum, 1);
//            }
//        }
//        for(int j=0; j<approach.inst.J(); j++){            
//            for(int i=0; i<approach.inst.Gj(j); i++){
//                IloNumExpr sum = null;
//                for(int n=0; n<N; n++){
//                    sum = cplex.SumProd(sum, 1, Hjni[j][n][i]);
//                }
//                cplex.addLe(sum, 1);
//            }
//        }
//        
//        //IloNumVar Tjns[][][] = new IloNumVar[approach.inst.J()][N][approach.Steps()+1];
//        IloNumVar Tjns[][][] = cplex.numVarArray(approach.inst.J(), N, approach.Steps()+1, 0, 1, "Tjns");
//        for(int j=0; j<approach.inst.J(); j++){
//            for(int n=0; n<N-1; n++){
//                for(int s=0; s<approach.Steps()+1; s++){
//                    
//                    cplex.addIF_Between_Them_Y("x", Tjns[j][n][s], Tjn[j][n], s, Tjn[j][n+1]);
//                    
//                    for(int i=0; i<approach.inst.Gj(j); i++){
//                        IloNumVar y = cplex.And(Tjns[j][n][s], Hjni[j][n][i]);
//                        cplex.addGe(Zjsi[j][s][i], y);
//                        cplex.addLe(cplex.sum(Zjsi[j][s][i], y), 2);
//                    }
//                }
//            }
//        }
        
//        for(int s=0; s<approach.Steps(); s++){
//            for(int j=0; j<approach.inst.J(); j++){
//                cplex.setPriority(Yjs[j][s], s+1);
//            }
//        }
        
        
        
        //---------------------- Fortalecimentos ---------------------------------------
//        //Não pode voltar a ativar uma aresta que foi utilizada e depois desativada no passado
//        //Z(s-1,j,i) and not Z(s,j,i) --> Z(k,j,i) = 0      for all (s,j,i,k>s) 
//        for(int s=1; s<approach.Steps()+1; s++){
//            for(int k=s+1; k<approach.Steps()+1; k++){
//                for(int j=0; j<approach.inst.J(); j++){                   
//                    for(int i=0; i<approach.inst.Gj(j); i++){
//                        cplex.addLe(cplex.sum(Zjsi[j][k][i], Zjsi[j][s-1][i]), cplex.sum(Zjsi[j][s][i], 1), "Str1");
//                    }
//                }
//            }
//        }
//        //A aresta fica desativada sempre até a primeira ativação
//        //not Z(s,j,i) and Z(s+1,j,i) --> Z(k,j,i) = 0      for all (s,j,i,k<s) 
//        for(int s=1; s<approach.Steps(); s++){
//            for(int k=0; k<s; k++){
//                for(int j=0; j<approach.inst.J(); j++){                   
//                    for(int i=0; i<approach.inst.Gj(j); i++){
//                        cplex.addLe(cplex.sum(Zjsi[j][k][i], Zjsi[j][s+1][i]), cplex.sum(Zjsi[j][s][i], 1), "Str2");
//                    }
//                }
//            }
//        }
//        
//        //No primeiro instante (s=0) uma e apenas uma aresta deve estar ativa
//        for(int j=0; j<approach.inst.J(); j++){                   
//            IloNumExpr expr = null;
//            for(int i=0; i<approach.inst.Gj(j); i++){
//                expr = cplex.SumProd(expr, 1.0, Zjsi[j][0][i]);
//            }
//            cplex.addEq(expr, 1, "STR3");
//        }
//        
//        //Deve haver no máximo 'G(j)-1' trocas de aresta para cada obstáculo ao longo o horizonte de planejamento
//        for(int j=0; j<approach.inst.J(); j++){                   
//            IloNumExpr expr = null;
//            for(int s=0; s<approach.Steps()+1; s++){
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    expr = cplex.SumProd(expr, 1.0, Zjsi[j][s][i]);
//                }
//            }
//            cplex.addLe(expr, approach.Steps()+approach.inst.Gj(j), "STR4");
//        }
//        
//        //Proibido ativar a aresta da troca e não faze-la
//        for(int j=0; j<approach.inst.J(); j++){
//            for(int s=1; s<approach.Steps(); s++){
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    IloNumExpr expr = null;
//                    expr = cplex.SumProd(expr, +2.0, Zjsi[j][s][i]);
//                    expr = cplex.SumProd(expr, -1.0, Zjsi[j][s-1][i]);
//                    expr = cplex.SumProd(expr, -1.0, Zjsi[j][s+1][i]);
//                    cplex.addLe(expr, 1, "STR5");
//                }
//            }
//        }
        
//        //É nescessário  haver no máximo 2 arestas ativas a cada instante para cada obstáculo
//        for(int j=0; j<approach.inst.J(); j++){
//            for(int s=0; s<approach.Steps()+1; s++){
//                IloNumExpr expr = null;
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    expr = cplex.SumProd(expr, 1.0, Zjsi[j][s][i]);
//                }
//                cplex.addLe(expr, 2, "STR6");
//            }
//        }
        
       
        
        
        //---------------------- Fortalecimentos no objetivo---------------------------------------
//        IloNumExpr extra_obj = null;
//        for(int j=0; j<approach.inst.J(); j++){  
//            for(int s=0; s<approach.Steps()+1; s++){
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    extra_obj = cplex.SumProd(extra_obj, 0.01, Zjsi[j][s][i]);
//                }
//            }
//        }
//        cplex.addMinimizeExtra(extra_obj);
    }
    public abstract void addRisckAllocationConstraint(int j, int t) throws Exception;

    @Override
    public boolean addChanges() throws Exception {
        return false;
    }
    
    public final IloNumVar Zs(int j, int step, int i){
        return Zjsi[j][step][i];
    }
    public final IloNumVar Zt(int j, int t, int i){
        int s = system.Step(t);
        return Zjsi[j][s][i];
    }

    public final int[][][] getZjti() throws IloException{
        int vZjti[][][] = new int[system.inst().J()][system.Steps()+1][];
        for(int j=0; j<system.inst().J(); j++){
            for(int t=0; t<system.Steps()+1; t++){
                vZjti[j][t] = new int[system.inst().Gj(j)];
                if(isO[j][t]){
                    double vals[] = cplex.getValues(Zjsi[j][t]);
                    for(int i=0; i<system.inst().Gj(j); i++){
                        vZjti[j][t][i] = vals[i] > 0.5 ? 1 : 0; 
                    }
                }
            }
        }
        return vZjti;
    }
    
    public final double[][] getXt() throws IloException {
        double [][] vXt = new double[system.Waypoints()+1][];
        for(int t=0; t<system.Waypoints()+1; t++){
            vXt[t] = cplex.getValues(states[t].x);
        }
        return vXt;
    }
    public void fixWith(BlackmoreModel toFix) throws IloException {
        int count_fix = 0;
        int count_tot = 0;
        int vZjti[][][] = toFix.getZjti();
        for(int j=0; j<system.inst().J(); j++){
            for(int s=0; s<system.Steps()+1; s++){
                if(toFix.isN[j][s] && toFix.isO[j][s]){
                    for(int i=0; i<system.inst().Gj(j); i++){
                        Zs(j, s, i).setLB(vZjti[j][s][i]);
                        Zs(j, s, i).setUB(vZjti[j][s][i]);
                    }
                    count_fix++;
                }
                count_tot++;
            }
        }
        System.out.printf("fix about %1.1f%% variables\n", count_fix*100.0/count_tot);
    }
    private void fixZjti(BlackmoreModel toFix) throws IloException{
        int vZjti[][][] = toFix.getZjti();
        for(int j=0; j<system.inst().J(); j++){
            for(int s=0; s<system.Steps()+1; s++){
                for(int i=0; i<system.inst().Gj(j); i++){
                    Zs(j, s, i).setLB(vZjti[j][s][i]);
                    Zs(j, s, i).setUB(vZjti[j][s][i]);
                }
            }
        }
    }
    private void fixControls(BlackmoreModel toFix) throws IloException{
        for(int t=0; t<controls.length; t++){
            for(int n=0; n<controls[t].u.length; n++){
                controls[t].u[n].setLB(toFix.cplex.getValue(toFix.controls[t].u[n]));
                controls[t].u[n].setUB(toFix.cplex.getValue(toFix.controls[t].u[n]));
            }
        }
    }
    public final int size(){
        int count = 0;
        for(int j=0; j<system.inst().J(); j++){
            count += system.inst().Gj(j);
        }
        return count*(system.Steps()+1);
    }
    public final boolean addOs(int j, int s) throws Exception{
        if(!isO[j][s]){
            for(int t=system.Waypoint(s); t<system.Waypoint(s+1); t++){
                addRisckAllocationConstraint(j, t);
            }
            if(s+1==system.Steps()){
                int t = system.Waypoints();
                addRisckAllocationConstraint(j, t);
            }
            isO[j][s] = true;
            return true;
        }
        return false;
    }
    public final boolean addNs(int j, int s){
        if(!isN[j][s]){
            isN[j][s] = true;
            return true;
        }
        return false;
    }

    private LinearParticleControlContinuos crt[];
    protected double true_alloaction;
    @Override
    public void extract(iCplexExtract ext, Callback type) throws IloException, Exception {
        super.extract(ext, type); //To change body of generated methods, choose Tools | Templates.
        //System.out.println("type = "+type);
        if(type == Callback.FINAL_EXTRACT){
            final int length = Math.min(10000, Math.max(100,(int)(1.0/system.Delta())));
            crt = new LinearParticleControlContinuos[length];
            true_alloaction = 0;
            for(int n=0; n<system.ParticleControl(); n++){
                LinearParticleControlContinuos temp = new LinearParticleControlContinuos(system.unc(), system.dynamic(), states, controls, system.inst().obstacles);
                if(temp.hasColision()){
                    true_alloaction+=1.0/system.ParticleControl();
                }
                if(n<crt.length){
                    crt[n] = temp;
                }
            }
            system.plot().repaint();
            //System.out.println("CRT");
        }
    }
    @Override
    public void paint(Graphics2DReal gr, double size) throws Exception {
        //System.out.println("All_paint");
        if(system.plot().type() == BlackmorePlot.ALLOC.ALL || system.plot().type() == BlackmorePlot.ALLOC.CRT){
            //System.out.println("CRT_paint-1");
            if(crt!=null){
                //System.out.println("CRT_paint-2");
                //int length = Math.min(crt.length, Math.max(1000,(int)(1.0/approach.Delta())));
                for(int n=0; n<crt.length; n++){
                    if(crt[n]!=null && !crt[n].hasColision()){
                        crt[n].paint(gr, size);
                    }
                }
                for(int n=0; n<crt.length; n++){
                    if(crt[n]!=null && crt[n].hasColision()){
                        crt[n].paint(gr, size);
                    }
                }
            }
        }
        
        super.paint(gr, size);
        
        if(isExtract()){
            //gr.g2.setStroke(new BasicStroke(3));
            Color color = Color.BLACK;
            for(int s=0; s<system.Steps()+1; s++){
                int t = system.Waypoint(s);
                states[t].plot.fillPoint(gr, color, 0.005*size);
            }
        }
        
        
        if(isExtract()){
            for (LinearState state : states) {
    //            state.plot.drawRiskAllocation(gr, approach.unc, approach.Delta(), Color.RED, approach.inst.obstacles);
    //            state.plot.drawRiskAllocation(gr, approach.unc, approach.Delta(), Color.MAGENTA);
                state.plot.drawRiskAllocation(gr, system.unc(), system.Delta(), Color.RED);
            }
        }
        
//        for(int j : obst){
//            approach.inst.obstacles[j].paint(gr, Color.ORANGE, size);
//        }
    }

    @Override
    public void results(LinkerResults link) throws Exception {
        super.results(link); //To change body of generated methods, choose Tools | Templates.
//        if(isFeasible()){
//            double vZjsi[][][] = cplex.getValues(Zjsi);
//            //double vYs[] = cplex.getValues(Ys);
//            System.out.printf("step : ");
//            for(int j=0; j<system.inst().J(); j++){
//                System.out.printf("[%"+(system.inst().Gj(j)*5)+"s ] ", "j = "+j);
//            }
//            System.out.println();
//            for(int s=0; s<system.Steps(); s++){
//                //System.out.printf("[%2d] %3.1f: ", s, vYs[s]);
//                System.out.printf("[%2d] : ", s);
//                for(int j=0; j<system.inst().J(); j++){
//                    System.out.print("[ ");
//                    for(int i=0; i<system.inst().Gj(j); i++){
//                        System.out.printf("%4.2f ", Math.max(0,vZjsi[j][s][i]));
//                    }
//                    System.out.print("] ");
//                }
//                System.out.println();
//            }
//        }
            
    }

    
    
    @Override
    public void save() throws Exception {
        if(isFeasible()){
            
            
            
    //        out.printf("[");
    //        int t=0; 
    //        //final String format = "%1.14f";
    //        final String format = "%g";
    //        for(LinearState state : states){
    //            double x[] = cplex.getValues(state.x);
    //            if(approach.N()==2){
    //                out.printf("(");
    //                for(int i=0; i<2; i++){
    //                    out.printf(format+", ", x[i]);
    //                }
    //                out.printf("%1.0f)", 0.0);
    //            }else{
    //                out.printf("(");
    //                for(int i=0; i<2; i++){
    //                    out.printf(format+", ", x[i]);
    //                }
    //                out.printf(format+")", x[2]);
    //            }
    //            if(t<approach.Waypoints()){
    //                out.printf(", ", 0.0);
    //            }
    //            t++;
    //        }
    //        out.printf("]\n");
            
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
            
            route.close();
            
            
//            out.printf("%d\n", states.length);
//            
//            PointGeo p = approach.inst.Si.parseTo(approach.inst.base);
//            out.printf("%1.14f\t", p.longitude);
//            out.printf("%1.14f\t", p.latitude);
//            out.printf("%1.14f\n", p.altitude);
//            
//            p = approach.inst.Sf.parseTo(approach.inst.base);
//            out.printf("%1.14f\t", p.longitude);
//            out.printf("%1.14f\t", p.latitude);
//            out.printf("%1.14f\n", p.altitude);
//            
//            for(LinearState state : states){
//                double x[] = cplex.getValues(state.x);
//                p = new Point3D(x[0], x[1], x[2]).parseTo(approach.inst.base);
//                out.printf("%1.14f\t", p.longitude);
//                out.printf("%1.14f\t", p.latitude);
//                out.printf("%1.14f\n", p.altitude);
//            }
//            
//            p = approach.inst.Ei.parseTo(approach.inst.base);
//            out.printf("%1.14f\t", p.longitude);
//            out.printf("%1.14f\t", p.latitude);
//            out.printf("%1.14f\n", p.altitude);
//            
//            p = approach.inst.Ef.parseTo(approach.inst.base);
//            out.printf("%1.14f\t", p.longitude);
//            out.printf("%1.14f\t", p.latitude);
//            out.printf("%1.14f\n", p.altitude);
//            
//            
//            out.close();
        
        }
    }

//    public LinkedList<Integer> obst = new LinkedList<Integer>();

}
