/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.karla.model.RealeImaginario;

import ProOF.CplexExtended.CplexExtended;
import ProOF.utilities.uUtil;
import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;

/**
 *
 * @author Administrador
 */
public class RIModel {
    private final int BigM = 10000;
    private RIInstance inst;
    private CplexExtended cplex; //sempre tem que ter
    public IloNumVar x[][];
    public IloNumVar a[][];
    public IloNumVar b[][];
    public IloNumVar vr[];
    public IloNumVar vi[];
    public IloNumVar L[];
    public IloNumVar Ir[][];
    public IloNumVar Ii[][];

    public RIModel(CplexExtended cplex, RIInstance inst,  boolean isRelax) throws IloException {
        this.cplex = cplex;
        this.inst = inst;
//        x = cplex.boolVarArray(inst.Nnos, inst.Nnos, "xij");
//        a = cplex.boolVarArray(inst.Nnos, inst.Nnos, "aij");
//        b = cplex.boolVarArray(inst.Nnos, inst.Nnos, "bij");
        vr = cplex.numVarArray(inst.Nnos, 0, Double.POSITIVE_INFINITY, "vr");
        vi = cplex.numVarArray(inst.Nnos, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "vi");
        L = cplex.numVarArray(inst.Nnos, 0, Double.POSITIVE_INFINITY, "L");
        Ir = cplex.numVarArray(inst.Nnos, inst.Nnos, 0, Double.POSITIVE_INFINITY, "Ir");
        Ii = cplex.numVarArray(inst.Nnos, inst.Nnos, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "Ii");
        x = new IloNumVar[inst.Nnos][inst.Nnos];
        a = new IloNumVar[inst.Nnos][inst.Nnos];
        b = new IloNumVar[inst.Nnos][inst.Nnos];

        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    if(isRelax){
                        x[i][j] = cplex.numVar(0,1,"x({"+(i+1)+","+(j+1)+"})");
                    }
                    else{
                       x[i][j] = cplex.boolVar("x({"+(i+1)+","+(j+1)+"})"); 
                    }
                  
                    a[i][j] = cplex.numVar(0,Double.POSITIVE_INFINITY, "a({"+(i+1)+","+(j+1)+"})");
                    b[i][j] = cplex.numVar(0,Double.POSITIVE_INFINITY, "b({"+(i+1)+","+(j+1)+"})");
                   // x[i][j].setLB(inst.x0[i][j]);
                  //  x[i][j].setUB(inst.x0[i][j]);

                }
                //System.out.printf("%f ", inst.Rr[i][j]);
            }
           // System.out.println();
        }

        //função objetivo
        IloNumExpr objective = null;
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                 if (inst.Rr[i][j] > 0.0000001) {
//                    objective = cplex.SumProd(objective, inst.w[i][j] * inst.Rr[i][j], cplex.prod(Ir[i][j], Ir[i][j]));
//                    objective = cplex.SumProd(objective, inst.w[i][j] * inst.Rr[i][j], cplex.prod(Ii[i][j], Ii[i][j]));

//                     IloNumVar pos = cplex.numVar(0,Double.POSITIVE_INFINITY);
//                     IloNumVar neg = cplex.numVar(0,Double.POSITIVE_INFINITY);
//                     cplex.addGe(pos, Ii[i][j]);
//                     cplex.addGe(neg, cplex.prod(-1, Ii[i][j]));
                     
                     objective = cplex.SumNumScalProd(objective, "Obj", 64, inst.M[i][j],   inst.w[i][j] * inst.Rr[i][j], Ir[i][j]);
                     objective = cplex.SumNumScalProd(objective, "Obj", 64, inst.M[i][j],   inst.w[i][j] * inst.Rr[i][j], Ii[i][j]);
                     
                     //objective = cplex.SumNumScalProd(objective, "Obj", 32, 500,    inst.w[i][j] * inst.Rr[i][j], neg);
                 }
            }
        }
        cplex.addMinimize(objective);

        
        
        
         //restrições
        
        
        //retrição 1 = garante rede conexa
        IloNumExpr expr = null;
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    expr = cplex.SumProd(expr, 1, x[i][j]);
                }
            }
        }
        cplex.addEq(expr, inst.Nnos - inst.F.size());

        // restrição 2 = demanda é atendida
        for (int i = 0; i < inst.Nnos; i++) {
            if (!inst.F.contains(i+1)) {
                expr = null;
                for (int j = 0; j < inst.Nnos; j++) {
                    if (inst.Rr[i][j] > 0.0000001) {
                        expr = cplex.SumProd(expr, 1, Ir[j][i]);
                        expr = cplex.SumProd(expr, -1, Ir[i][j]);
                    }

                }
                cplex.addEq(expr, inst.Dr[i]);
            }
        }

        //restrição 3 = demanda é atendida 
        for (int i = 0; i < inst.Nnos; i++) {
            if (!inst.F.contains(i+1)) {
                expr = null;
                for (int j = 0; j < inst.Nnos; j++) {
                    if (inst.Rr[i][j] > 0.0000001) {
                        expr = cplex.SumProd(expr, 1, Ii[j][i]);
                        expr = cplex.SumProd(expr, -1, Ii[i][j]);
                    }

                }
                cplex.addEq(expr, inst.Di[i]);
            }
        }

        //restrição 4 = garante queda de tensão
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    if (!inst.F.contains(i+1) && !inst.F.contains(j+1)) {
                        expr = vr[j];
                        expr = cplex.SumProd(expr, -1, vr[i]);
                        expr = cplex.SumProd(expr, BigM, x[i][j]);
                        cplex.addLe(expr, BigM);
                    }

                }
            }
        }

        // restrição 5 = garante queda de tensao
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    if (inst.F.contains(i+1) && !inst.F.contains(j+1)) {
                        expr = vr[j];
                        expr = cplex.SumProd(expr, BigM, x[i][j]);
                        cplex.addLe(expr, BigM + inst.v0fr[i]);
                    }

                }
            }
        }

        // restrição 6 = garante queda de tensao
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    if (!inst.F.contains(i+1) && inst.F.contains(j+1)) {
                        expr = cplex.prod(-1, vr[i]);
                        expr = cplex.SumProd(expr, BigM, x[i][j]);
                        cplex.addLe(expr, BigM - inst.v0fr[j]);
                    }

                }
            }
        }

        // restrição 7 = garante que não esta aberta na ida e na volta
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    expr = x[i][j];
                    expr = cplex.SumProd(expr, 1, x[j][i]);
                    cplex.addLe(expr, 1);
                }
            }
        }

        //restrição 8= numeração dos nós para não haver ciclo (restrição do caxeiro viajante)
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    if (inst.F.contains(i+1)) {
                        expr = L[j];
                        expr = cplex.SumProd(expr, -1, L[i]);
                        expr = cplex.SumProd(expr, -inst.Nnos, x[i][j]);
                        cplex.addGe(expr, -2 * inst.Nnos + 1);
                    }
                }
            }
        }

        //restrição 9= numeração dos nós para não haver ciclo (restrição do caxeiro viajante)
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    if (!inst.F.contains(i+1)) {
                        expr = L[j];
                        expr = cplex.SumProd(expr, -1, L[i]);
                        expr = cplex.SumProd(expr, -inst.Nnos, x[i][j]);
                        cplex.addGe(expr, - inst.Nnos + 1);
                    }
                }
            }
        }

        
        
        //restrição 10 = permite que chegue apenas um em cada nó
        for (int i = 0; i < inst.Nnos; i++) {
            if (!inst.F.contains(i+1)) {
                expr = null;
                    for (int j = 0; j < inst.Nnos; j++) {
                        if (inst.Rr[j][i] > 0.0000001) {
                            expr = cplex.SumProd(expr, 1, x[j][i]);
                        }

                    }
                    cplex.addLe(expr, 1);
                }
            }
        

   

        //restrição 11= garante 
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    cplex.addLe(Ir[i][j], cplex.prod(BigM, x[i][j]));
                }
            }
        }

                
                
                
        // restrição 12= garante de v seja pelo menos min
        for (int i = 0; i < inst.Nnos; i++) {
            if (!inst.F.contains(i+1)) {
                cplex.addGe(vr[i], inst.vmin);
            }
        }

        
        
        
        //restrição 13= garante que o fluxo só ocorre em ligações ativas e tem que satisfazer um limite superior
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    if (!inst.F.contains(i+1) && !inst.F.contains(j+1)) {
                        expr = vr[i];
                        expr = cplex.SumProd(expr, -1, vr[j]);
                        expr = cplex.SumProd(expr, -inst.Rr[i][j] * inst.M[i][j], x[i][j]);
                        expr = cplex.SumProd(expr, BigM, x[i][j]);
                        cplex.addLe(expr, BigM);
                    }

                }

            }

        }
        
        
        
        
        //restrição 14= garante que o fluxo só ocorre em ligações ativas e tem que satisfazer um limite superior
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    if (!inst.F.contains(i+1) && inst.F.contains(j+1)) {
                        expr = vr[i];
                        expr = cplex.SumProd(expr, -inst.Rr[i][j] * inst.M[i][j], x[i][j]);
                        expr = cplex.SumProd(expr, BigM, x[i][j]);
                        cplex.addLe(expr, BigM+inst.v0fr[j]);
                    }

                }

            }

        }
        
        
        
        
        
        //restrição 15= garante que o fluxo só ocorre em ligações ativas e tem que satisfazer um limite superior
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {
                    if (inst.F.contains(i+1) && !inst.F.contains(j+1)) {
                        expr = cplex.prod(-1, vr[j]);
                        expr = cplex.SumProd(expr, -inst.Rr[i][j] * inst.M[i][j], x[i][j]);
                        expr = cplex.SumProd(expr, BigM, x[i][j]);
                        cplex.addLe(expr, BigM-inst.v0fr[i]);
                    }

                }

            }
        }
          
        
            
            
       //restrição 16= define o valor aij=1 quando ocorre uma manobra em que a ligação na rede original xij0=1 é desativada na configuração atual xij=0
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {     
                    expr = a[i][j];
                    expr = cplex.SumProd(expr, 1, x[i][j]);
                    cplex.addGe(expr, inst.x0[i][j]);
                }
            }
        }
        
        
        
        
        
        //restrição 17= define o valor bij=1 quando ocorre uma manobra em que a ligação na rede original xij0=0 é ativada na configuração atual xij=1
        for (int i = 0; i < inst.Nnos; i++) {
            for (int j = 0; j < inst.Nnos; j++) {
                if (inst.Rr[i][j] > 0.0000001) {     
                    expr = b[i][j];
                    expr = cplex.SumProd(expr, -1, x[i][j]);
                    cplex.addGe(expr, -inst.x0[i][j]);
                }
            }
        }
        
        
       // x[1][0].setLB(1);
        
        
        
//        for(int i=0; i<inst.Nnos; i++){
//            if (!inst.F.contains(i+1)) {
//                cplex.addLe(cplex.sum(cplex.prod(vr[i], vr[i]), cplex.prod(vi[i], vi[i])), 1);
//            }

            
//        }
        
 cplex.exportModel("./RI.lp");

    }
    
    
    
    public void print () throws IloException{
       // double vx[][] = cplex.getValues(x);
        for(int i=0; i<inst.Nnos; i++){
            for(int j=0; j<inst.Nnos; j++){
                if(inst.Rr[i][j]>0.000001 && cplex.getValue(x[i][j])> 0.5){
                    System.out.printf("%3d -> %3d   |   Ir = %8g   | Ii = %10g\n", i+1,j+1, cplex.getValue(Ir[i][j]), cplex.getValue(Ii[i][j]));
                }
            }
        }
    }

}


