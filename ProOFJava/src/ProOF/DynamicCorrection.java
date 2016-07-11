/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF;

import ProOF.CplexExtended.CplexExtended;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import java.util.Locale;

/**
 *
 * @author marcio
 */
public class DynamicCorrection {
    
    private static final int STEPS = 10;
    
    private static final double Vmax = 3.0;
    private static final double Umax = 1.0;
    private static final double dt = 1.0/STEPS;
    
    
    //For the dynamics A and B bellow the valuer of correction is K = N*Umax*dt*dt/8 = 0.250*Umax*dt*dt
    private static final int N = 2; //for 2D
    private static final double A[][] = {
        {1.0,   0.0,    dt,     0},
        {0.0,   1.0,   0.0,    dt},
        {0.0,   0.0,   1.0,   0.0},
        {0.0,   0.0,   0.0,   1.0},
    };
    private static final double B[][] = {
        {dt*dt/2,   0.0},
        {0.0,       dt*dt/2},
        {dt,        0.0},
        {0.0,       dt},
    };
    
    //For the dynamics A and B bellow the valuer of correction is K = N*Umax/8  = 0.375*Umax
//    private static final int N = 3; //for 3D
//    private static final double A[][] = {   
//        {1.0,   0.0,    0.0,    dt,     0.0,    0.0},
//        {0.0,   1.0,    0.0,    0.0,    dt,     0.0},
//        {0.0,   0.0,    1.0,    0.0,    0.0,    dt},
//        {0.0,   0.0,    0.0,    1.0,    0.0,    0.0},
//        {0.0,   0.0,    0.0,    0.0,    1.0,    0.0},
//        {0.0,   0.0,    0.0,    0.0,    0.0,    1.0},
//    };
//    private static final double B[][] = {
//        {dt*dt/2,   0.0,        0.0},
//        {0.0,       dt*dt/2,    0.0},
//        {0.0,       0.0,        dt*dt/2},
//        {dt,        0.0,        0.0},
//        {0.0,       dt,         0.0},
//        {0.0,       0,         dt},
//    };
    
    public static void main(String[] args) throws IloException, Exception {
        Locale.setDefault(Locale.ENGLISH);
        
        CplexExtended cplex = new CplexExtended();
        
        IloNumVar X0[] = cplex.numVarArray(2*N, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "X0");
        IloNumVar X1[] = cplex.numVarArray(2*N, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "X1");
        IloNumVar U[] = cplex.numVarArray(2*N, -Umax, +Umax, "U");
        IloNumVar Yn[][] = cplex.numVarArray(STEPS+1, 2*N, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "Yn");
        IloNumVar Zn[][] = cplex.numVarArray(STEPS+1, N, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "Zn");
        IloIntVar Qn[] = cplex.boolVarArray(STEPS+1, "Qn");
        
        //IloNumVar W = cplex.numVar(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "W");
        //cplex.addMaximize(W);
         
        IloNumVar W[] = cplex.numVarArray(N, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "W");
        IloNumExpr objective = null;
        for(int k=0; k<N; k++){
            objective = cplex.SumProd(objective, 1.0, W[k]);
        }
        cplex.addMaximize(objective);
         
        //cplex.addMinimize(W);
          
//        X0[0].setLB(0);
//        X0[0].setUB(0);
//        X0[1].setLB(0);
//        X0[1].setUB(0);
//        X1[0].setLB(1);
//        X1[0].setUB(1);
//        X1[1].setLB(0);
//        X1[1].setUB(0);
        
        //cplex.addEq(X0[1], X1[1]);
        
        
        for(int k=N; k<2*N; k++){
            X0[k].setLB(-Vmax);
            X0[k].setUB(+Vmax);
            X1[k].setLB(-Vmax);
            X1[k].setUB(+Vmax);
        }
        for(int n=0; n<STEPS+1; n++){
            for(int k=N; k<2*N; k++){
                Yn[n][k].setLB(-Vmax);
                Yn[n][k].setUB(+Vmax);
            }
        }
        
        //Y0 = X0
        for(int k=0; k<2*N; k++){
            cplex.addEq(Yn[0][k], X0[k]);
        }
        //YN = X1
        for(int k=0; k<2*N; k++){
            cplex.addEq(Yn[STEPS][k], X1[k]);
        }
        //Yn+1 = bar(A)*Yn + bar(B)*U
        for(int n=0; n<STEPS; n++){
            for(int i=0; i<2*N; i++){
                IloNumExpr exp = null;
                for(int j=0; j<2*N; j++){
                    exp = cplex.SumProd(exp, A[i][j], Yn[n][j]);
                }
                for(int j=0; j<N; j++){
                    exp = cplex.SumProd(exp, B[i][j], U[j]);
                }
                cplex.addEq(Yn[n+1][i], exp);
            }
        }
        //Zn = X0 + (X1-X0)*n/N   = X0*(1-n/N) + X1*n/N
        for(int n=0; n<STEPS+1; n++){
            for(int k=0; k<N; k++){
                cplex.addEq(Zn[n][k], cplex.sum(cplex.prod(X0[k], 1.0-n*1.0/STEPS), cplex.prod(X1[k], n*1.0/STEPS)));
            }
        }
        
        for(int n=0; n<STEPS+1; n++){
//            if(N!=2) throw new Exception("Only valid for N = 2 (2D)");
//            IloNumExpr exp[] = new IloNumExpr[32];
//            for(int p=0; p<32; p++){
//                exp[p] = W;
//                exp[p] = cplex.SumProd(exp[p], -Math.cos(p*2*Math.PI/32.0), Yn[n][0]);
//                exp[p] = cplex.SumProd(exp[p], -Math.sin(p*2*Math.PI/32.0), Yn[n][1]);
//                exp[p] = cplex.SumProd(exp[p], +Math.cos(p*2*Math.PI/32.0), Zn[n][0]);
//                exp[p] = cplex.SumProd(exp[p], +Math.sin(p*2*Math.PI/32.0), Zn[n][1]);
//            }
//            cplex.addIF_Y_Them_Le("Qn=1", Qn[n], exp);
            IloNumExpr exp[] = new IloNumExpr[N];
            for(int k=0; k<N; k++){
                exp[k] = W[k];
                exp[k] = cplex.SumProd(exp[k], -1.0, Yn[n][k]);
                exp[k] = cplex.SumProd(exp[k], +1.0, Zn[n][k]);
            }
            cplex.addIF_Y_Them_Eq("Qn=1", Qn[n], exp);
        }
        
        IloNumExpr sum = null;
        for(int n=0; n<STEPS+1; n++){
            sum = cplex.SumProd(sum, 1.0, Qn[n]);
        }
        cplex.addEq(sum, 1);
        
        if(cplex.solve()){
            System.out.printf("Status       = %s\n", cplex.getStatus());
            System.out.printf("Objective    = %s\n", cplex.getObjValue());
            System.out.printf("Lower        = %s\n", cplex.getBestObjValue());
            
            double vZn[][] = cplex.getValues(Zn);
            double vYn[][] = cplex.getValues(Yn);
            double vQn[] = cplex.getValues(Qn);
            System.out.printf("%15s %15s %15s %15s %15s %15s\n", "Zn(x)", "Zn(y)", "Yn(px)", "Yn(py)", "Yn(vx)", "Yn(vy)");
            for(int n=0; n<STEPS+1; n++){
                System.out.printf("%15g %15g %15g %15g %15g %15g %s\n", vZn[n][0], vZn[n][1], vYn[n][0], vYn[n][1], vYn[n][2], vYn[n][3], vQn[n]>0.9?"*":"");
            }
        }else{
            System.out.printf("Status       = %s\n", cplex.getStatus());
        }
    }
}
