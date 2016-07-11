/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.flaviana.model.PINF;

import ProOF.CplexExtended.CplexExtended;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;

/**
 *
 * @author Flaviana
 */
public class PINFModel {

    private CplexExtended cplex;
    //private IloIntVar Xij[][];//para o  cplex
    //public IloNumVar Xij[][];//para o RF
    //private IloNumVar Ui[];

    public IloIntVar Ym[];
    private IloNumVar KF;
    private IloNumVar Qpmt[][][];
    private IloNumVar Fpmt[][][];

    public PINFModel(CplexExtended cplex, PINFInstance inst) throws IloException {
        this.cplex = cplex;
//        if(isRelax){
//            Xij =  cplex.numVarArray(inst.N, inst.N, 0.0, 1.0, "Xij");
//        }else{
//            Xij =  cplex.boolVarArray(inst.N, inst.N, "Xij");
//        }
        //variaveis do problema
        //Xij =  cplex.boolVarArray(inst.N, inst.N, "Xij");
        //Ui = cplex.numVarArray(inst.N, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "Ui");

        Ym = cplex.boolVarArray(inst.M, "Ym");
        KF = cplex.numVar(0, Double.POSITIVE_INFINITY, "KF");
        Qpmt = cplex.numVarArray(inst.P, inst.M, inst.T, 0, Double.POSITIVE_INFINITY, "Qpmt");
        Fpmt = cplex.numVarArray(inst.P, inst.M, inst.T, 0, Double.POSITIVE_INFINITY, "Fpmt");

        /**
         * Funcao objetivo
         */
        IloNumExpr objetive = cplex.prod(inst.CF, KF);
        for (int m = 0; m < inst.M; m++) {
            //objetive += CF*KF + soma(Cm*Ym);
            objetive = cplex.SumProd(objetive, inst.Cm[m], Ym[m]);

        }
        cplex.addMinimize(objetive);

        /**
         * Restricoes
         */
        /**
         * soma Ym <= Ma
         */
        IloNumExpr expr = null;//no inicio do somatorio
        for (int m = 0; m < inst.M; m++) {
            expr = cplex.SumProd(expr, 1, Ym[m]);
        }
        cplex.addLe(expr, inst.Ma);

        /**
         * Fpmt <= Ym para todo p,m,t
         */
        for (int p = 0; p < inst.P; p++) {
            //IloNumExpr expr = null;
            for (int m = 0; m < inst.M; m++) {
                for (int t = 0; t < inst.T; t++) {
                    //expr = cplex.SumProd(expr, 1, Ym[m]);
                    cplex.addLe(Fpmt[p][m][t], Ym[m]);
                }
            }
            //cplex.addLe(expr, inst.Ym);
        }
        /**
         * Fpmt <= ACpm para todo p,m,t
         */
        for (int p = 0; p < inst.P; p++) {
            for (int m = 0; m < inst.M; m++) {
                for (int t = 0; t < inst.T; t++) {
                    cplex.addLe(Fpmt[p][m][t], inst.ACpm[p][m]);
                }
            }
            //cplex.addLe(expr, inst.Ym);
        }
        /**
         * soma Fpmt = Ym para todo m,t
         */
        for (int m = 0; m < inst.Ma; m++) {
            //IloNumExpr expr = null;
            for (int t = 0; t < inst.T; t++) {
                expr = null;
                for (int p = 0; p < inst.P; p++) {
                    //IloNumExpr expr = null;
                    expr = cplex.SumProd(expr, 1, Fpmt[p][m][t]);
                }
                cplex.addEq(expr, Ym[m]);
            }
        }
        /**
         * Qpmt=Fpma*(Rp*Wp*NSm*TGm*Nm)
         */
        for (int p = 0; p < inst.P; p++) {
            //IloNumExpr expr = null;
            for (int m = 0; m < inst.M; m++) {
                for (int t = 0; t < inst.T; t++) {
                    cplex.addEq(Qpmt[p][m][t], cplex.prod(Fpmt[p][m][t], inst.Rp[p] * inst.Wp[p] * inst.NSm[m] * inst.TGm[m] * inst.Nm[m]));
                }
            }
        }
        /**
         * soma da soma Qpmt >= soma Dis para todo p,t
         */
        for (int p = 0; p < inst.P; p++) {
            //IloNumExpr expr = null;
            for (int t = 0; t < inst.T; t++) {
                expr = null;
                for (int s = 0; s <= t; s++) {
                    for (int m = 0; m < inst.M; m++) {
                        //IloNumExpr expr = null;
                        expr = cplex.SumProd(expr, 1, Qpmt[p][m][s]);
                    }
                }
                double Dps = 0;
                for (int s = 0; s <= t; s++) {
                    Dps += inst.Dpt[p][s];

                }
                cplex.addGe(expr, Dps);

            }
        }
        /**
         * soma da soma Qpmt <= KF para todo t
         */
        for (int t = 0; t < inst.T; t++) {
            expr = null;
            for (int p = 0; p < inst.P; p++) {
                for (int m = 0; m < inst.M; m++) {
                    expr = cplex.SumProd(expr, 1, Qpmt[p][m][t]);
                }
            }
            cplex.addLe(expr, KF);
        }
    }

}
