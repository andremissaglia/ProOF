/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear;

import ProOF.CplexExtended.CplexExtended;
import ProOF.CplexExtended.iCplexExtract;
import ProOF.apl.UAV.abst.State;
import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;

/**
 *
 * @author marcio
 */
public abstract class LinearState extends State{
    public final IloNumVar x[];   //[px   py  ...| vx  vy  ...]d
    public final int t;
    
    public LinearState(CplexExtended cplex, int length, int t) throws IloException {
        this.t = t;
        x = cplex.numVarArray(length, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "X");
    }
    public abstract IloNumExpr delta() throws IloException;
    public abstract IloNumExpr risk() throws IloException;
    public abstract int FRT() throws Exception;
    
    public LinearPlotState plot;
    
    @Override
    public final void extract(iCplexExtract ext) throws IloException{
        plot= new LinearPlotState(ext.getValues(x), t);
    }

    public final double[] x() {
        return plot.copy_x();
    }
    
    
    
}
