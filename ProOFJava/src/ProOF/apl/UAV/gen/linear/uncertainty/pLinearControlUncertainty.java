/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.uncertainty;

import ProOF.apl.UAV.abst.uncertainty.pStateUncertainty;
import java.io.PrintStream;

/**
 *
 * @author marcio
 */
public abstract class pLinearControlUncertainty extends pStateUncertainty{
    public abstract double Sigma(int t, int i, int j) throws Exception;
    public abstract int N() throws Exception;
    public abstract double[] onlineFeedBack(double u[], double z[], double x[]) throws Exception;
    
    public double risk_allocation(int t, double delta) throws Exception{
        double a[] = new double[N()];
        for(int i=0; i<a.length; i++){
            a[i] = 1.0/Math.sqrt(2);
        }
        return inv_erf(1-2*delta)*Math.sqrt(2*sigma(t, a))/Math.sqrt(2);
    }
    public double risk_allocation(int t, double delta, double a[]) throws Exception{
        return inv_erf(1-2*delta)*Math.sqrt(2*sigma(t,a));
    }
    public double sigma(int t) throws Exception{
        double sigma = 0;
        for(int n=0; n<N(); n++){
            sigma += Sigma(t, n, n);
        }
        return sigma/N();
    }
    public double sigma(int t, double a[]) throws Exception{
        double temp[] = new double[N()];
        for(int n=0; n<N(); n++){
            for(int k=0; k<N(); k++){
                temp[n] += a[k] * Sigma(t, k, n);
            }
        }
        double sigma = 0;             // = a(j,t)^T * Sigma(t) * a(j,t)
        for(int k=0; k<N(); k++){
            sigma += temp[k] * a[k];
        }
        return sigma;
    }

    public void print_covariance(PrintStream out)throws Exception {
        
    }
}
