/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import Jama.Matrix;
import ProOF.apl.UAV.gen.linear.LinearApproach;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearStateUncertainty;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;

/**
 *
 * @author marcio
 */
public class RoverStateUncertainty extends pLinearStateUncertainty{
    private final LinearSystem approach;
    private oLinearDynamic dynamic;
    
    
    private double std_position;
    private double sigma_x;
    private double sigma_y;
    private double angle;
    
    public RoverStateUncertainty(LinearSystem approach) {
        this.approach = approach;
    }
    
    @Override
    public String name() {
        return "Rover State Unc.";
    }

    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        dynamic = link.need(oLinearDynamic.class, dynamic);
    }
    
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); //To change body of generated methods, choose Tools | Templates.
        std_position = link.Dbl("sigma-0", 0.1, 0.0, 1e6);
        sigma_x = link.Dbl("sigma-x", 0.5, 0.0, 1e6);
        sigma_y = link.Dbl("sigma-y", 0.3, 0.0, 1e6);
        angle = link.Dbl("angle", 45.0, 0.0, 360.0);
    }
    
    private Matrix sigma[];
    private void check() throws Exception {
         if(sigma==null){
             sigma = new Matrix[approach.Waypoints()+1];

             Matrix A = new Matrix(dynamic.A);

             Matrix R = new Matrix(approach.N()*2, approach.N()*2);
             R.set(0, 0, Math.cos(angle*Math.PI/180));
             R.set(0, 1, -Math.sin(angle*Math.PI/180));
             R.set(1, 0, Math.sin(angle*Math.PI/180));
             R.set(1, 1, Math.cos(angle*Math.PI/180));

             Matrix S = new Matrix(approach.N()*2, approach.N()*2);
             S.set(0, 0, sigma_x*sigma_x);
             S.set(1, 1, sigma_y*sigma_y);

             for(int t=0; t<approach.Waypoints()+1; t++){
                 if(t==0){
                     sigma[t] = new Matrix(approach.N()*2, approach.N()*2);
                     for(int n=0; n<approach.N(); n++){
                         sigma[t].set(n, n, std_position*std_position);
                     }
                 }else{
                     sigma[t] = A.times(sigma[t-1]).times(A.transpose()).plus(R.times(S).times(R.transpose()));
                 }
             }
             for(int t=0; t<approach.Waypoints()+1; t++){
                 System.out.printf("-----------------------------[t=%d]---------------------------\n", t);
                 for(int i=0; i<approach.N()*2; i++){
                     for(int j=0; j<approach.N()*2; j++){
                         System.out.printf("%8.5f ", sigma[t].get(i, j));
                     }
                     System.out.println();
                 }
             }
         }
     }
    
    
    @Override
    public double Sigma(int t, int row, int col) throws Exception {
        check();
        return sigma[t].get(row, col);
    }

    @Override
    public int N() throws Exception {
        return approach.N();
    }

    @Override
    public double SigmaX0(int row, int col) throws Exception {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public double SigmaWt(int row, int col) throws Exception {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    
}
