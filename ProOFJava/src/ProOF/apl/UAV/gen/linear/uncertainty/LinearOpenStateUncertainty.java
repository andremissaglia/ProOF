/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.uncertainty;

import Jama.Matrix;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;

/**
 *
 * @author marcio
 */
public class LinearOpenStateUncertainty extends pLinearStateUncertainty{
    
    
    private final LinearSystem approach;
    
    private oLinearDynamic dynamic;
    
    private Matrix Sigma[];
    private Matrix SigmaX0;
    private Matrix SigmaWt;

    private double std_position;
    private double std_velocity;
    private double inc_factor;
    
    public LinearOpenStateUncertainty(LinearSystem approach) {
        this.approach = approach;
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); //To change body of generated methods, choose Tools | Templates.
        std_position = link.Dbl("std-position", 0.05, 0.0, 1e6);
        std_velocity = link.Dbl("std-velocity", 0.00, 0.0, 1e6);
        inc_factor = link.Dbl("inc-factor", 1.0, 0.0, 1e6);
    }
    
    @Override
    public String name() {
        return "State Open Unc.";
    }
    @Override
    public int N() throws Exception {
        return approach.N();
    }
    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        dynamic = link.need(oLinearDynamic.class, dynamic);
    }
    
    private void check() throws Exception{
        //A + B*K = 0 --> B*K = -A --> K = -B^(-1)*A
        if(Sigma==null){
            //System.out.println("==============================[State]==============================");
            Matrix A = new Matrix(dynamic.A);
            
            SigmaX0 = new Matrix(dynamic.n(), dynamic.n());
            for(int i=0; i<dynamic.m(); i++){
                SigmaX0.set(i, i, Math.pow(std_position,2));
            }
            for(int i=dynamic.m(); i<dynamic.n(); i++){
                SigmaX0.set(i, i, Math.pow(std_velocity,2));
            }
            SigmaWt = new Matrix(dynamic.n(), dynamic.n());
            for(int i=0; i<dynamic.m(); i++){
                SigmaWt.set(i, i, Math.pow(std_position,2)*(Math.pow(inc_factor+1,2)-1)/approach.Waypoints());
            }//2^2-1 = 3; 3^3-1 = 8
            for(int i=dynamic.m(); i<dynamic.n(); i++){
                SigmaWt.set(i, i, Math.pow(std_velocity,2)*(Math.pow(inc_factor+1,2)-1)/approach.Waypoints());
            }
            
            Sigma = new Matrix[approach.Waypoints()+1];
            Sigma[0] = SigmaX0.copy();
//            System.out.println("-----------------[sigma("+0+")]------------------");
//            Sigma[0].print(8, 5);
            
            for(int t=0; t<approach.Waypoints(); t++){
                Sigma[t+1] = A.times(Sigma[t]).times(A.transpose()).plus(SigmaWt);
                //Sigma[t+1] = Sigma[t].plus(SigmaWt);
                
//                System.out.println("-----------------[sigma("+(t+1)+")]------------------");
//                Sigma[t+1].print(8, 5);
            }
        }
    }

    @Override
    public double Sigma(int t, int row, int col) throws Exception {
        check();
        //System.out.println("uncX = "+Sigma[t].get(row, col));
        return Sigma[t].get(row, col);
    }
    
    @Override
    public double SigmaX0(int row, int col) throws Exception{
        check();
        return SigmaX0.get(row, col);
    }
    @Override
    public double SigmaWt(int row, int col) throws Exception{
        check();
        return SigmaWt.get(row, col);
    }
}
