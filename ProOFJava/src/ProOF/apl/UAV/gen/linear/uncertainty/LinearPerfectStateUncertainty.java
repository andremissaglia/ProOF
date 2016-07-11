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
public class LinearPerfectStateUncertainty extends pLinearStateUncertainty{
    
    
    private final LinearSystem approach;
    
    private oLinearDynamic dynamic;
    
    private Matrix Sigma[];
    private Matrix SigmaX0;
    private Matrix SigmaWt;

    private double std_position;
    private double std_velocity;
    
    public LinearPerfectStateUncertainty(LinearSystem approach) {
        this.approach = approach;
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); //To change body of generated methods, choose Tools | Templates.
        std_position = link.Dbl("std-position", 0.1, 0.0, 1e6);
        std_velocity = link.Dbl("std-velocity", 0.00, 0.0, 1e6);
    }
    
    @Override
    public String name() {
        return "State Perfect Unc.";
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
            System.out.println("==============================[State Perfect Uncertainty]==============================");
            Matrix A = new Matrix(dynamic.A);
            Matrix B = new Matrix(dynamic.B);
            
            //Matrix Q = new Matrix(dynamic.n(), dynamic.n());
            Matrix Q = Matrix.identity(dynamic.n(), dynamic.n());
            
            Matrix R = Matrix.identity(dynamic.m(), dynamic.m());
            //Matrix R = new Matrix(dynamic.m(), dynamic.m());
            Matrix N = new Matrix(dynamic.n(), dynamic.m());
            
            
            Matrix P = Matrix.identity(dynamic.n(), dynamic.n());
            
            for(int i=0; i<100; i++){
//                System.out.println("-----------------[P]------------------");
//                P.print(8, 5);
                
                Matrix APA = A.transpose().times(P).times(A);
                
                Matrix BPB = B.transpose().times(P).times(B);
                Matrix inv = R.plus(BPB).inverse();
                Matrix APB = A.transpose().times(P).times(B);
                Matrix BPA = B.transpose().times(P).times(A);
                
                P = APA.minus(APB.times(inv).times(BPA)).plus(Q);
            }
            System.out.println("-----------------[P]------------------");
            P.print(8, 5);
            
            Matrix BPB = B.transpose().times(P).times(B);
            Matrix BPA = B.transpose().times(P).times(A);
            
            Matrix F = R.plus(BPB).inverse().times(BPA.plus(N.transpose()));
            System.out.println("-----------------[F]------------------");
            F.print(8, 5);
            
            Matrix K = F.uminus();//.times(0);
            
            System.out.println("-----------------[A]------------------");
            A.print(8, 5);
            
            System.out.println("-----------------[B]------------------");
            B.print(8, 5);
            
            System.out.println("-----------------[K]------------------");
            K.print(8, 5);
            
            Matrix ABK = A.plus(B.times(K));
            System.out.println("-----------------[A+BK]------------------");
            ABK.print(8, 5);
            
            SigmaX0 = new Matrix(dynamic.n(), dynamic.n());
            for(int i=0; i<dynamic.m(); i++){
                SigmaX0.set(i, i, Math.pow(std_position,2));
            }
            for(int i=dynamic.m(); i<dynamic.n(); i++){
                SigmaX0.set(i, i, Math.pow(std_velocity,2));
            }
            SigmaWt = SigmaX0.copy();
            
            //Matrix sigma = new Matrix(approach.N()*2, approach.N()*2);]
            Sigma = new Matrix[approach.Waypoints()+1];
            Sigma[0] = SigmaX0.copy();
            System.out.println("-----------------[sigma("+0+")]------------------");
            Sigma[0].print(8, 5);
            
            for(int t=0; t<approach.Waypoints(); t++){
                Sigma[t+1] = ABK.times(Sigma[t]).times(ABK.transpose()).plus(SigmaWt);
            }
            System.out.println("-----------------[sigma("+approach.Waypoints()+")]------------------");
            Sigma[approach.Waypoints()].print(8, 5);
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
