/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.uncertainty;

import Jama.Matrix;
import ProOF.apl.UAV.gen.linear.LinearApproach;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;
import ProOF.com.Linker.LinkerApproaches;

/**
 *
 * @author marcio
 */
public class LinearPerfectControlUncertainty extends pLinearControlUncertainty{
    
    
    private final LinearApproach approach;
    
    private pLinearStateUncertainty unc;
    private oLinearDynamic dynamic;
    
    private Matrix K_Sigma[];
    private Matrix K;

    public LinearPerfectControlUncertainty(LinearApproach approach) {
        this.approach = approach;
    }
    
    @Override
    public String name() {
        return "Control Perfect Unc.";
    }

    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        unc = link.need(pLinearStateUncertainty.class, unc);
        dynamic = link.need(oLinearDynamic.class, dynamic);
    }
    
    @Override
    public double[] onlineFeedBack(double u[], double z[], double x[]) throws Exception{
        check();
        //ut = e(ut) + K(xt-e(xt));
        double r[] = new double[u.length];
        for(int i=0; i<u.length; i++){
            r[i] = u[i];
            for(int j=0; j<z.length; j++){
                r[i] += K.get(i, j)*(z[j]-x[j]);
            }
        }
        return r;
    }
    
    private void check() throws Exception{
        //A + B*K = 0 --> B*K = -A --> K = -B^(-1)*A
        if(K_Sigma==null){
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
//            System.out.println("-----------------[P]------------------");
//            P.print(8, 5);
            
            Matrix BPB = B.transpose().times(P).times(B);
            Matrix BPA = B.transpose().times(P).times(A);
            
            Matrix F = R.plus(BPB).inverse().times(BPA.plus(N.transpose()));
//            System.out.println("-----------------[F]------------------");
//            F.print(8, 5);
            
            K = F.uminus();//.times(0);
            
//            System.out.println("-----------------[A]------------------");
//            A.print(8, 5);
//            
//            System.out.println("-----------------[B]------------------");
//            B.print(8, 5);
            
//            System.out.println("-----------------[K]------------------");
//            K.print(8, 5);
            
            Matrix ABK = A.plus(B.times(K));
//            System.out.println("-----------------[A+BK]------------------");
//            ABK.print(8, 5);
            
            Matrix SigmaInit = new Matrix(dynamic.n(), dynamic.n());
            for(int i=0; i<SigmaInit.getRowDimension(); i++){
                for(int j=0; j<SigmaInit.getColumnDimension(); j++){
                    SigmaInit.set(i, j, unc.Sigma(0, i, j));
                }
            }
            
            Matrix Sigma = SigmaInit.copy();//new Matrix(approach.N()*2, approach.N()*2);
            K_Sigma = new Matrix[approach.Waypoints()];
            for(int t=0; t<approach.Waypoints(); t++){
                Sigma = ABK.times(Sigma).times(ABK.transpose()).plus(SigmaInit);
                
//                System.out.println("-----------------[sigma]------------------");
//                Sigma.print(8, 5);
                
                
                K_Sigma[t] = K.uminus().times(Sigma);
                
            }
            System.out.println("----------------[K_sigma("+0+")]-----------------");
            K_Sigma[0].print(8, 5);
            System.out.println("----------------[K_sigma("+(approach.Waypoints()-1)+")]-----------------");
            K_Sigma[approach.Waypoints()-1].print(8, 5);
        }
    }

    @Override
    public double Sigma(int t, int row, int col) throws Exception {
        check();
        //System.out.println("uncU = "+K_Sigma[t].get(row, col));
        return K_Sigma[t].get(row, col);
    }

    @Override
    public int N() throws Exception {
        return approach.N();
    }
    
}
