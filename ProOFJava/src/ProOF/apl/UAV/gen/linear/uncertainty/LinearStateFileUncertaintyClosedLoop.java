/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.uncertainty;

import Jama.Matrix;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.com.Linker.LinkerParameters;
import java.io.File;
import java.io.PrintStream;
import java.util.Scanner;

/**
 *
 * @author marcio
 */
public class LinearStateFileUncertaintyClosedLoop extends pLinearStateUncertainty{
    private final LinearSystem approach;
    private File file;
    private double Sigma[][][];

    public LinearStateFileUncertaintyClosedLoop(LinearSystem approach) {
        this.approach = approach;
    }
    
    @Override
    public String name() {
        return "Linear State Unc. File Closed Loop";
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); //To change body of generated methods, choose Tools | Templates.
        file = link.File("Uncertainty", null, "txt");
    }

    @Override
    public void start() throws Exception {
        super.load(); //To change body of generated methods, choose Tools | Templates.
//        Scanner sc = new Scanner(file);
//        Sigma = new double[approach.Waypoints()+1][approach.N()*2][approach.N()*2];
//        for(int t=0; t<approach.Waypoints()+1; t++){
//            sc.nextLine();
//            for(int i=0; i<approach.N()*2; i++){
//                for(int j=0; j<approach.N()*2; j++){
//                    Sigma[t][i][j] = sc.nextDouble();
//                }
//                sc.nextLine();
//            }
//        }
//        sc.close();
        
        
        Matrix A = null;
        Matrix B = null;
        Matrix Sigma_x0 = null;
        Matrix Sigma_wt = null;
        
        Matrix Q = null;
        Matrix R = null;
        Matrix K = null;
        
        
        Scanner sc = new Scanner(file);
        while(sc.hasNextLine()){
            String type = sc.nextLine();
            if(type.contains("<A>")){
                A = read(sc, approach.N()*2,approach.N()*2);
            }else if(type.contains("<B>")){
                B = read(sc, approach.N()*2,approach.N());
            }else if(type.contains("<Sigma_x0>")){
                Sigma_x0 = read(sc, approach.N()*2,approach.N()*2);
            }else if(type.contains("<Sigma_wt>")){
                Sigma_wt = read(sc, approach.N()*2,approach.N()*2);
            }else if(type.contains("<Q>")){
                Q = read(sc, approach.N()*2,approach.N()*2);
            }else if(type.contains("<R>")){
                R = read(sc, approach.N(),approach.N());
            }else if(type.contains("<Kt>")){
                K = read(sc, approach.N(), approach.N()*2);
            }else{
                break;
            }
        }
        sc.close();
        
        if(A==null || B==null || Sigma_x0 == null || Sigma_wt == null){
            System.err.println("if (A==null || B==null || Sigma_x0 == null || Sigma_wt == null) occurred -> error");
            System.exit(-1);
        }else if(K==null && Q==null && R==null){
            //malha aberta
            System.out.println("Kt, Q or R not found. Using open loop control");
            System.out.println("Sigma(t+1) = A*Sigma(t)*A' + Sigma_wt, with Sigma(0)=Sigma_x0");
            K = new Matrix(approach.N(), approach.N()*2);
        }else if(K!=null){
            System.out.println("Using Kt defined by user");
            //utiliza o K
        }else{
            System.out.println("Calculating optimal Kt");
            //clacula o K a partir do Q e R
            if(Q==null){
                Q = new Matrix(approach.N()*2, approach.N()*2);
            }else{
                System.out.println("Using Q defined by user");
            }
            if(R==null){
                R = new Matrix(approach.N(), approach.N());
            }else{
                System.out.println("Using R defined by user");
            }
            
            Matrix N = new Matrix(approach.N()*2, approach.N());
            
            
            Matrix P = Matrix.identity(approach.N()*2, approach.N()*2);
            
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
            
            K = F.uminus();//.times(0);
            
        }
        
        System.out.println("-----------------[A]------------------");
        A.print(8, 5);

        System.out.println("-----------------[B]------------------");
        B.print(8, 5);

        System.out.println("-----------------[Kt]------------------");
        K.print(8, 5);

        //=============================== calcula os Sigma =================================
        Matrix ABK = A.plus(B.times(K));
//        System.out.println("-----------------[A+BK]------------------");
//        ABK.print(8, 5);



        Matrix SigmaTemp = Sigma_x0;
        
        
        Sigma = new double[approach.Waypoints()+1][approach.N()*2][approach.N()*2];
        for(int t=0; t<approach.Waypoints()+1; t++){
            for(int i=0; i<SigmaTemp.getRowDimension(); i++){
                for(int j=0; j<SigmaTemp.getColumnDimension(); j++){
                    Sigma[t][i][j] = SigmaTemp.get(i, j);
                }
            }
//            System.out.println("-----------------[sigma(t="+t+")]------------------");
//            SigmaTemp.print(8, 5);
            
            SigmaTemp = ABK.times(SigmaTemp).times(ABK.transpose()).plus(Sigma_wt);
        }
        
        print_covariance(System.out);
    }
    
    private Matrix read(Scanner sc, int row, int col){
        Matrix r = new Matrix(row, col);
        for(int i=0; i<r.getRowDimension(); i++){
            for(int j=0; j<r.getColumnDimension(); j++){
                r.set(i, j, sc.nextDouble());
            }
            sc.nextLine();
        }
        return r;
    }
    
    @Override
    public double Sigma(int t, int row, int col) throws Exception {
        return Sigma[t][row][col];
    }
    
    @Override
    public int N() throws Exception {
        return approach.N();
    }
    public void print_covariance(PrintStream out)throws Exception {
        out.println("<used covariance matrix for each time step>");
        for(int t=0; t<approach.Waypoints()+1; t++){
            out.printf("--------------------------------[%2d]--------------------------------\n", t);
            for(int i=0; i<2*N(); i++){
                for(int j=0; j<2*N(); j++){
                    out.printf("%g, ", Sigma(t, i, j));
                }
                out.println();
            }
        }
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
