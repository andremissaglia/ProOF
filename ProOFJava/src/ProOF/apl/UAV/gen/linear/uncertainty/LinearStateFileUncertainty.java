/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.uncertainty;

import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.com.Linker.LinkerParameters;
import java.io.File;
import java.io.PrintStream;
import java.util.Scanner;

/**
 *
 * @author marcio
 */
public class LinearStateFileUncertainty extends pLinearStateUncertainty{
    private final LinearSystem approach;
    private File file;
    private double Sigma[][][];

    public LinearStateFileUncertainty(LinearSystem approach) {
        this.approach = approach;
    }
    
    @Override
    public String name() {
        return "Linear State Unc. File";
    }

    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); //To change body of generated methods, choose Tools | Templates.
        file = link.File("Uncertainty", null, "txt");
    }

    @Override
    public void start() throws Exception {
        super.load(); //To change body of generated methods, choose Tools | Templates.
        Scanner sc = new Scanner(file);
        Sigma = new double[approach.Waypoints()+1][approach.N()*2][approach.N()*2];
        for(int t=0; t<approach.Waypoints()+1; t++){
            sc.nextLine();
            for(int i=0; i<approach.N()*2; i++){
                for(int j=0; j<approach.N()*2; j++){
                    Sigma[t][i][j] = sc.nextDouble();
                }
                sc.nextLine();
            }
        }
        sc.close();
        print_covariance(System.out);
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
