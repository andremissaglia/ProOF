/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.uncertainty;

import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.com.Linker.LinkerParameters;
import java.io.PrintStream;

/**
 *
 * @author marcio
 */
public class LinearStateUncertainty extends pLinearStateUncertainty{
    private final LinearSystem approach;
    
    private double std_position;
    private double std_velocity;
    private double inc_factor;

    public LinearStateUncertainty(LinearSystem approach) {
        this.approach = approach;
    }
    
    @Override
    public String name() {
        return "Linear State Unc.";
    }

    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); //To change body of generated methods, choose Tools | Templates.
        std_position = link.Dbl("std-position", 0.05, 0.0, 1e6);
        std_velocity = link.Dbl("std-velocity", 0.00, 0.0, 1e6);
        inc_factor = link.Dbl("inc-factor", 1.0, 0.0, 1e6);
    }
    
    @Override
    public double Sigma(int t, int row, int col) throws Exception {
        if(row==col){
            if(row<approach.N()){ //position
                return Math.pow(std_position*(1+(inc_factor*t)/approach.Waypoints()) ,2);
            }else{              //velocity
                return Math.pow(std_velocity*(1+(inc_factor*t)/approach.Waypoints()) ,2);
            }
        }
        return 0.0;
    }
    
    @Override
    public int N() throws Exception {
        return approach.N();
    }
    @Override
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
        if(row==col){
            if(row<approach.N()){ //position
                return Math.pow(std_position ,2);
            }else{              //velocity
                return Math.pow(std_velocity ,2);
            }
        }
        return 0.0;
    }

    @Override
    public double SigmaWt(int row, int col) throws Exception {
        if(row==col){
            if(row<approach.N()){ //position
                return Math.pow(std_position*inc_factor/approach.Waypoints() ,2);
            }else{              //velocity
                return Math.pow(std_velocity*inc_factor/approach.Waypoints() ,2);
            }
        }
        return 0.0;
    }
}
