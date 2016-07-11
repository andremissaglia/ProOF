/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.vehicle.parts;

import ProOF.apl.UAV.gen.linear.LinearModel;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.language.Factory;
import java.io.File;
import java.util.Scanner;

/**
 *
 * @author marcio
 */
public class fLinearDynamic extends Factory<oLinearDynamic>{
    public static final fLinearDynamic obj = new fLinearDynamic();
    @Override
    public String name() {
        return "Dynamic";
    }
    @Override
    public oLinearDynamic build(int index) {  //build the operators
        switch(index){
            case 0: return new AirFree();
            case 1: return new AirResistence();
            case 2: return new BlackmorePure();
            case 3: return new BlackmoreBased();
            case 4: return new Rover();
            case 5: return new FileMatrix();
            case 6: return new FileMatrixClosedLoop();
            case 7: return new AirFreePV();
            case 8: return new AirResistence2();
        }
        return null;
    }
    
    public static class AirFree extends oLinearDynamic<LinearSystem, LinearModel>{
        private final DynamicDt waypoint = new DynamicDt();
        @Override
        public String name() {
            return "Air-Free";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            link.add(waypoint);
        }
        @Override
        public void start(LinearSystem approach) throws Exception {
            super.letBeAirFree(2*approach.N(), approach.N(), waypoint.dt());
        }
        @Override
        public void addConstraints(LinearSystem approach, LinearModel model) throws Exception {
            super.addConstraints(approach, model); //To change body of generated methods, choose Tools | Templates.
        }
    }
    private class AirResistence extends oLinearDynamic<LinearSystem, LinearModel>{
        private final PowerDt waypoint = new PowerDt();
        @Override
        public String name() {
            return "Air-Resistence";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            link.add(waypoint);
        }
        @Override
        public void start(LinearSystem approach) throws Exception {
            //System.out.println("massa="+mass);
            super.letBeAirResistence(2*approach.N(), approach.N(), waypoint.mass, waypoint.maxControl(), waypoint.maxVelocity(), waypoint.dt());
            //print();
        }
        @Override
        public void addConstraints(LinearSystem approach, LinearModel model) throws Exception {            
            super.addConstraints(approach, model); //To change body of generated methods, choose Tools | Templates.
        }
    }
    private class BlackmorePure extends oLinearDynamic<LinearSystem, LinearModel>{
        private final FixedDt waypoint = new FixedDt(1.0);  //it is suposed dt=1.0 second
        @Override
        public String name() {
            return "Blackmore";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            link.add(waypoint);
        }
        @Override
        public void start(LinearSystem approach) throws Exception {
            super.letBeBlackmorePure(2*approach.N(), approach.N());
        }
        @Override
        public void addConstraints(LinearSystem approach, LinearModel model) throws Exception {
            super.addConstraints(approach, model); //To change body of generated methods, choose Tools | Templates.
        }
    }
    private class BlackmoreBased extends oLinearDynamic<LinearSystem, LinearModel>{
        private final FixedDt waypoint = new FixedDt(1.0);  //it is suposed dt=1.0 second
        private double pr;
        private double vr;
        @Override
        public String name() {
            return "Blackmore-Based";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            link.add(waypoint);
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            pr = link.Dbl("pr", 0.7869, 0.0001, 0.9999);
            vr = link.Dbl("vr", 0.6065, 0.0001, 0.9999);
        }
        @Override
        public void start(LinearSystem approach) throws Exception {
            super.letBeBlackmoreBased(2*approach.N(), approach.N(), pr, vr);
        }
        @Override
        public void addConstraints(LinearSystem approach, LinearModel model) throws Exception {
            super.addConstraints(approach, model); //To change body of generated methods, choose Tools | Templates.
        }
    }
    public static class Rover extends oLinearDynamic<LinearSystem, LinearModel>{
        private final VelocityDt waypoint = new VelocityDt();
        @Override
        public String name() {
            return "Rover";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            link.add(waypoint);
        }
        @Override
        public void start(LinearSystem approach) throws Exception {
            super.letBeRover(2*approach.N(), approach.N(), waypoint.dt());
        }
        @Override
        public void addConstraints(LinearSystem approach, LinearModel model) throws Exception {
            super.addConstraints(approach, model); //To change body of generated methods, choose Tools | Templates.
        }
    }
    public static class FileMatrix extends oLinearDynamic<LinearSystem, LinearModel>{
        private final DynamicDt waypoint = new DynamicDt();
        private File file;
        @Override
        public String name() {
            return "File-Matrix";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            link.add(waypoint);
        }

        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            file = link.File("Dynamic", null, "txt");
        }
        
        @Override
        public void start(LinearSystem approach) throws Exception {
            super.check(approach.N()*2,approach.N());
            
            Scanner sc = new Scanner(file);
            sc.nextLine();
            for(int i=0; i<A.length; i++){
                for(int j=0; j<A[i].length; j++){
                    A[i][j] = sc.nextDouble();
                }
                sc.nextLine();
            }
            sc.nextLine();
            for(int i=0; i<B.length; i++){
                for(int j=0; j<B[i].length; j++){
                    B[i][j] = sc.nextDouble();
                }
                sc.nextLine();
            }
            sc.close();
            
            //print();
        }
        @Override
        public void addConstraints(LinearSystem approach, LinearModel model) throws Exception {
            super.addConstraints(approach, model); //To change body of generated methods, choose Tools | Templates.
        }
    }
    public static class FileMatrixClosedLoop extends oLinearDynamic<LinearSystem, LinearModel>{
        private final DynamicDt waypoint = new DynamicDt();
        private File file;
        @Override
        public String name() {
            return "File-Matrix-ClosedLoop";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            link.add(waypoint);
        }

        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            file = link.File("Dynamic", null, "txt");
        }
        
        @Override
        public void start(LinearSystem approach) throws Exception {
            super.check(approach.N()*2,approach.N());
            
           
            Scanner sc = new Scanner(file);
            while(sc.hasNextLine()){
                String type = sc.nextLine();
                if(type.contains("<A>")){
                    for(int i=0; i<A.length; i++){
                        for(int j=0; j<A[i].length; j++){
                            A[i][j] = sc.nextDouble();
                        }
                        sc.nextLine();
                    }
                }else if(type.contains("<B>")){
                    for(int i=0; i<B.length; i++){
                        for(int j=0; j<B[i].length; j++){
                            B[i][j] = sc.nextDouble();
                        }
                        sc.nextLine();
                    }
                }else{
                    break;
                }
            }
            sc.close();
            
            //print();
        }
        @Override
        public void addConstraints(LinearSystem approach, LinearModel model) throws Exception {
            super.addConstraints(approach, model); //To change body of generated methods, choose Tools | Templates.
        }
    }
    public static class AirFreePV extends oLinearDynamic<LinearSystem, LinearModel>{
        private final VelocityDt waypoint = new VelocityDt();
        @Override
        public String name() {
            return "Air-Free-PV";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            link.add(waypoint);
        }
        @Override
        public void start(LinearSystem approach) throws Exception {
            super.letBeAirFreePV(approach.N(), waypoint.dt());
        }
        @Override
        public void addConstraints(LinearSystem approach, LinearModel model) throws Exception {
            super.addConstraints(approach, model); //To change body of generated methods, choose Tools | Templates.
        }
    }
    private class AirResistence2 extends oLinearDynamic<LinearSystem, LinearModel>{
        private final AirDt waypoint = new AirDt();
        @Override
        public String name() {
            return "Air-Resistence2";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            link.add(waypoint);
        }
        @Override
        public void start(LinearSystem approach) throws Exception {
            //System.out.println("massa="+mass);
            super.letBeAirResistence(2*approach.N(), approach.N(), waypoint.maxControl(), waypoint.maxVelocity(), waypoint.dt());
            //print();
        }
        @Override
        public void addConstraints(LinearSystem approach, LinearModel model) throws Exception {            
            super.addConstraints(approach, model); //To change body of generated methods, choose Tools | Templates.
        }
    }
    
    private static class DynamicDt extends pLinearWaypoints{
        private int waypoints;
        private double time_horizon;
        private double maxVelocity;
        private double maxControl;
        
        @Override
        public String name() {
            return "Dynamic dt";
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            waypoints = link.Int("Waypoints", 20, 2, 1000);
            time_horizon = link.Dbl("time horizon", 20.0, 0.001, 1e5);
            maxVelocity = link.Dbl("Max-velocity", 3.0, 1e-5, 1e6);
            maxControl = link.Dbl("Max-control", 1.0, 1e-5, 1e6);
        }
        @Override
        public double timeHorizon() {
            return time_horizon;
        }
        @Override
        public int Waypoints() {
            return waypoints;
        }
        @Override
        public double dt(){
            return time_horizon/waypoints;
        }
        @Override
        public double maxVelocity() {
            return maxVelocity;
        }
        @Override
        public double maxControl() {
            return maxControl;
        }
    }
    private static class PowerDt extends pLinearWaypoints{
        private int waypoints;
        private double time_horizon;
        private double maxVelocity;
        private double power;
        private double mass;
        @Override
        public String name() {
            return "Power dt";
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            waypoints = link.Int("Waypoints", 20, 2, 1000);
            time_horizon = link.Dbl("time horizon", 20.0, 0.001, 1e5);
            maxVelocity = link.Dbl("Max-velocity", 3.0, 1e-5, 1e6);
            power = link.Dbl("Power(hp)", 0.00402144772117962466487935656836, 1e-5, 1e5);   //UAV with Umax = 1.0, Vmax = 3.0, mass = 1.0 
            mass = link.Dbl("mass", 1.0, 0.001, 1e5);
            //power = link.Dbl("Power(hp)", 0.01, 1e-5, 1e5);
            //mass = link.Dbl("mass", 2.4866666, 0.001, 1e5);
        }
        @Override
        public double timeHorizon() {
            return time_horizon;
        }
        @Override
        public int Waypoints() {
            return waypoints;
        }
        @Override
        public double dt(){
            return time_horizon/waypoints;
        }
        @Override
        public double maxVelocity() {
            return maxVelocity;
        }
        @Override
        public double maxControl() {
            return power*746.0/(mass*maxVelocity);
        }
    }
    private static class AirDt extends pLinearWaypoints{
        private int waypoints;
        private double time_horizon;
        private double v_term;
        private double u_max;
        @Override
        public String name() {
            return "Power dt";
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            waypoints = link.Int("Waypoints", 20, 2, 1000, "T > 'time horizon' * U-max / V-term");
            time_horizon = link.Dbl("time horizon", 20.0, 0.001, 1e5);
            v_term = link.Dbl("V-term", 3.0, 1e-5, 1e6);
            u_max = link.Dbl("U-max", 1.0, 1e-5, 1e5);   //UAV with Umax = 1.0, Vmax = 3.0, mass = 1.0 
        }
        @Override
        public double timeHorizon() {
            return time_horizon;
        }
        @Override
        public int Waypoints() {
            return waypoints;
        }
        @Override
        public double dt(){
            return time_horizon/waypoints;
        }
        @Override
        public double maxVelocity() {
            return v_term;
        }
        @Override
        public double maxControl() {
            return u_max;
        }
    }
    private static class FixedDt extends pLinearWaypoints{
        private final double dt;
        private int waypoints;
        private double maxVelocity;
        private double maxControl;

        public FixedDt(double dt) {
            this.dt = dt;
        }
        @Override
        public String name() {
            return "Fixed dt";
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            waypoints = link.Int("Waypoints", 20, 2, 1000);
            maxVelocity = link.Dbl("Max-velocity", 3.0, 1e-5, 1e6);
            maxControl = link.Dbl("Max-control", 1.0, 1e-5, 1e6);
        }
        @Override
        public double timeHorizon() {
            return waypoints*dt;
        }
        @Override
        public int Waypoints() {
            return waypoints;
        }
        @Override
        public double dt(){
            return dt;
        }
        @Override
        public double maxVelocity() {
            return maxVelocity;
        }
        @Override
        public double maxControl() {
            return maxControl;
        }
    }
    private static class VelocityDt extends pLinearWaypoints{
        private int waypoints;
        private double time_horizon;
        private double maxControl;
        
        @Override
        public String name() {
            return "Velocity dt";
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            super.parameters(link); //To change body of generated methods, choose Tools | Templates.
            waypoints = link.Int("Waypoints", 20, 2, 1000);
            time_horizon = link.Dbl("time horizon", 20.0, 0.001, 1e5);
            maxControl = link.Dbl("Max-control", 1.0, 1e-5, 1e6);
        }
        @Override
        public double timeHorizon() {
            return time_horizon;
        }
        @Override
        public int Waypoints() {
            return waypoints;
        }
        @Override
        public double dt(){
            return time_horizon/waypoints;
        }
        @Override
        public double maxVelocity() {
            return 0;
        }
        @Override
        public double maxControl() {
            return maxControl;
        }
    }
}
