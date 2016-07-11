/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.vehicle.parts;

import ProOF.apl.UAV.abst.vehicle.parts.oDynamic;
import ProOF.apl.UAV.gen.linear.LinearModel;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ilog.concert.IloNumExpr;

/**
 *
 * @author marcio
 * @param <App>
 * @param <Model>
 */
public abstract class oLinearDynamic<App extends LinearSystem, Model extends LinearModel> extends oDynamic<App, Model>{
    public double A[][];
    public double B[][];
    
    public abstract void start(App approach) throws Exception;
    
    public void print(){
        System.out.println("-------------------------[A]-------------------------");
        for(int i=0; i<A.length; i++){
            for(int j=0; j<A[i].length; j++){
                System.out.printf("%8.3f ", A[i][j]);
            }
            System.out.println();
        }
        System.out.println("-------------------------[B]-------------------------");
        for(int i=0; i<B.length; i++){
            for(int j=0; j<B[i].length; j++){
                System.out.printf("%8.3f ", B[i][j]);
            }
            System.out.println();
        }
    }
    
    @Override
    public void addConstraints(App approach, Model model) throws Exception {
        for(int t=0; t<model.controls.length; t++){
            for(int i=0; i<A.length; i++){
                IloNumExpr exp = null;
                for(int j=0; j<A[i].length; j++){
                    exp = model.cplex.SumProd(exp, A[i][j], model.states[t].x[j]);
                }
                for(int j=0; j<B[i].length; j++){
                    exp = model.cplex.SumProd(exp, B[i][j], model.controls[t].u[j]);
                }
                model.cplex.addEq(model.states[t+1].x[i], exp, "DynamicLinear["+(i+1)+"]");
            }
        }
    }
    
    public double[] next(double x[], double u[]) throws Exception{
        double r[] = new double[x.length];
        for(int i=0; i<A.length; i++){
            r[i] = 0;
            for(int j=0; j<A[i].length; j++){
                r[i] += A[i][j] * x[j];
            }
            for(int j=0; j<B[i].length; j++){
                r[i] += B[i][j] * u[j];
            }
        }
        return r;
    }
    public double[] nextContinus(double x[], double u[], int divisor) throws Exception{
        double r[] = new double[x.length];
        for(int i=0; i<A.length; i++){
            r[i] = 0;
            for(int j=0; j<A[i].length; j++){
                if(i==j){
                    r[i] += A[i][j] * x[j];
                }else{
                    r[i] += A[i][j] * x[j] / divisor;
                }
            }
            for(int j=0; j<B[i].length; j++){
                if(i==j){
                    r[i] += B[i][j] * u[j] / (divisor*divisor);
                }else{
                    r[i] += B[i][j] * u[j] / divisor;
                }
                
            }
        }
        return r;
    }
    
    public void setMatrixes(double[][] A, double[][] B) {
        this.A = A;
        this.B = B;
    }
    
    public final int n(){
        return A.length;
    }
    public final int m(){
        return B[0].length;
    }
    
    public boolean check(int n, int m){
        if(this.A==null || this.B==null){
            setMatrixes(new double[n][n], new double[n][m]);
            return true;
        }
        return false;
    }
    
    /**
     * Blackmore 2011 pure matrixes, assume dt=1, pr=0.7869, vr=0.6065
     * @param n
     * @param m
     */
    public void letBeBlackmorePure(final int n, final int m) {
        letBeBlackmoreBased(n, m, 0.7869, 0.6065);
    }
    
    /**
     * Blackmore 2011 based matrixes, assume dt=1
     * <pre>
     * -------------- example 2D -------------
     * A = new double[][]{
     *      {1,     0,      pr,     0},
     *      {0,     1,      0,      pr},
     *      {0,     0,      vr,     0},
     *      {0,     0,      0,      vr},
     * };
     * B = new double[][]{
     *      {1-pr,  0},
     *      {0,     1-pr},
     *      {1-vr,  0},
     *      {0,     1-vr},
     * };
     * </pre>
     * @param n
     * @param m
     * @param pr
     * @param vr 
     */
    public void letBeBlackmoreBased(final int n, final int m, final double pr, final double vr) {
        if(check(n, m)){
            for(int i=0; i<m; i++){
                A[i][i] = 1.0;
                A[i][m+i] = pr;
                A[m+i][m+i] = vr;
            }
            for(int i=0; i<m; i++){
                B[i][i] = 1-pr;
                B[i+m][i] = 1-vr;
            }
        }
    }
    
    /**
     * <pre>
     * -------------- example 2D -------------
     * A = new double[][]{
     *      {1,     0,      dt,     0},
     *      {0,     1,      0,      dt},
     *      {0,     0,      1,      0},
     *      {0,     0,      0,      1},
     * };
     * B = new double[][]{
     *      {dt2,   0},
     *      {0,     dt2},
     *      {dt,    0},
     *      {0,     dt},
     * };
     * </pre>
     * @param n
     * @param m
     * @param dt 
     */
    public void letBeAirFree(final int n, final int m, final double dt) {
        if(check(n,m)){
            for(int i=0; i<n; i++){
                A[i][i] = 1.0;
            }
            for(int i=0; i<m; i++){
                A[i][m+i] = dt;
            }
            for(int i=0; i<m; i++){
                B[i][i] = dt*dt/2;
                B[i+m][i] = dt;
            }
        }
    }
    
    /**
     * <pre>
     * -------------- example 2D -------------
     * A = new double[][]{
     *      {1,     0},
     *      {0,     1},
     * };
     * B = new double[][]{
     *      {dt,    0},
     *      {0,     dt},
     * };
     * </pre>
     * @param n
     * @param dt 
     */
    public void letBeAirFreePV(final int n, final double dt) {
        if(check(n,n)){
            for(int i=0; i<n; i++){
                A[i][i] = 1.0;
            }
            for(int i=0; i<n; i++){
                B[i][i] = dt;
            }
        }
    }
    
    /**
     * <pre>
     * -------------- example 2D -------------
     * A = new double[][]{
     *      {1,     0,      pr,     0},
     *      {0,     1,      0,      pr},
     *      {0,     0,      vr,      0},
     *      {0,     0,      0,      vr},
     * };
     * B = new double[][]{
     *      {dt2,   0},
     *      {0,     dt2},
     *      {dt,    0},
     *      {0,     dt},
     * };
     * </pre>
     * @param n
     * @param m
     * @param mass
     * @param full_throttle
     * @param terminal_velocity
     * @param dt 
     */
    public void letBeAirResistence(final int n, final int m, double mass, double full_throttle, double terminal_velocity, final double dt) {
        if(check(n, m)){
            final double Kd = calculateAirResistance(mass, full_throttle, terminal_velocity);
            final double pr = calculatePositionReduction(mass, terminal_velocity, Kd, dt);
            final double vr = calculateVelocityReduction(mass, terminal_velocity, Kd, dt);
            
            System.out.printf("dt   = %g\n", dt);
            System.out.printf("mass = %g\n", mass);
            System.out.printf("Umax = %g\n", full_throttle);
            System.out.printf("Vmax = %g\n", terminal_velocity);
            System.out.printf("Kd = mass*Umax/(Vmax*Vmax)  = %g\n", Kd);
            System.out.printf("vr = 1 - Kd * Vmax * dt / mass  = %g\n", vr);
            System.out.printf("pr = dt - Kd * Vmax * dt * dt / (2*mass)  = %g\n", pr);
//            
            for(int i=0; i<m; i++){
                A[i][i] = 1.0;
                A[i][m+i] = pr;
                A[m+i][m+i] = vr;
            }
            final double dt2 = dt*dt/2;
            for(int i=0; i<m; i++){
                B[i][i] = dt2;
                B[i+m][i] = dt;
            }
        }
    }
    
    public void letBeAirResistence(final int n, final int m, double full_throttle, double terminal_velocity, final double dt) {
        if(check(n, m)){
            final double K = K(full_throttle, terminal_velocity);
            final double Kp = Kp(terminal_velocity, K, dt);
            final double Kv = Kv(terminal_velocity, K, dt);
            
            System.out.printf("dt   = %g\n", dt);
            System.out.printf("Umax = %g\n", full_throttle);
            System.out.printf("Vterm = %g\n", terminal_velocity);
            System.out.printf("K  = Umax/(Vmax*Vmax)  = %g\n", K);
            System.out.printf("Kv = 1 - Kd * Vmax * dt  = %g\n", Kv);
            System.out.printf("Kp = dt - Kd * Vmax * dt^2/2  = %g\n", Kp);
//            
            for(int i=0; i<m; i++){
                A[i][i] = 1.0;
                A[i][m+i] = Kp;
                A[m+i][m+i] = Kv;
            }
            final double dt2 = dt*dt/2;
            for(int i=0; i<m; i++){
                B[i][i] = dt2;
                B[i+m][i] = dt;
            }
        }
    }
    
    public void  letBeRover(final int n, final int m, double dt) {
        if(check(n,m)){
            for(int i=0; i<m; i++){
                A[i][i] = 1.0;
            }
            for(int i=0; i<m; i++){
                B[i][i] = dt;
            }
        }
    }
    public static double K(double full_throttle, double terminal_velocity){
        return full_throttle / Math.pow(terminal_velocity,2);
    }
    public static double Kp(double terminal_velocity, double K, double dt){
        return dt - K * terminal_velocity * Math.pow(dt,2)/2;
    }
    public static double Kv(double terminal_velocity, double K, double dt){
        return 1 - K * terminal_velocity * dt;
    }
    
    public static double calculateAirResistance(double mass, double full_throttle, double terminal_velocity){
        return (mass * full_throttle) / Math.pow(terminal_velocity,2);
    }
    public static double calculateVelocityReduction(double mass, double terminal_velocity, double air_resistance, double dt){
        return 1 - air_resistance * terminal_velocity * dt / mass;
    }
    public static double calculatePositionReduction(double mass, double terminal_velocity, double air_resistance, double dt){
        return dt - air_resistance * terminal_velocity * dt * dt / (2*mass);
    }

    
    
}
