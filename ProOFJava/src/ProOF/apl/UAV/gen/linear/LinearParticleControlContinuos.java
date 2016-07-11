/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear;

import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearControlUncertainty;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearStateUncertainty;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;
import ProOF.apl.UAV.map.Obstacle;
import java.awt.Color;
import jsc.distributions.Normal;

/**
 *
 * @author marcio
 */
public class LinearParticleControlContinuos {

    private final pLinearStateUncertainty unc;
    private final oLinearDynamic dynamic;

    private final int size = 10;
    private final LinearPlotState states[];
    private final Obstacle obstacles[];
    private boolean hasColision;
    
    public LinearParticleControlContinuos(pLinearStateUncertainty unc, oLinearDynamic dynamic, LinearState[] states, LinearControl[] controls, Obstacle obstacles[]) throws Exception {
        this.unc = unc;
        this.dynamic = dynamic;
        this.obstacles = obstacles;

        this.states = new LinearPlotState[controls.length*size+1];
        this.hasColision = false;
        calculateRealContinuos(states, controls);
    }

    public final boolean hasColision() {
        return hasColision;
    }
    
    /**
     * it just nothing supposed
     * @param t
     * @param x
     * @throws Exception 
     */
    private void calculateRealContinuos(LinearState[] states, LinearControl[] controls) throws Exception{
        double x[] = states[0].x();
        disturbanceContinuos(x, 1, true);
        this.states[0] = new LinearPlotState(x, 0);
        colisions(x); 
        
        int n=0;
        for (int t = 0; t < controls.length; t++) {
            //double u[] = uncU.onlineFeedBack(controls[t].u(), x, states[t].x());
            double u[] = controls[t].u();
            for (int s = 0; s < size; s++) {
                x = dynamic.nextContinus(x, u, size);
                disturbanceContinuos(x, size, false);
                this.states[n + 1] = new LinearPlotState(x, n + 1);
                colisions(x); 
                n++;
            }
        }
    }
    private void colisions(double x[]) throws Exception{
        for(Obstacle obs : obstacles){
            if(obs.distance(x)<1e-6){
                hasColision = true;
            }
        }
    }

    private void disturbanceContinuos(double x[], int divisor, boolean isInitial) throws Exception {
        double w[] = new double[x.length];
        for (int i = 0; i < x.length; i++) {
            w[i] = 0;
            for (int j = 0; j < x.length; j++) {
                double sigma;
                if(isInitial){
                    sigma = unc.SigmaX0(i, j);
                }else{
                    sigma = unc.SigmaWt(i, j);
                }
                if (sigma > 1e-6) {
                    Normal n = new Normal(0, Math.sqrt(sigma/divisor));
                    w[i] += n.random();
                }
            }
        }
        for (int i = 0; i < x.length; i++) {
            x[i] += w[i];
        }
    }

    public void paint(Graphics2DReal gr, double size) throws Exception {
        if(hasColision){
            for (LinearPlotState state : states) {
                state.fillPoint(gr, Color.RED, 0.001*size);
            }
        }else{
            for (LinearPlotState state : states) {
                state.fillPoint(gr, Color.PINK, 0.001*size);
            }
        }
            
    }
    public void paint(Graphics2DReal gr, double size, int end) throws Exception {
        for(int t=0; t<=end; t++){
            states[t].fillPoint(gr, Color.PINK, 0.001*size);
        }
    }
}
