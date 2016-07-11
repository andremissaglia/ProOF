/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear;

import ProOF.CplexExtended.CplexExtended;
import ProOF.CplexExtended.iCplexExtract;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.abst.UAVModel;
import ProOF.com.Linker.LinkerResults;
import ilog.concert.IloException;
import java.awt.Color;
 
/**
 *
 * @author marcio
 * @param <App>
 */
public abstract class LinearModel<App extends LinearSystem> extends UAVModel<App>{
    public final LinearState states[];
    public final LinearControl controls[]; 
    private boolean extract = false;
    public LinearModel(App approach, String name, CplexExtended cplex) throws IloException, Exception {
        super(name, approach, cplex);
        states = approach.vehicle().build_states(approach, this);
        controls = approach.vehicle().build_controls(approach, this);
        approach.vehicle().addConstraints(approach, this);
    }
    @Override
    public void extract(iCplexExtract ext, Callback type) throws Exception {
        super.extract(ext, type);
        for(LinearState s : states){
            s.extract(ext);
        }
        for(LinearControl c : controls){
            c.extract(ext);
        }
        extract = true;
    }

    public boolean isExtract() {
        return extract;
    }
    
    
    @Override
    public void paint(Graphics2DReal gr, double size) throws Exception {
        if(isExtract()){
            //gr.g2.setStroke(new BasicStroke(3));
            Color color = Color.BLACK;
            for(int t=0; t<states.length-1; t++){//plot lines
                states[t].plot.drawLine(gr, color, states[t+1].plot);
            }
            for(int t=0; t<states.length; t++){
                states[t].plot.fillPoint(gr, color, 0.003*size);
            }
            //gr.g2.setStroke(new BasicStroke(1));
//            for(int t=0; t<states.length; t++){
//                states[t].plot.drawLabel(gr, Color.BLACK, 0.004*size);
//            }
        }
    }

    @Override
    public void results(LinkerResults link) throws Exception {
        super.results(link); //To change body of generated methods, choose Tools | Templates.
        if(isFeasible()){
            System.out.println("------------------------ [ states ] ------------------------");
            for(LinearState state : states){
                double x[] = cplex.getValues(state.x);
                for(int i=0; i<x.length; i++){
                    System.out.printf("%5.2f ", x[i]);
                }
                System.out.println();
            }
            System.out.println("------------------------ [ controls ] ------------------------");
            for(LinearControl control : controls){
                double u[] = cplex.getValues(control.u);
                for(int i=0; i<u.length; i++){
                    System.out.printf("%5.2f ", u[i]);
                }
                System.out.println();
            }
        }
    }
    
}
