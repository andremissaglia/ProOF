/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.gen.linear.LinearState;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.apl.UAV.map.Plot;
import ProOF.com.Linker.LinkerResults;
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;
import java.util.Locale;
import javax.swing.JComboBox;
import static ProOF.apl.UAV.mission.Blackmore.BlackmorePlot.ALLOC.*;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.com.Linker.LinkerParameters;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Rectangle;

/**
 *
 * @author marcio
 */
public class BlackmorePlot extends Plot{
    private final LinearSystem approach;
    private BlackmoreInstance  inst;
//    private final int width = 1100;
//    private final int height = 1030;
    
    private BlackmoreModel model = null;
    private final LinkedList<BlackmoreModel> buffer_model = new LinkedList<BlackmoreModel>();

    
    
     
    public static enum ALLOC{ ALL, FRR, CRT }
    private ALLOC type = ALL;
    
    private JComboBox cb_model;
    private JComboBox cb_alloc;

    public BlackmorePlot(LinearSystem approach) {
        this.approach = approach;
    }

    public void setInst(BlackmoreInstance inst) {
        this.inst = inst;
    }
    
    
    
    
    @Override
    public String name() {
        return "Blackmore Plot";
    }

    private double resolution;
    private boolean savePNG;
    private boolean saveGIF;
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); //To change body of generated methods, choose Tools | Templates.
        savePNG = link.Bool("Save png", true);
        saveGIF = link.Bool("Save gif", true);
        resolution = link.Dbl("resolution", 2.5, 0.1, 10.0, "factor of resolution to save the '.png' results");
    }
    
    @Override
    public void start() throws Exception {
        super.start(); //To change body of generated methods, choose Tools | Templates.
        setTitle(name()+" : "+(approach.N())+"D");
        Config(1100, 1030);
        
        
        goTo(inst.rect, 0.15);

//goTo(approach.inst.rectangle(), 0.05);
//        
        //goMove(0, -10.30/2-1.25);   //choise the center
        //goMove(10, 10);   //choise the center
        //goZoom(0.3);            //spefic the zoom
//                
        
        if(isPlot()){
            cb_alloc = new JComboBox(ALLOC.values());
            cb_alloc.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    type = ALLOC.values()[cb_alloc.getSelectedIndex()];
                    //System.out.println("chooise Alloc = "+type.name());
                    repaint();
                    if(saveGIF){
                        repaint_gif();
                    }
                }
            }); 
            panel.add(cb_alloc);
            
            cb_model = new JComboBox();
            cb_model.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    int index = cb_model.getSelectedIndex();
                    model = buffer_model.get(index);
                    //System.out.println("chooise Model = "+model.name);
                    repaint();
                    if(saveGIF){
                        repaint_gif();
                    }
                }
            }); 
            panel.add(cb_model);
        }
    }

    
    @Override
    public void results(LinkerResults link) throws Exception {
        super.results(link); //To change body of generated methods, choose Tools | Templates.
        if(savePNG){
            savePNG();
        }
        if(saveGIF){
            saveGIF();
        }
    }
    private void savePNG() throws Exception{
        ALLOC curr_type = type;
        BlackmoreModel curr_model = model;
        for(ALLOC alloc : ALLOC.values()){
            type = alloc;
            int count = 1;
            for(BlackmoreModel m : buffer_model){
                this.type = alloc;
                this.model = m;
                String prefix = String.format(Locale.ENGLISH, "%s%dD_J%2dT%2dD%7.5f%s%s", 
                    alloc, approach.N(), inst.J(), approach.Waypoints(), approach.Delta(), 
                    inst.map_mame(), fBlackmoreOperator.obj.choises().isEmpty()? "" : fBlackmoreOperator.obj.choises().get(0));
                
                String suffix = String.format("%2d%s", count, m.name);

                prefix = prefix.replaceAll(" ", "0");
                suffix = suffix.replaceAll(" ", "0");

                save(resolution, "./", prefix, suffix);    //2.5 the plot resolution = 2.5*width x 2.5*height
                count++;
            }
        }
        type = curr_type;
        model = curr_model;
        repaint();
    }
    private void saveGIF() throws Exception{
        String prefix = String.format(Locale.ENGLISH, "%s%dD_J%2dT%2dD%7.5f%s%s", 
            "Gif", approach.N(), inst.J(), approach.Waypoints(), approach.Delta(), 
            inst.map_mame(), fBlackmoreOperator.obj.choises().get(0));
        prefix = prefix.replaceAll(" ", "0");
                
        save_gif("./", prefix, null);
    }

    @Override
    protected void paintStatic(Graphics2D g2, Rectangle rect) throws Throwable {
//        g2.setColor(Color.BLUE);
//        g2.draw(rect);
        g2.setFont(new Font(Font.MONOSPACED, Font.BOLD, 14));
        g2.setColor(Color.RED); 
        if(model==null){
            g2.drawString("model = null", 70, 80);
        }else{
            g2.drawString(String.format("cplex = %s", model.cplex_status()), 65, 70);
            g2.drawString(String.format("model = %s", model.model_status()), 65, 90);
        }
    }

    @Override
    protected void paintDynamic(Graphics2DReal gr, double x, double y, double w, double h) throws Throwable {
        //gr.draw(approach.inst.rectangle(), Color.RED);
        
//        gr.setColor(Color.RED);
//        gr.drawOvalR(x+w/2, y+h/2, w/2, h/2);
//        
        final double size = (inst.rect.width+inst.rect.height)/2;
        
        //obstacles
        for(Obstacle obs : inst.obstacles){
            obs.paint(gr, size);
        }
        if(type == BlackmorePlot.ALLOC.ALL){
            for(Obstacle obs : inst.obstacles){
                obs.paint_sense(gr, 0.005*size, 0.02*size);
            }
        }
        approach.paint(gr, size);
        if(model!=null){
            model.paint(gr, size);
            if(type==BlackmorePlot.ALLOC.ALL && model.isExtract()){
                for (LinearState state : model.states) {
                    state.plot.drawTrueAllocation(gr, Color.GREEN, inst.obstacles);
                }
            }
        }
    }
    public ALLOC type() {
        return type;
    }
    
    public void addModel(BlackmoreModel model) {
        if(!buffer_model.contains(model)){
            buffer_model.addLast(model);
            if(isPlot()){
                cb_model.addItem(model.name);
                cb_model.setSelectedIndex(buffer_model.size()-1);
            }else{
                this.model = model;
                if(saveGIF){
                    repaint_gif();
                }
            }
        }else{
            buffer_model.set(buffer_model.indexOf(model), model);
            if(isPlot()){
                cb_model.setSelectedIndex(buffer_model.indexOf(model));
            }else{
                this.model = model;
                //repaint();
                if(saveGIF){
                    repaint_gif();
                }
            }
        }
    }
}
