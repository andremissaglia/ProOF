/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.pSulu;

import ProOF.apl.UAV.mission.Ono.*;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.map.Plot;
import ProOF.com.Linker.LinkerResults;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;
import java.util.Locale;
import javax.swing.JComboBox;
import ProOF.apl.UAV.mission.Ono.OnoInstance.Episode;
import ProOF.apl.UAV.mission.Ono.OnoPlot.ALLOC;
import static ProOF.apl.UAV.mission.Ono.OnoPlot.ALLOC.ALL;
import ProOF.com.Linker.LinkerParameters;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.geom.Rectangle2D;

/**
 *
 * @author marcio
 */
public class pSuluPlot extends Plot{
    private final pSuluPlanner approach;
//    private final int width = 1100;
//    private final int height = 1030;
    
    private OnoModel model = null;
    private final LinkedList<OnoModel> buffer_model = new LinkedList<OnoModel>();

    
    
     
    private ALLOC type = ALL;
    
    private JComboBox cb_model;
    private JComboBox cb_alloc;

    private Rectangle2D rect;
    
    public pSuluPlot(pSuluPlanner approach) {
        this.approach = approach;
    }
    @Override
    public String name() {
        return "Ono Plot";
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
        setTitle(name()+" : "+(approach.inst.N())+"D");
        Config(1900, 1030);
        rect = approach.inst.rectangle();
        goTo(rect, 0.15);
        
        //goMove(12, -12);   //choise the center
        //goZoom(0.6);            //spefic the zoom
//        
//        goMove(0, -10.30/2-1.25);   //choise the center
//        goZoom(0.6);            //spefic the zoom
//                
        
        if(isPlot()){
            cb_alloc = new JComboBox(ALLOC.values());
            cb_alloc.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    type = ALLOC.values()[cb_alloc.getSelectedIndex()];
                    //System.out.println("chooise Alloc = "+type.name());
                    repaint();
                    repaint_gif();
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
                    repaint_gif();
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
        OnoModel curr_model = model;
        for(ALLOC alloc : ALLOC.values()){
            type = alloc;
            int count = 1;
            for(OnoModel m : buffer_model){
                this.type = alloc;
                this.model = m;
                String prefix = String.format(Locale.ENGLISH, "%s%dD_e%2dE%2dJ%2d%s", 
                    alloc, approach.inst.N(), approach.inst.n_events, approach.inst.episodes.length, approach.inst.regions.length,
                    approach.inst.map_mame());
                
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
        String prefix = String.format(Locale.ENGLISH, "%s%dD_e%2dE%2dJ%2d%s", 
            "Gif", approach.inst.N(), approach.inst.n_events, approach.inst.episodes.length, approach.inst.regions.length,
            approach.inst.map_mame());
        
        prefix = prefix.replaceAll(" ", "0");
                
        save_gif("./", prefix, null);
    }
    
    @Override
    protected void paintStatic(Graphics2D g2, Rectangle rect) throws Throwable {
//        g2.setColor(Color.BLUE);
//        g2.draw(rect);
    }

    @Override
    protected void paintDynamic(Graphics2DReal gr, double x, double y, double w, double h) throws Throwable {
        //gr.draw(approach.inst.rectangle(), Color.RED);

        for(Episode ep: approach.inst.episodes){
            ep.paint(gr, (rect.getWidth()+rect.getHeight())/2, type());
        }
        //obstacles
//        for(Obstacle obs : approach.inst.obstacles){
//            obs.paint(gr);
//        }
//        if(approach.plot.type() == OnoPlot.ALLOC.ALL){
//            for(Obstacle obs : approach.inst.obstacles){
//                obs.paint_sense(gr, 0.05, 0.2);
//            }
//        }
        approach.paint(gr, (rect.getWidth()+rect.getHeight())/2);
        if(model!=null){
            model.paint(gr, (rect.getWidth()+rect.getHeight())/2);
//            if(type==OnoPlot.ALLOC.ALL){
//                for (LinearState state : model.states) {
//                    state.plot.drawTrueAllocation(gr, Color.GREEN, approach.inst.obstacles);
//                }
//            }
        }
    }
    public ALLOC type() {
        return type;
    }
    
    public void addModel(OnoModel model) {
        if(!buffer_model.contains(model)){
            buffer_model.addLast(model);
            if(isPlot()){
                cb_model.addItem(model.name);
                cb_model.setSelectedIndex(buffer_model.size()-1);
            }else{
                this.model = model;
                repaint_gif();
            }
        }else{
            if(isPlot()){
                cb_model.setSelectedIndex(buffer_model.indexOf(model));
            }else{
                this.model = model;
                repaint_gif();
            }
        }
    }
}
