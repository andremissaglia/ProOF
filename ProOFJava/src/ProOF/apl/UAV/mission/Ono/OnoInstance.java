/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.opt.abst.problem.Instance;
import java.awt.Color;
import java.awt.Font;
import java.awt.Rectangle;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Scanner;

/**
 *
 * @author marcio
 */
public abstract class OnoInstance extends Instance{
    
    /**
     * initial state (px py ... vx vy ...)
     */
    public double start_state[];
    /**
     * chance constraints (c0, c1 ...)
     */
    public double chance_constraints[];
    
    public int n_events;
    
    public TemporalConstraints temporal_constraints[];
    public Episode episodes[];
    public Obstacle[] regions;

    
    public abstract int N();
    public abstract String map_mame();
    
    
    private Font font = new Font(Font.SANS_SERIF, Font.BOLD, 12);
    
    public final Rectangle2D rectangle(){
        Rectangle2D rect = new Rectangle2D.Double((int)start_state[0], (int)start_state[1], 0, 0);
        for (Obstacle obstacle : regions) {
            if(obstacle!=null && obstacle.rect!=null){
                rect.add(obstacle.rect);
            }
        }
        font = new Font(Font.SANS_SERIF, Font.BOLD, (int)(rect.getWidth()+rect.getHeight()));
        //rect.grow((int)(rect.width*offset), (int)(rect.height*offset));
        return rect;
    }

    public void setFont(Rectangle2D rect) {
        font = new Font(Font.SANS_SERIF, Font.BOLD, (int)(rect.getWidth()+rect.getHeight()));
    }
    
    
    public LinkedList<TemporalConstraints> temporalConstraints(Scanner sc, int n_events, double time) throws Exception{
        LinkedList<TemporalConstraints> list = new LinkedList<TemporalConstraints>();
        String next = sc.next();
        if(next.equals("forall(e):")){
            String line = sc.nextLine();
            for(int e=0; e<n_events-1; e++){
                list.add(new TemporalConstraints(new Scanner(line), e, n_events, time));
            }
        }else if(next.equals("cmd:")){
            list.add(new TemporalConstraints(sc, 10000, n_events, time));
            sc.nextLine();
        }else{
            throw new Exception("command '"+next+"' unknow");
        }
        return list;
    }
    
    public class TemporalConstraints{
        public final int eS;
        public final int eE;
        public final double lb;
        public final double ub;

        public TemporalConstraints(int eS, int eE, double lb, double ub) {
            this.eS = eS;
            this.eE = eE;
            this.lb = lb;
            this.ub = ub;
        }
        public TemporalConstraints(Scanner sc){
            this.eS = sc.nextInt();
            this.eE = sc.nextInt();
            this.lb = sc.nextDouble();
            this.ub = sc.nextDouble();
            sc.nextLine();
        }
        public int event(Scanner sc, int e, int n_event){
            String next = sc.next();
            if(next.equals("[e]")){
                return e;
            }else if(next.equals("[e+1]")){
                return e+1;
            }else if(next.equals("[e-1]")){
                return e-1;
            }else if(next.equals("[eS]")){
                return 0;
            }else if(next.equals("[eE]")){
                return n_event-1;
            }else{
                return Integer.parseInt(next);
            }
        }
        public double time(Scanner sc, double time){
            String next = sc.next();
            if(next.equals("[time]")){
                return time;
            }else{
                return Double.parseDouble(next);
            }
        }
        public TemporalConstraints(Scanner sc, int e, int n_events, double time){
            this.eS = event(sc, e, n_events);
            this.eE = event(sc, e, n_events);
            this.lb = time(sc, time);
            this.ub = time(sc, time);
        }
        

        @Override
        public String toString() {
            return String.format("tc = from %d to %d with [%g , %g]", eS, eE, lb, ub);
        }
        
        
    }
    

    
    public class Ra{
        public final LinkedList<NonConvex> I = new LinkedList<NonConvex>();
        public final LinkedList<NonConvex> O = new LinkedList<NonConvex>();
        public Ra(Scanner sc) {
            int nI = sc.nextInt();
            int nO = sc.nextInt();
            for(int i=0; i<nI; i++){
                I.addLast(new NonConvex(sc));
            }
            for(int i=0; i<nO; i++){
                O.addLast(new NonConvex(sc));
            }
            sc.nextLine();
        }

        
        private int nI(Scanner sc, int n_way){
            String next = sc.next();
            if(next.equals("|I|")){
                return n_way;
            }else{
                return Integer.parseInt(next);
            }
        }
        private int nO(Scanner sc, int n_obs){
            String next = sc.next();
            if(next.equals("|O|")){
                return n_obs;
            }else{
                return Integer.parseInt(next);
            }
        }
        private Ra(Scanner sc, int e, int way[], int obs[]) throws Exception {
            int nI = nI(sc, way.length);
            int nO = nO(sc, obs.length);
            
            for(int i=0; i<nI; i++){
                I.addAll(nonConvexs(sc, e, way, obs));
            }
            for(int o=0; o<nO; o++){
                O.addAll(nonConvexs(sc, e, way, obs));
            }
        }
        @Override
        public String toString() {
            return String.format("%s %s", I, O);
        }

        private void paint(Graphics2DReal gr, String name, double size, OnoPlot.ALLOC type) {
            for(NonConvex i : I){
                i.paint(gr, Color.GREEN, name, size, type);
            }
            for(NonConvex o : O){
                o.paint(gr, Color.LIGHT_GRAY, name, size, type);
            }
        }
        
    }

    public LinkedList<NonConvex> nonConvexs(Scanner sc, int e, int way[], int obs[]) throws Exception{
        LinkedList<NonConvex> list = new LinkedList<NonConvex>();
        String next = sc.next();
        if(next.equals("forall(i):")){
            String line = sc.next()+" "+sc.next();
            for(int i=0; i<way.length; i++){
                list.add(new NonConvex(new Scanner(line), e, way[i]));
            }
        }else if(next.equals("forall(o):")){
            String line = sc.next()+" "+sc.next();
            for(int o=0; o<obs.length; o++){
                list.add(new NonConvex(new Scanner(line), o, obs[o]));
            }
        }else if(next.equals("cmd:")){
            list.add(new NonConvex(sc, e, 10000));
        }else{
            throw new Exception("command '"+next+"' unknow");
        }
        return list;
    }
        
    public class NonConvex{
        public final LinkedList<Integer> C = new LinkedList<Integer>();
        public NonConvex(Scanner sc, int e, int index) {
            int n = sc.nextInt();
            for(int i=0; i<n; i++){
                C.addLast(index(sc, e, index));
            }
        }
        public NonConvex(Scanner sc) {
            int n = sc.nextInt();
            for(int i=0; i<n; i++){
                C.addLast(sc.nextInt());
            }
        }
        private int index(Scanner sc, int e, int index){
            String next = sc.next();
            if(next.equals("[e]")){
                return e;
            }else if(next.equals("[o]")){
                return index;
            }else if(next.equals("[i]")){
                return index;
            }else{
                return Integer.parseInt(next);
            }
        }
        private void paint(Graphics2DReal gr, Color fill_color, String name, double size, OnoPlot.ALLOC type) {
            for(Integer i : C){
                gr.setColor(fill_color);
                regions[i].paint(gr, size);
                if(type == OnoPlot.ALLOC.ALL){
                    regions[i].paint_sense(gr, size*0.004, size*0.02);
                }
               
                if(name.equals("null")){
                    
                }else if(name.equals("index")){
                    gr.setColor(Color.BLACK);
                    Point2D center = regions[i].center();
                    gr.drawStringR(""+i, font, center.getX(), center.getY());
                }else if(name.contains(":index")){
                    gr.setColor(Color.BLACK);
                    Point2D center = regions[i].center();
                    gr.drawStringR(name.replaceAll("[:]index", "")+":"+i, font, center.getX(), center.getY());
                }else{
                    gr.setColor(Color.BLACK);
                    Point2D center = regions[i].center();
                    gr.drawStringR(name, font, center.getX(), center.getY());
                }
            }
        }

        @Override
        public String toString() {
            return C.toString();
        }

        
    }
    
    public LinkedList<Episode> episodes(Scanner sc, int n_events, int way[], int obs[]) throws Exception{
        LinkedList<Episode> list = new LinkedList<Episode>();
        String next = sc.next();
        if(next.equals("forall(e):")){
            String line = sc.nextLine();
            for(int e=0; e<n_events-1; e++){
                list.add(new Episode(new Scanner(line), e, n_events, way, obs));
            }
        }else if(next.equals("cmd:")){
            list.add(new Episode(sc, 10000, n_events, way, obs));
        }else{
            throw new Exception("command '"+next+"' unknow");
        }
        return list;
    }
    private int INDEX = 0;

    public class Episode{
        public static final int START_IN = 0;
        public static final int REMAIN_IN = 1;
        public static final int END_IN = 2;
        
        public Episode(Scanner sc, int e, int n_events, int way[], int obs[]) throws Exception{
            this.index = INDEX++;
            this.name = name(sc, e);
            this.type = type(sc);
            this.c = sc.nextInt();
            this.eS = event(sc, e, n_events);
            this.eE = event(sc, e, n_events);
            this.ra = new Ra(sc, e, way, obs);
        }
        public String name(Scanner sc, int e){
            String next = sc.next();
            if(next.equals("[e]")){
                return ""+(char)('A'+e);
            }else {
                return next;
            }
        }
        public int event(Scanner sc, int e, int n_event){
            String next = sc.next();
            if(next.equals("[e]")){
                return e;
            }else if(next.equals("[e+1]")){
                return e+1;
            }else if(next.equals("[e-1]")){
                return e-1;
            }else if(next.equals("[eS]")){
                return 0;
            }else if(next.equals("[eE]")){
                return (n_event-1);
            }else{
                return Integer.parseInt(next);
            }
        }
        
        public final int index;
        public final String name;
        public final int type;
        public final int c;
        public final int eS;
        public final int eE;
        public final Ra ra;

        public Episode(int index, Scanner sc) {
            this.index = index;
            this.name = sc.next();
            this.type = type(sc);
            this.c = sc.nextInt();
            this.eS = sc.nextInt();
            this.eE = sc.nextInt();
            this.ra = new Ra(sc);
        }
        
        @Override
        public String toString() {
            return String.format("%8s %8d %2d %2d %2d {%s}", name, type, c, eS, eE, ra);
        }
        
        private int type(Scanner sc){
            String s = sc.next();
            if(s.equals("start-in")){
                return START_IN;
            }else if(s.equals("remain-in")){
                return REMAIN_IN;
            }else if(s.equals("end-in")){
                return END_IN;
            }else{
                return -1;
            }
        }
        public void paint(Graphics2DReal gr, double size, OnoPlot.ALLOC type) {
            ra.paint(gr, name, size, type);
        }
    }
}
