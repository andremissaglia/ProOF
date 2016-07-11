/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.map;

import ProOF.CplexExtended.Hyperplane;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.utilities.uIO;
import java.awt.Color;
import java.awt.Font;
import java.awt.geom.Point2D;
import java.util.Locale;
import java.util.Scanner;

/**
 *
 * @author marcio
 */
public class Obstacle3DHalf extends Obstacle{
    
    private final Point2D points[];
    private final Color color;
    private final String name;
    private final Font font;
    public final boolean is2D;
    
    public Obstacle3DHalf(double height, Color color, String name, Font font, Scanner sc) {
        this(height, color, name, font, sc, true);
    }
    public Obstacle3DHalf(double height, Color color, String name, Font font, Scanner sc, boolean ignore_frist_line) {
        this.color = color;
        this.name = name;
        this.font = font;
        this.is2D = false;
        if(ignore_frist_line)sc.nextLine();
        double x[] = uIO.toVectorDouble(sc.nextLine());
        double y[] = uIO.toVectorDouble(sc.nextLine());
        this.points = new Point2D[x.length];
        for(int i=0; i<x.length; i++){
            points[i] = new Point2D.Double(x[i], y[i]);
        }
        
        this.hyperplans = hyperplansFrom(points, new Hyperplane(height, 0, 0, 1));
        this.rect = rectangleFrom(points);
    }
    public Obstacle3DHalf(Color color, String name, Font font, Scanner sc) {
        this(color, name, font, sc, true);
    }
    public Obstacle3DHalf(Color color, String name, Font font, Scanner sc, boolean ignore_frist_line) {
        this.color = color;
        this.name = name;
        this.font = font;
        this.is2D = true;
        if(ignore_frist_line)sc.nextLine();
        double x[] = uIO.toVectorDouble(sc.nextLine());
        double y[] = uIO.toVectorDouble(sc.nextLine());
        this.points = new Point2D[x.length];
        for(int i=0; i<x.length; i++){
            points[i] = new Point2D.Double(x[i], y[i]);
        }
        
        this.hyperplans = hyperplansFrom(points);
        this.rect = rectangleFrom(points);
    }
    public Obstacle3DHalf(double height_max, double dist_ref, double cx_ref, double cy_ref, Font font, Scanner sc) {
        this.font = font;
        this.is2D = false;
        sc.nextLine();
        double x[] = uIO.toVectorDouble(sc.nextLine());
        double y[] = uIO.toVectorDouble(sc.nextLine());
        this.points = new Point2D[x.length];
        for(int i=0; i<x.length; i++){
            points[i] = new Point2D.Double(x[i], y[i]);
        }
        
        
        double cx = centerX();
        double cy = centerY();
        double dist_center = Math.sqrt((cx-cx_ref)*(cx-cx_ref) + (cy-cy_ref)*(cy-cy_ref));
        double height = height_max*(dist_ref/(dist_center+dist_ref));
        int rgb = (int) (255 - (height*(200))/height_max);
        this.color = new Color(rgb,rgb,rgb); 
        this.name = String.format(Locale.ENGLISH, "%1.1f", height);
        
        Hyperplane top = new Hyperplane(height_max, 0, 0, 1);
        this.hyperplans = hyperplansFrom(points, top);
        this.rect = rectangleFrom(points);
        //if(PRINT)System.out.printf(Locale.ENGLISH, "%8.4f * x + %8.4f * y  + %8.4f * z  = %8.4f\n", 0.0, 0.0, 1.0, height_max);
    }
    public Obstacle3DHalf(double height, String name, Font font) {
        this.name = name;
        this.font = font;
        this.color = null;
        this.is2D = false;
        this.points = null;
        this.rect = null;
        hyperplans = new Hyperplane[]{
            new Hyperplane(height, 0, 0, 1)
        };
        if(PRINT) System.out.println("---------------------- ground -------------------------");
        if(PRINT) System.out.printf(Locale.ENGLISH, "%8.4f * x + %8.4f * y  + %8.4f * z  = %8.4f\n", 0.0, 0.0, 1.0, height);
    }
    @Override
    public Point2D center() {
        double mx = averageX(points);
        double my = averageY(points);
        return new Point2D.Double(mx, my);
    }
    private double centerX(){
        double sum = 0;
        for(Point2D p : points){
            sum += p.getX();
        }
        return sum/points.length;
    }
    private double centerY(){
        double sum = 0;
        for(Point2D p : points){
            sum += p.getY();
        }
        return sum/points.length;
    }
    @Override
    public void paint(Graphics2DReal gr, double size) {
        if(points!=null){   // not paint the ground
            gr.paintPolygon(points, name, font, color, Color.BLACK);
        }
    }
    @Override
    public void paint(Graphics2DReal gr, Color color, double size) {
        gr.paintPolygon(points, name, size, color, Color.BLACK);
    }
    @Override
    public void paint_sense(Graphics2DReal gr, double r, double f) {
        if(points!=null){   // not paint the ground
            gr.paintNormal(points, hyperplans, r, f, Color.BLUE);
        }
    }
    
    
    private static Hyperplane[] hyperplansFrom(Point2D points[], Hyperplane ...add){
        Hyperplane[] hyperplans = new Hyperplane[points.length+add.length];
        for(int i=0; i<points.length; i++){
            int k = (i+1) % points.length;
            double x1 = points[i].getX();
            double y1 = points[i].getY();
            double x2 = points[k].getX();
            double y2 = points[k].getY();

            //x'
            double ax = -( y2 - y1 );
            //y'
            double ay = +( x2 - x1 );
            
            
            //normalize
            double norm = Math.sqrt(ax*ax+ay*ay);
            ax /= norm;
            ay /= norm;

            double b = ax * x1  + ay * y1;

            
            hyperplans[i] = new Hyperplane(b, ax, ay, 0.0);
            
            //if(PRINT)System.out.printf(Locale.ENGLISH, "%8.4f * x + %8.4f * y  = %8.4f\n", ax, ay, b);
        }
        System.arraycopy(add, 0, hyperplans, points.length, add.length);
        return hyperplans;
    }
}
