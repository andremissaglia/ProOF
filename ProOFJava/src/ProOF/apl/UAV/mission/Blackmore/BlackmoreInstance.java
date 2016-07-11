/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.apl.UAV.map.Obstacle;
import ProOF.apl.UAV.map.Point3D;
import ProOF.apl.UAV.map.PointGeo;
import ProOF.opt.abst.problem.Instance;
import java.awt.Rectangle;

/**
 *
 * @author marcio
 */
public abstract class BlackmoreInstance extends Instance{
    /**
     * initial state (px py ... vx vy ...)
     */
    public double start_state[];
    /**
     * end position (px py ... vx vy ...)
     */
    public double end_point[];
    
    
    public Obstacle obstacles[];
    
    public Rectangle rect;
    
    public PointGeo base;
    public Point3D Si;
    public Point3D Sf;
    public Point3D Ei;
    public Point3D Ef;
    
    public final int J(){
        return obstacles.length;
    }
    public final int Gj(int j){
        return obstacles[j].Gj();
    }
    public final double bigM() {
        return 2*(rect.width+rect.height);
    }
    public final double a(int j, int i, int n) {
        return obstacles[j].hyperplans[i].a[n];
    }
    public final double b(int j, int i) {
        
        return obstacles[j].hyperplans[i].b;
    }
    
    public abstract int N();
    public abstract String map_mame();
    
        
    protected final Rectangle rectangle(){
        Rectangle rect = new Rectangle();
        rect.add(start_state[0], start_state[1]);
        rect.add(end_point[0], end_point[1]);
        for (Obstacle obstacle : obstacles) {
            if(obstacle.rect!=null){
                rect.add(obstacle.rect);
            }
        }
        //rect.grow((int)(rect.width*offset), (int)(rect.height*offset));
        return rect;
    }
}
