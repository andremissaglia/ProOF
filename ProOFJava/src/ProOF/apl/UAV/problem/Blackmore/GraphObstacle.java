/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore;

import Jama.Matrix;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearStateUncertainty;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.apl.UAV.map.Obstacle2D;
import ProOF.apl.UAV.map.Obstacle3DHalf;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

/**
 *
 * @author marcio
 */
public class GraphObstacle {
    protected final ArrayList<Point2D> vertexes = new ArrayList<Point2D>();
    protected final Path2D path;
    protected final Obstacle obs;
    public GraphObstacle(LinearSystem approach, pLinearStateUncertainty unc, Obstacle2D obstacle, double delta_to_cut) throws Exception {
        this.obs = obstacle;
        this.path = new Path2D.Double();
        for(int i = 0; i<obstacle.hyperplans.length; i++){
            int k = (i-1+obstacle.hyperplans.length)%obstacle.hyperplans.length;

            double fixed_risk_i = Double.MAX_VALUE;
            double fixed_risk_k = Double.MAX_VALUE;
            for(int t=0; t<approach.Waypoints()+1; t++){
                fixed_risk_i = Math.min(fixed_risk_i, unc.risk_allocation(t, delta_to_cut, obstacle.hyperplans[i].a));
                fixed_risk_k = Math.min(fixed_risk_k, unc.risk_allocation(t, delta_to_cut, obstacle.hyperplans[k].a));
            }
            //c_(i,t) (δ_jt ) + b_i ≤ a_i^T*μ_t
            //c_(k,t) (δ_jt ) + b_k ≤ a_k^T*μ_t
            //C+B = A*x
            //x = A^(-1)*(C+B)
            Matrix A = new Matrix(new double[][]{
                {obstacle.hyperplans[i].a[0],obstacle.hyperplans[i].a[1]},
                {obstacle.hyperplans[k].a[0],obstacle.hyperplans[k].a[1]},
            });
            Matrix B = new Matrix(new double[][]{
                {obstacle.hyperplans[i].b},
                {obstacle.hyperplans[k].b}
            });
            Matrix C = new Matrix(new double[][]{
                {fixed_risk_i},
                {fixed_risk_k}
            });
            Matrix x = A.inverse().times(C.plus(B));
            Point2D p = new Point2D.Double(x.get(0, 0), x.get(1, 0));
            vertexes.add(p);
            if(i==0){
                path.moveTo(p.getX(), p.getY());
            }else{
                path.lineTo(p.getX(), p.getY());
            }
        }
        path.closePath();
    }
    public GraphObstacle(LinearSystem approach, pLinearStateUncertainty unc, Obstacle3DHalf obstacle, double delta_to_cut) throws Exception {
        this.obs = obstacle;
        this.path = new Path2D.Double();
        final int H = obstacle.hyperplans.length;
        for(int i = 0; i<H; i++){
            int k = (i-1+H)%H;

            double fixed_risk_i = Double.MAX_VALUE;
            double fixed_risk_k = Double.MAX_VALUE;
            for(int t=0; t<approach.Waypoints()+1; t++){
                fixed_risk_i = Math.min(fixed_risk_i, unc.risk_allocation(t, delta_to_cut, obstacle.hyperplans[i].a));
                fixed_risk_k = Math.min(fixed_risk_k, unc.risk_allocation(t, delta_to_cut, obstacle.hyperplans[k].a));
            }
            //c_(i,t) (δ_jt ) + b_i ≤ a_i^T*μ_t
            //c_(k,t) (δ_jt ) + b_k ≤ a_k^T*μ_t
            //C+B = A*x
            //x = A^(-1)*(C+B)
            Matrix A = new Matrix(new double[][]{
                {obstacle.hyperplans[i].a[0],obstacle.hyperplans[i].a[1]},
                {obstacle.hyperplans[k].a[0],obstacle.hyperplans[k].a[1]},
            });
            Matrix B = new Matrix(new double[][]{
                {obstacle.hyperplans[i].b},
                {obstacle.hyperplans[k].b}
            });
            Matrix C = new Matrix(new double[][]{
                {fixed_risk_i},
                {fixed_risk_k}
            });
            Matrix x = A.inverse().times(C.plus(B));
            Point2D p = new Point2D.Double(x.get(0, 0), x.get(1, 0));
            vertexes.add(p);
            if(i==0){
                path.moveTo(p.getX(), p.getY());
            }else{
                path.lineTo(p.getX(), p.getY());
            }
        }
        path.closePath();
    }
    public void paint(Graphics2DReal gr) throws Exception {
//            gr.setColor(Color.BLACK);
//            for(int i=0; i<obs.Gj(); i++){
//                int k = (i+1)%vertexes.size();
//                Point2D p1 = vertexes.get(i);
//                Point2D p2 = vertexes.get(k);
//                gr.drawStringR("%d", (p1.getX()+p2.getX())/2, (p1.getY()+p2.getY())/2, i);
//            }
        //gr.draw(path);
    }
}
