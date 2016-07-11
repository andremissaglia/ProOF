/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.HCCQSP;

import static ProOF.apl.UAV.HCCQSP.HCCQSP.VarType.*;
/**
 *
 * @author marcio
 */
public class HCCQSP {
    enum VarType{
        deterministic, stochastic
    };
    
    public static final VarBound unbounded = new VarBound(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        
    public static class VarBound{
        public final double lb;
        public final double ub;
        public VarBound(double lb, double ub) {
            this.lb = lb;
            this.ub = ub;
        }
    }
    
    public static class Variable{
        public final String name;
        public Variable(String name) {
            this.name = name;
        }
    }
    public static class Continuous extends Variable{
        public final VarType type;
        public final VarBound bound;
        public Continuous(String name, VarType type, VarBound bound) {
            super(name);
            this.type = type;
            this.bound = bound;
        }
    }
    public static class Literals extends Variable{
        public Literals(String name) {
            super(name);
        }
    }
    
    
//    public static class Action{
//        public final double lb;
//        public final double ub;
//        
//    }
    
    Continuous px = new Continuous("px", stochastic, unbounded);
    Continuous py = new Continuous("py", stochastic, unbounded);
    Continuous vx = new Continuous("vx", stochastic, new VarBound(-1,+1));
    Continuous vy = new Continuous("vy", stochastic, new VarBound(-1,+1));
    Continuous fuel = new Continuous("fuel", deterministic, new VarBound(0, 100));
    Literals water = new Literals("water");
    Literals fire1 = new Literals("fire1");
    Literals fire2 = new Literals("fire2");
    
    //p = get(position)
    //v = get(velocity)
    //fuel = get
    //x = need(state.class)
    
    //2D state := [px, py, vx, vy]
    //3D state := [px, py, pz, vx, vy, vz]
    //2D state-fuel := [px, py, vx, vy, fuel]
    //
}
