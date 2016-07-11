/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.gen.operator;


import ProOF.opt.abst.problem.meta.Objective;
import ProOF.opt.abst.problem.meta.Problem;
import ProOF.opt.abst.problem.meta.Solution;
import ProOF.opt.abst.problem.meta.codification.Codification;
import ProOF.opt.abst.problem.meta.codification.Operator;

/**
 *
 * @author marcio
 */
public abstract class oTrailPheromone <
        Prob extends Problem, Codif extends Codification, Obj extends Objective
> extends Operator {
    public abstract void initialize(Prob prob) throws Exception;
    public abstract double build(Prob prob, Codif ant) throws Exception;
    public abstract void evaporate(Prob prob) throws Exception;
    public abstract void deposit(Prob prob, Codif ant, Obj obj, double weight) throws Exception;
    
    public final double build(Prob prob, Solution<Prob, Obj, Codif, Solution> ant) throws Exception{
        return build(prob, ant.codif());
    }
    public final void deposit(Prob prob, Solution<Prob, Obj, Codif, Solution> ant, double weight) throws Exception{
        deposit(prob, ant.codif(), ant.obj(), weight);
    }
}
