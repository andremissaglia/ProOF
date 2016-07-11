package ProOF.opt.abst.problem.meta;

import ProOF.com.Linker.LinkerResults;
import ProOF.com.Stream.StreamPrinter;
import ProOF.opt.abst.problem.meta.codification.Codification;
import java.io.Serializable;

/**
 *
 * @author Márcio
 * @param <Prob>
 * @param <Codif>
 * @param <Obj>
 */
public abstract class Objective<
        Prob extends Problem,
        Codif extends Codification,
        Obj extends Objective
> implements Serializable, Comparable<Obj>{
    /**
     * Initialize the objective.
     * @throws Exception 
     */
    public abstract void start() throws Exception;
    
    /**
     * Dedodificate the solution and evaluate it.
     * @param prob
     * @param codif
     * @throws Exception 
     */
    protected abstract void evaluate(Prob prob, Codif codif) throws Exception;
   
    /**
     * 
     * @param prob
     * @param source
     * @throws Exception 
     */
    public abstract void copy(Prob prob, Obj source) throws Exception;

    /**
     * 
     * @param prob
     * @return
     * @throws Exception 
     */
    public abstract Obj build(Prob prob) throws Exception;

    /**
     * Compare the intensit for this less than other in absolute terms
     * @param other
     * @return 
     */
    public abstract double compareToAbs(Obj other);
    
    /**
     * Compare the intensit for this less than other in relative terms
     * @param other
     * @return 
     */
    public abstract double compareToRel(Obj other);
    
    /**
     * Compare the deviation between the objectives.
     * @param other
     * @return 
     */
    public abstract double deviation(Obj other) throws Exception;

    /**
     * Compare the relative gap between the objectives
     * @param other
     * @return
     * @throws Exception 
     */
    public abstract double gap(Obj other) throws Exception;
    
    public abstract void printer(Prob prob, StreamPrinter stream, Codif codif) throws Exception;
    
    public abstract void results(Prob prob, LinkerResults link, Codif codif) throws Exception;
    
    @Override
    public abstract String toString();    
    /**
     * 
     * @param prob
     * @return
     * @throws Exception 
     */
    public final Obj clone(Prob prob) throws Exception{
        Obj obj = build(prob);
        obj.copy(prob, this);
        return obj;
    }
    
    /**
     * Return if this solution is penalized
     * @return 
     * @throws java.lang.Exception 
     */
    public boolean penalized() throws Exception{
        return false;
    }

    //public abstract void add(double extra);
}
