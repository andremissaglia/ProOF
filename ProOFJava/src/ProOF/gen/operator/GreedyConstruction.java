/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.gen.operator;

import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.com.Linker.LinkerValidations;
import ProOF.com.language.Approach;
import ProOF.opt.abst.problem.meta.Problem;
import ProOF.opt.abst.problem.meta.Solution;

/**
 *
 * @author marcio
 */
public final class GreedyConstruction extends Approach{
    public final static GreedyConstruction obj = new GreedyConstruction();
    
    private Problem prob;
    private oGreedyConstruction inits[];
    
    private GreedyConstruction() {
        
    }
    @Override
    public String name() {
        return "GreedyConstruction";
    }
    @Override
    public String description() {
        throw new UnsupportedOperationException("Not supported yet.");
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        
    }
    @Override
    public void services(LinkerApproaches link) throws Exception {
        prob = link.need(Problem.class, prob);
        inits = link.needs(oGreedyConstruction.class, new oGreedyConstruction[1]);
    }
    @Override
    public boolean validation(LinkerValidations link) throws Exception {
        throw new UnsupportedOperationException("Not supported yet.");
    }
    @Override
    public void load() throws Exception {
        
    }
    @Override
    public void start() throws Exception {
        
    }
    @Override
    public void results(LinkerResults link) throws Exception {
        
    }
    
    public void initialize(Solution ind, final double alpha) throws Exception {
        int index = prob.rnd.nextInt(inits.length);
        inits[index].initialize(prob, ind.codif(), alpha);
    }
}
