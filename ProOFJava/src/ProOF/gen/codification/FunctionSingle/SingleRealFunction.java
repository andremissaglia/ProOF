/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.gen.codification.FunctionSingle;

import ProOF.com.Linker.LinkerResults;
import ProOF.com.Results.Result;
import ProOF.com.Stream.StreamPrinter;
import ProOF.com.language.Factory;
import ProOF.gen.codification.Real.cReal;
import ProOF.opt.abst.problem.meta.codification.Codification;
import ProOF.opt.abst.problem.meta.objective.BoundDbl;
import ProOF.opt.abst.problem.meta.objective.SingleObjective;
import java.util.LinkedList;

/**
 *
 * @author marcio
 */
public class SingleRealFunction extends SingleFunction{
    
    public SingleRealFunction(Factory single, Factory operator) {
        super(single, operator);
    }
    
    @Override
    public String name() {
        return "Single - Real";
    }
    @Override
    public Codification build_codif() throws Exception {
        return new cReal(func.size());
    }
    @Override
    public SingleObjective build_obj() throws Exception {
        return new ObjReal(bound);
    }
    private class ObjReal extends SingleObjective<SingleRealFunction, cReal, ObjReal> {
        private LinkedList<Result> stream = new LinkedList<>();
        public ObjReal(BoundDbl bound) throws Exception {
            super(bound);
        }
        @Override
        public void evaluate(SingleRealFunction prob, cReal codif) throws Exception {
            stream.clear();
            set(func.evaluate(codif.X, stream));
        }
        @Override
        public void copy(SingleRealFunction prob, ObjReal source) throws Exception {
            super.copy(prob, source); //To change body of generated methods, choose Tools | Templates.
            this.stream.clear();
            this.stream.addAll(source.stream);
        }
        @Override
        public ObjReal build(SingleRealFunction prob) throws Exception {
            return new ObjReal(bound);
        }
        @Override
        public void printer(SingleRealFunction prob, StreamPrinter stream, cReal codif) throws Exception {
            super.printer(prob, stream, codif); //To change body of generated methods, choose Tools | Templates.
            for(Result r : this.stream){
                r.printer(stream);
            }
            func.printer(stream, codif.X);
        }
        @Override
        public void results(SingleRealFunction prob, LinkerResults link, cReal codif) throws Exception {
            super.results(prob, link, codif); //To change body of generated methods, choose Tools | Templates.
            for(Result r : this.stream){
                r.results(link);
            }
        }
    }
}

