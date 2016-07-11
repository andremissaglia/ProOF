/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.gen.codification.FunctionSingle;

import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.com.Results.Result;
import ProOF.com.Stream.StreamPrinter;
import ProOF.com.language.Factory;
import ProOF.gen.codification.BinaryReal.cBinaryReal;
import ProOF.opt.abst.problem.meta.codification.Codification;
import ProOF.opt.abst.problem.meta.objective.BoundDbl;
import ProOF.opt.abst.problem.meta.objective.SingleObjective;
import java.util.LinkedList;

/**
 *
 * @author marcio
 */
public class SingleBinRealFunction extends SingleFunction{
    private int length;
    public SingleBinRealFunction(Factory single, Factory operator) {
        super(single, operator);
    }
    @Override
    public String name() {
        return "Single - BinReal";
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link);
        length = link.Int("length", 16, 2, 1048576);
    }
    @Override
    public Codification build_codif() throws Exception {
        return new cBinaryReal(func.size(), length);
    }
    @Override
    public SingleObjective build_obj() throws Exception {
        return new ObjBinReal(bound);
    }
    private class ObjBinReal extends SingleObjective<SingleBinRealFunction, cBinaryReal, ObjBinReal> {
        private LinkedList<Result> stream = new LinkedList<>();
        public ObjBinReal(BoundDbl bound) throws Exception {
            super(bound);
        }
        @Override
        public void evaluate(SingleBinRealFunction prob, cBinaryReal codif) throws Exception {
            stream.clear();
            set(func.evaluate(codif.X(), stream));
        }
        @Override
        public void copy(SingleBinRealFunction prob, ObjBinReal source) throws Exception {
            super.copy(prob, source); //To change body of generated methods, choose Tools | Templates.
            this.stream.clear();
            this.stream.addAll(source.stream);
        }
        @Override
        public ObjBinReal build(SingleBinRealFunction prob) throws Exception {
            return new ObjBinReal(bound);
        }
        @Override
        public void printer(SingleBinRealFunction prob, StreamPrinter stream, cBinaryReal codif) throws Exception {
            super.printer(prob, stream, codif); //To change body of generated methods, choose Tools | Templates.
            for(Result r : this.stream){
                r.printer(stream);
            }
        }
        @Override
        public void results(SingleBinRealFunction prob, LinkerResults link, cBinaryReal codif) throws Exception {
            super.results(prob, link, codif); //To change body of generated methods, choose Tools | Templates.
            for(Result r : this.stream){
                r.results(link);
            }
        }
    }
}

