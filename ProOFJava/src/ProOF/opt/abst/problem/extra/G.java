/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.opt.abst.problem.extra;

import ProOF.com.Linker.LinkerParameters;
import ProOF.opt.abst.problem.meta.Objective;
import ProOF.opt.abst.problem.meta.Problem;
import ProOF.opt.abst.problem.meta.Solution;


/**
 *
 * @author marcio
 */
public class G extends Fitness{
    private int memory_size;
    private Objective memory[];
    @Override
    public String name() {
        return "F(codif)+G(obj)";
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); //To change body of generated methods, choose Tools | Templates.
        memory_size = link.Int("memory size", 1000, 10, 1000000);
    }

    @Override
    public void start() throws Exception {
        super.start(); //To change body of generated methods, choose Tools | Templates.
        memory = new Objective[memory_size];
    }
    
    private int n=0;
    @Override
    public void evaluate(Problem prob, Solution sol) {
        if(n<memory_size){
            memory[n] = sol.obj();
            //sol.obj().add(1e6);
            n++;
        }
    }
}
