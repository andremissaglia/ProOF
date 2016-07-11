/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV;

import ProOF.CplexExtended.CplexExtended;
import ProOF.apl.UAV.abst.UAVApproach;
import ProOF.apl.UAV.abst.UAVModel;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.opt.abst.run.Heuristic;
import ProOF.utilities.uTime;
import ProOF.utilities.uTimeMilli;
/**
 * Customized Solution Approach(CSA)
 * @author marcio
 */
public class CSA extends Heuristic{
    private UAVApproach approach;
    private oCSA csa;
    
    protected double execTime;
    protected double epGap;
    protected int threads;
    private boolean print_war;
    private boolean print_out;
    
    private UAVModel frr;
    private UAVModel frt;
    private UAVModel raa1;
    private UAVModel raa2;
    
    
    private double total_time;
    private String status = null;
    private double upper;
    private double lower;
    
    private final uTime elapsed_time = new uTimeMilli();
    
    
    @Override
    public String name() {
        return "AUV.CSA";
    }
    @Override
    public void services(LinkerApproaches link) throws Exception {
        approach = link.get(fUAVApproach.obj, approach);
        csa = link.need(oCSA.class, csa);
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        execTime = link.Dbl("Time", 3600.0, 1.0, 180000.0);
        epGap = link.Dbl("Gap Rel", 0.0001, 0.0, 100.0);
        threads = link.Int("Threads", 1, 0, 16);
        
        print_war = link.Bool("warning", false);
        print_out = link.Bool("output", true);
    }

    private double remaining_time(){
        return execTime-elapsed_time.time();
    }
    @Override
    public void execute() throws Exception {
        elapsed_time.start();
        upper = Integer.MAX_VALUE;
        lower = Integer.MIN_VALUE;
        status = "ok";
        
        frr = csa.build_FRR(approach, new CplexExtended());
        double time = Math.max((remaining_time()-5)/2, 5);
        frr.solve(time, epGap, threads, print_war, print_out);
        if (frr.isFeasible()) {
            lower = frr.lower();
            
            raa1 = csa.build_RAA(approach, new CplexExtended(), frr);
            time = Math.max((remaining_time()-5)/2, 5);
            raa1.solve(time, epGap, threads, print_war, print_out);
            if (raa1.isFeasible()) {
                upper = raa1.upper();
                //rote = raa1.rote();
            } else {
                frt = csa.build_FRT(approach, new CplexExtended());
                time = Math.max((remaining_time()-5), 5);
                frt.solve(time, epGap, threads, print_war, print_out);
                if (frt.isFeasible()) {
                    upper = frt.upper();
                    raa2 = csa.build_RAA(approach, new CplexExtended(), frt);
                    time = Math.max(remaining_time(), 5);
                    raa2.solve(time, epGap, threads, print_war, print_out);
                    if (raa2.isFeasible()) {
                        upper = raa2.upper();
                        //rote = raa2.rote();
                    } else {
                        status = "fail[3]";
                        System.err.println("Problem fail[3]");
                        upper = Integer.MAX_VALUE;
                    }
                } else {
                    upper = Integer.MAX_VALUE;
                    status = "fail[2]";
                    System.err.println("Problem fail[2]");
                }
            }
        } else {
            lower = Integer.MAX_VALUE;
            System.err.println("Problem fail[1]");
            status = "fail[1]";
        }
        
        total_time = elapsed_time.time();
    }
    @Override
    public void results(LinkerResults link) throws Exception {
        link.writeDbl("method-time", total_time);
        link.writeDbl("method-objective", upper);
        link.writeDbl("method-lower", lower);
        link.writeString("CSA-status", status);
        
        if (frr != null && frr.isFeasible()) {
            frr.results(link);
        }
        if (frt != null && frt.isFeasible()) {
            frt.results(link);
        }
        if (raa1 != null && raa1.isFeasible()) {
            raa1.results(link);
            raa1.save();
        }
        if (raa2 != null && raa2.isFeasible()) {
            raa2.results(link);
            raa2.save();
        }
    }
}
