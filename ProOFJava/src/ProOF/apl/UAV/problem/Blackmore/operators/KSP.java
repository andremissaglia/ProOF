/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.apl.UAV.problem.Blackmore.Graph.Graph2;
import ProOF.apl.UAV.problem.Blackmore.Graph.YenKSP;
import ProOF.apl.UAV.problem.Blackmore.GraphVertex;
import ProOF.gen.operator.oInitialization;
import ProOF.utilities.uRouletteList;

/**
 *
 * @author andre
 */
public class KSP extends oInitialization<BlackmoreProblem, BlackmoreCodification> {
    private static YenKSP ksp;
    
    public static YenKSP getKsp(BlackmoreProblem prob){
        if(ksp == null){
            Graph2 g = new Graph2(prob.map.graph.vertexes.length);
            for(GraphVertex from : prob.map.graph.vertexes){
                from.adj.forEach((to) -> {
                    g.addEdge(from.id, to.id, from.costTo(to));
                });
            }
            ksp = new YenKSP(g, prob.map.graph.source().id, prob.map.graph.target().id);
        }
        return ksp;
    }
    
    @Override
    public String name() {
        return "KSP";
    }

    @Override
    public void initialize(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        ind.path.clear();
        ind.path.addAll(getKsp(prob).next().path);
    }
}
