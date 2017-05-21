package ProOF.apl.UAV.problem.Blackmore;

import ProOF.com.Stream.StreamPrinter;
import ProOF.opt.abst.problem.meta.codification.Codification;
import java.util.ArrayList;
import java.util.LinkedList;

/**
 *
 * @author marcio
 */
public class BlackmoreCodification extends Codification<BlackmoreProblem, BlackmoreCodification> {
    public ArrayList<Integer> path = new ArrayList<Integer>();
    
    /** 
     * Pesos relacionados ao numero de waypoints em uma aresta.
     * Deve ter tamanho path.length() - 1.
     * Se for `null`, a atribuicao sera proporcional ao custo da aresta (para 
     * nao quebrar o codigo existente).
     */
    public ArrayList<Double> weights = null;
    
    @Override
    public void copy(BlackmoreProblem prob, BlackmoreCodification source) throws Exception {
        //System.arraycopy(source.path, 0, this.path, 0, this.path.size());
        path.clear();
        path.addAll(source.path);
    }
    @Override
    public BlackmoreCodification build(BlackmoreProblem prob) throws Exception {
        return new BlackmoreCodification();
    }

    @Override
    public void printer(BlackmoreProblem prob, StreamPrinter stream) throws Exception {
        super.printer(prob, stream); //To change body of generated methods, choose Tools | Templates.
        prob.map.best_path = new GraphPath(path, prob.map.graph);
        prob.plot.repaint();
    }

    public GraphPath graph_path(BlackmoreProblem prob) {
        return new GraphPath(path, prob.map.graph);
    }

    public void repair(BlackmoreProblem prob) {
        boolean repeat[] = new boolean[prob.map.graph.vertexes.length];
        repeat[0] = true;   //source
        repeat[1] = true;   //target
        LinkedList<Integer> remove = new LinkedList<Integer>();
        for(int v : path){
            if(repeat[v]){
                remove.addLast(v);
            }else{
                repeat[v] = true;
            }
        }
        path.removeAll(remove);
    }
}
