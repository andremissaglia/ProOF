/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.apl.UAV.problem.Blackmore.GraphVertex;
import ProOF.gen.operator.oInitialization;
import ProOF.utilities.uRouletteList;

/**
 *
 * @author andre
 */
public class RandomBestFirst extends oInitialization<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Random Best First";
    }

    @Override
    public void initialize(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        int source = prob.map.graph.source().id;
        int target = prob.map.graph.target().id;
        int W = prob.Waypoints();

        // List visited nodes
        boolean visited[] = new boolean[prob.map.graph.vertexes.length];
        java.util.Arrays.fill(visited, false);

        // First vertex is chosen
        ind.path.clear();
        ind.path.add(source);

        visited[source] = true;
        int current = source;

        uRouletteList roulette = new uRouletteList(prob.rnd);

        // Build step by step a path following pheromone trail
        while (current != target) {
            roulette.clear();
            boolean dead_end = true;
            boolean toTarget = false;
            for (GraphVertex v : prob.map.graph.vertexes[current].adj) {
                if (!visited[v.id]) {
                    if (v.id == target) {
                        toTarget = true;
                        break;
                    }
                    roulette.add(1.0 / v.costToTarget, v.id);
                    dead_end = false;
                }
            }

            if (dead_end || toTarget || ind.path.size() == W) {
                ind.path.add(target);
                break;
            }
            // Select a candidate by roulette
            current = roulette.roulette_wheel();
            ind.path.add(current);

            visited[current] = true;
        }
    }
}
