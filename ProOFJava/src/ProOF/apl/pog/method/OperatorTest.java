package ProOF.apl.pog.method;

import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.com.Linker.LinkerValidations;
import ProOF.com.language.Factory;
import ProOF.com.language.Run;
import ProOF.gen.operator.oCrossover;
import ProOF.gen.operator.oInitialization;
import ProOF.gen.operator.oMutation;
import ProOF.gen.stopping.CountIteration;
import ProOF.gen.stopping.Stop;
import ProOF.opt.abst.problem.meta.Problem;
import ProOF.opt.abst.problem.meta.Solution;
import ProOF.opt.abst.problem.meta.codification.Codification;
import ProOF.opt.abst.problem.meta.objective.SingleObjective;
import java.util.ArrayList;
import java.util.Random;

public class OperatorTest extends Run {

    private final CountIteration loop = CountIteration.obj;

    private Problem problem;
    private Stop stop;
    private oInitialization inits[];
    private oCrossover cross[];
    private oMutation muts[];

    private final Factory fStop;
    private final Factory fProblem;
    private double infeasibleThreshold;
    private InitOperator initOperators[];
    private CrossOperator crossOperators[];
    private MutOperator mutOperators[];
    private Random rnd;

    @Override
    public String name() {
        return "Operator Test";
    }

    @Override
    public String description() {
        return "Evaluates the quality of the operators";
    }

    public OperatorTest(Factory fStop, Factory fProblem) {
        this.fStop = fStop;
        this.fProblem = fProblem;
        rnd = new Random();
    }

    @Override
    public void execute() throws Exception {
        Solution sol1 = problem.build_sol();
        Solution sol2 = problem.build_sol();
        while (!stop.end()) {
            initialize(sol1);
            initialize(sol2);
            crossover(sol1, sol2);
            mutate(sol1);
        }
    }

    private void initialize(Solution sol) throws Exception {
        initOperators[rnd.nextInt(initOperators.length)].getFeasible(sol);
    }

    private void mutate(Solution sol) throws Exception {
        mutOperators[rnd.nextInt(mutOperators.length)].mutate(sol);
    }

    private void crossover(Solution sol1, Solution sol2) throws Exception {
        crossOperators[rnd.nextInt(crossOperators.length)].crossover(sol1, sol2);
    }

    @Override
    public void services(LinkerApproaches link) throws Exception {
        link.add(loop);
        stop = link.get(fStop, stop);
        problem = link.get(fProblem, problem);
        inits = link.needs(oInitialization.class, new oInitialization[1]);
        cross = link.needs(oCrossover.class, new oCrossover[1]);
        muts = link.needs(oMutation.class, new oMutation[1]);
    }

    @Override
    public void parameters(LinkerParameters link) throws Exception {
        infeasibleThreshold = link.Dbl("Infeasible Threshold", 1000);
    }

    @Override
    public void load() throws Exception {
        initOperators = new InitOperator[inits.length];
        for (int i = 0; i < inits.length; i++) {
            initOperators[i] = new InitOperator(inits[i]);
        }

        mutOperators = new MutOperator[muts.length];
        for (int i = 0; i < muts.length; i++) {
            mutOperators[i] = new MutOperator(muts[i]);
        }

        crossOperators = new CrossOperator[cross.length];
        for (int i = 0; i < cross.length; i++) {
            crossOperators[i] = new CrossOperator(cross[i]);
        }
    }

    @Override
    public void start() throws Exception {
    }

    @Override
    public boolean validation(LinkerValidations link) throws Exception {
        return true;
    }

    @Override
    public void results(LinkerResults link) throws Exception {
        System.out.println("------------------------ [ initializations ] ---------------");
        for (InitOperator init : initOperators) {
            init.results();
        }
        System.out.println("------------------------ [ mutations ] ---------------------");
        for (MutOperator mut : mutOperators) {
            mut.results();
        }
        System.out.println("------------------------ [ crossovers ] --------------------");
        for (CrossOperator c : crossOperators) {
            c.results();
        }
    }

    @Override
    public void finish() throws Exception {
    }

    private class InitOperator {

        public Statistic objectives;
        public oInitialization operator;
        public int infeasible = 0;

        public InitOperator(oInitialization operator) {
            this.operator = operator;
            objectives = new Statistic();
        }

        public boolean initialize(Solution sol) throws Exception {
            operator.initialize(problem, sol.codif());
            problem.evaluate(sol);
            double obj = ((SingleObjective) sol.obj()).abs_value();
            if (obj < infeasibleThreshold) {
                objectives.add(obj);
                return true;
            } else {
                infeasible++;
                return false;
            }
        }

        public void getFeasible(Solution sol) throws Exception {
            while (!initialize(sol)) {
            }
        }

        public void results() {
            System.out.println("[ "+operator.name() + " ]:");
            System.out.println(String.format("Avg: %f", objectives.avg()));
            System.out.println(String.format("Std: %f", objectives.std()));
            System.out.println(String.format(
                    "Infeasible: %.3f%%",
                    (100.0 * infeasible) / (objectives.N() + infeasible))
            );
            System.out.println();
            System.out.println();
        }
    }

    private class MutOperator {

        public Statistic objectives;
        public oMutation operator;
        public int infeasible = 0;
        public int better = 0;

        public MutOperator(oMutation operator) {
            this.operator = operator;
            objectives = new Statistic();
        }

        public boolean mutate(Solution sol) throws Exception {
            double obj1 = ((SingleObjective) sol.obj()).abs_value();
            operator.mutation(problem, sol.codif());
            problem.evaluate(sol);
            double obj2 = ((SingleObjective) sol.obj()).abs_value();
            if (obj2 < obj1) {
                better++;
            }
            if (obj2 < infeasibleThreshold) {
                objectives.add(obj1 - obj2);
                return true;
            } else {
                infeasible++;
                return false;
            }
        }

        public void results() {
            System.out.println("[ "+operator.name() + " ]:");
            System.out.println(String.format("Avg: %f", objectives.avg()));
            System.out.println(String.format("Std: %f", objectives.std()));
            System.out.println(String.format(
                    "Infeasible: %.3f%%",
                    (100.0 * infeasible) / (objectives.N() + infeasible))
            );
            System.out.println(String.format(
                    "Better: %.3f%%",
                    (100.0 * better) / (objectives.N() + infeasible))
            );
            System.out.println();
            System.out.println();
        }
    }

    private class CrossOperator {

        public Statistic objectives;
        public oCrossover operator;
        public int infeasible = 0;
        public int worse = 0;
        public int mean = 0;
        public int better = 0;

        public CrossOperator(oCrossover operator) {
            this.operator = operator;
            objectives = new Statistic();
        }

        public boolean crossover(Solution sol1, Solution sol2) throws Exception {
            double obj1 = ((SingleObjective) sol1.obj()).abs_value();
            double obj2 = ((SingleObjective) sol2.obj()).abs_value();
            if (obj2 < obj1) {
                double aux = obj1;
                obj1 = obj2;
                obj2 = aux;
            }
            Codification codif = operator.crossover(problem, sol1.codif(), sol2.codif());
            Solution child = problem.build_sol(codif);
            problem.evaluate(child);
            double obj3 = ((SingleObjective) child.obj()).abs_value();
            if (obj3 > obj2) {
                worse++;
            } else if (obj3 > obj1) {
                mean++;
            } else {
                better++;
            }
            if (obj3 < infeasibleThreshold) {
                objectives.add(obj3 - (obj1 + obj2) / 2);
                return true;
            } else {
                infeasible++;
                return false;
            }
        }

        public void results() {
            System.out.println("[ "+operator.name() + " ]:");
            System.out.println(String.format("Avg: %f", objectives.avg()));
            System.out.println(String.format("Std: %f", objectives.std()));
            System.out.println(String.format(
                    "Infeasible: %.3f%%",
                    (100.0 * infeasible) / (objectives.N() + infeasible))
            );
            System.out.println(String.format(
                    "Worse: %.3f%%",
                    (100.0 * worse) / (objectives.N() + infeasible))
            );
            System.out.println(String.format(
                    "Mean: %.3f%%",
                    (100.0 * mean) / (objectives.N() + infeasible))
            );
            System.out.println(String.format(
                    "Better: %.3f%%",
                    (100.0 * better) / (objectives.N() + infeasible))
            );
            System.out.println();
            System.out.println();
        }
    }

    private class Statistic {

        public Statistic() {
            this.values = new ArrayList<>();
        }

        public void add(double val) {
            values.add(val);
            _sum += val;
        }

        public int N() {
            return values.size();
        }

        public double avg() {
            return _sum / values.size();
        }

        public double var() {
            if (_var >= 0) {
                return _var;
            }
            double avg = avg();

            double sum = values.stream()
                    .map((value) -> (value - avg) * (value - avg))
                    .reduce(0.0, Double::sum);

            _var = sum / (N() - 1);

            return _var;
        }

        public double std() {
            if (_std >= 0) {
                return _std;
            }
            _std = Math.sqrt(var());
            return _std;
        }
        private double _var = -1;
        private double _std = -1;
        private double _sum = 0;
        private final ArrayList<Double> values;

    }
}
