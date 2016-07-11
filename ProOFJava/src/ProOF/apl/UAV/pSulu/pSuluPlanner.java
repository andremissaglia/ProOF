/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.pSulu;

import Jama.Matrix;
import ProOF.CplexExtended.CplexExtended;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.abst.uncertainty.pStateUncertainty;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.apl.UAV.mission.Ono.OnoInstance;
import ProOF.apl.UAV.mission.Ono.OnoInstance.Episode;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.opt.abst.run.Exact;
import ProOF.utilities.uTime;
import ProOF.utilities.uTimeMilli;
import ProOF.utilities.uUtil;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloRange;
import java.awt.Color;
import java.util.ArrayList;
import java.util.LinkedList;
import org.jgrapht.EdgeFactory;
import org.jgrapht.alg.FloydWarshallShortestPaths;
import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.jgrapht.graph.DefaultWeightedEdge;

/**
 *
 * @author marcio
 */
public class pSuluPlanner extends Exact{
    protected pSuluInstance inst = new pSuluInstance();
    public final pSuluPlot plot = new pSuluPlot(this);
    
    protected double execTime;
    protected double dt;
    protected double max_control;
    protected double max_velocity;
    protected double std_position;
    protected double std_velocity;
    
    private final uTime elapsed_time = new uTimeMilli();
    private double total_time;

    private pSuluGraph start;
    private int n;
    private int m;
    private double A[][];
    private double B[][];
    private Matrix Sigma[];
    private Matrix K_Sigma[];
    @Override
    public String name() {
        return "pSulu Planner";
    }
    @Override
    public void services(LinkerApproaches link) throws Exception {
        link.add(inst);
        link.add(plot);
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        execTime = link.Dbl("Time", 3600.0, 1.0, 180000.0);
        dt = link.Dbl("dt", 10, 1e-3, 1e6);
        max_control = link.Dbl("max-control", 1.0, 1e-3, 1e6);
        max_velocity = link.Dbl("max-velocity", 3.0, 1e-3, 1e6);
        std_position = link.Dbl("std-position", 0.05, 0.0, 1e6);
        std_velocity = link.Dbl("std-velocity", 0.00, 0.0, 1e6);
    }

    @Override
    public void start() throws Exception {
        super.start(); //To change body of generated methods, choose Tools | Templates.
        for(OnoInstance.Episode a : inst.episodes){
            if(a.ra.I.size()>1){  // 
                throw new Exception("episode '"+a+"' has stay-in regions with more tham one non-convex definition,\n"
                        + "\tthis is not suported by pSulu Planner algorithm proposed by Ono et. al, 2013");
            }else if(a.ra.I.size()>0 && a.ra.I.getFirst().C.size()>1){
                throw new Exception("episode '"+a+"' has stay-in regions with more tham one convex definition,\n"
                        + "\tthis is not suported by pSulu Planner algorithm proposed by Ono et. al, 2013");
            }
        }
        if(inst.N()!=2){  // 
            throw new Exception("The pSulu Planner algorithm proposed by Ono et. al, 2013 can only solve 2D scennarios\n");
        }
        
        start = new pSuluGraph(inst.n_events, dt, inst.temporal_constraints);
        
        n = inst.N()*2;
        m = inst.N();
        A = new double[n][n];
        B = new double[n][m];
        for(int i=0; i<n; i++){
            A[i][i] = 1.0;
        }
        for(int i=0; i<m; i++){
            A[i][m+i] = dt;
        }
        for(int i=0; i<m; i++){
            B[i][i] = dt*dt/2;
            B[i+m][i] = dt;
        }
        
        
        //System.out.println("==============================[State]==============================");
        Matrix A = new Matrix(this.A);
        Matrix B = new Matrix(this.B);

        //Matrix Q = new Matrix(dynamic.n(), dynamic.n());
        Matrix Q = Matrix.identity(n, n);

        Matrix R = Matrix.identity(m, m);
        //Matrix R = new Matrix(dynamic.m(), dynamic.m());
        Matrix N = new Matrix(n, m);


        Matrix P = Matrix.identity(n, n);

        for(int i=0; i<100; i++){
//                System.out.println("-----------------[P]------------------");
//                P.print(8, 5);

            Matrix APA = A.transpose().times(P).times(A);

            Matrix BPB = B.transpose().times(P).times(B);
            Matrix inv = R.plus(BPB).inverse();
            Matrix APB = A.transpose().times(P).times(B);
            Matrix BPA = B.transpose().times(P).times(A);

            P = APA.minus(APB.times(inv).times(BPA)).plus(Q);
        }
//        System.out.println("-----------------[P]------------------");
//        P.print(8, 5);

        Matrix BPB = B.transpose().times(P).times(B);
        Matrix BPA = B.transpose().times(P).times(A);

        Matrix F = R.plus(BPB).inverse().times(BPA.plus(N.transpose()));
//        System.out.println("-----------------[F]------------------");
//        F.print(8, 5);

        Matrix K = F.uminus();//.times(0);

//        System.out.println("-----------------[A]------------------");
//        A.print(8, 5);
//
//        System.out.println("-----------------[B]------------------");
//        B.print(8, 5);
//
//        System.out.println("-----------------[K]------------------");
//        K.print(8, 5);

        Matrix ABK = A.plus(B.times(K));
//        System.out.println("-----------------[A+BK]------------------");
//        ABK.print(8, 5);

        Matrix SigmaX0 = new Matrix(n, n);
        for(int i=0; i<m; i++){
            SigmaX0.set(i, i, Math.pow(std_position,2));
        }
        for(int i=m; i<n; i++){
            SigmaX0.set(i, i, Math.pow(std_velocity,2));
        }
        Matrix SigmaWt = SigmaX0.copy();

        //Matrix sigma = new Matrix(approach.N()*2, approach.N()*2);]
        Sigma = new Matrix[start.T()+1];
        K_Sigma = new Matrix[start.T()];
        
        Sigma[0] = SigmaX0.copy();
//        System.out.println("-----------------[sigma("+0+")]------------------");
//        Sigma[0].print(8, 5);
        
        for(int t=0; t<start.T(); t++){
            //System.out.println("-----------------[K*sigma("+t+")]------------------");
            K_Sigma[t] = K.uminus().times(Sigma[t]);
            //K_Sigma[t].print(8, 5);
            
            //System.out.println("-----------------[sigma("+(t+1)+")]------------------");
            Sigma[t+1] = ABK.times(Sigma[t]).times(ABK.transpose()).plus(SigmaWt);
            //Sigma[t+1].print(8, 5);
        }
        
    }
    
    
    private double remaining_time(){
        return execTime-elapsed_time.time();
    }

    
    private class TemporalConstraints{
        private final Integer source;
        private final Integer target;
        private final DefaultWeightedEdge lb;
        private final DefaultWeightedEdge ub;
        public TemporalConstraints(Integer source, Integer targer, DefaultWeightedEdge lb, DefaultWeightedEdge ub) {
            this.source = source;
            this.target = targer;
            this.lb = lb;
            this.ub = ub;
        }
    }
    private class pSuluGraph extends DefaultDirectedWeightedGraph<Integer, DefaultWeightedEdge>{
        public final ArrayList<TemporalConstraints> temp_const = new ArrayList<TemporalConstraints>();
        public final FloydWarshallShortestPaths floyd;
        public final double[] fixes;
        public final int n_events;
        public final double dt;
        public pSuluGraph(int n_events, double dt, OnoInstance.TemporalConstraints... temporal_constraints) {
            super(new EdgeFactory<Integer, DefaultWeightedEdge>() {
                @Override
                public DefaultWeightedEdge createEdge(Integer sourceVertex, Integer targetVertex) {
                    return new DefaultWeightedEdge();
                }
            });
            this.dt = dt;
            this.n_events = n_events;
            this.fixes = new double[0];
            for(int i=0; i<n_events; i++){
                this.addVertex(i);
            }
            for(OnoInstance.TemporalConstraints tc : temporal_constraints){
                DefaultWeightedEdge ub = this.addEdge(tc.eS, tc.eE);
                DefaultWeightedEdge lb = this.addEdge(tc.eE, tc.eS);
                this.setEdgeWeight(ub, tc.ub);
                this.setEdgeWeight(lb, -tc.lb);
                this.temp_const.add(new TemporalConstraints(tc.eS, tc.eE, lb, ub));
            }
            floyd = new FloydWarshallShortestPaths<Integer, DefaultWeightedEdge>(this); 
        }
        public pSuluGraph(pSuluGraph graph, int t) {
            super(new EdgeFactory<Integer, DefaultWeightedEdge>() {
                @Override
                public DefaultWeightedEdge createEdge(Integer sourceVertex, Integer targetVertex) {
                    return new DefaultWeightedEdge();
                }
            });
            this.dt = graph.dt;
            this.n_events = graph.n_events;
            this.fixes = new double[graph.fixes.length+1];
            System.arraycopy(graph.fixes, 0, this.fixes, 0, graph.fixes.length);
            this.fixes[graph.fixes.length] = t*dt;
            for(int i=0; i<graph.vertexSet().size(); i++){
                this.addVertex(i);
            }
            for(TemporalConstraints temp : graph.temp_const){
                boolean find = false;
                int e=0;
                for(double value: fixes){
                    if(e==temp.source && e+1==temp.target){
                        DefaultWeightedEdge ub = this.addEdge(temp.source, temp.target);
                        DefaultWeightedEdge lb = this.addEdge(temp.target, temp.source);
                        this.setEdgeWeight(ub, value);
                        this.setEdgeWeight(lb, -value);
                        this.temp_const.add(new TemporalConstraints(temp.source, temp.target, lb, ub));
                        find = true;
                        break;
                    }
                    e++;
                }
                if(!find){
                    DefaultWeightedEdge ub = this.addEdge(temp.source, temp.target);
                    DefaultWeightedEdge lb = this.addEdge(temp.target, temp.source);
                    this.setEdgeWeight(ub, graph.floyd.shortestDistance(temp.source, temp.target));
                    this.setEdgeWeight(lb, graph.floyd.shortestDistance(temp.target, temp.source));
                    this.temp_const.add(new TemporalConstraints(temp.source, temp.target, lb, ub));
                }
            }
            floyd = new FloydWarshallShortestPaths<Integer, DefaultWeightedEdge>(this);
        }
        public void print(){
//            System.out.printf("vertex = [ ");
//            for (Integer v: this.vertexSet()){
//                System.out.printf("%d ", v);
//            }
//            System.out.println("]");
//
//            System.out.printf("edges = [ ");
//            for(DefaultWeightedEdge edge : this.edgeSet()){
//                System.out.printf("%s ", edge);
//            }
//            System.out.println("]");
//            System.out.println("---------------------------------------");
//            for (Integer v: this.vertexSet()){
//                System.out.printf("[ ");
//                for (DefaultWeightedEdge edge: this.edgesOf(v)){
//                    System.out.printf("%s ", edge);
//                }
//                System.out.println("]");
//            }
//            System.out.println("---------------------------------------");
//
//            for ( Object p: floyd.getShortestPaths(0)){
//                System.out.println(p);
//            }
//            System.out.println("---------------------------------------");
//            for(DefaultWeightedEdge e : this.edgeSet()){
//                System.out.printf("%d -> %d  = %g\n",this.getEdgeSource(e), this.getEdgeTarget(e), floyd.shortestDistance(this.getEdgeSource(e), this.getEdgeTarget(e)));
//            }
            System.out.println("---------------------------------------");
            for(TemporalConstraints temp : temp_const){
                System.out.printf("%d -> %d  = [%g : %g]\n", temp.source, temp.target, -floyd.shortestDistance(temp.target, temp.source), floyd.shortestDistance(temp.source, temp.target));
            }
            System.out.println("---------------------------------------");
        }
        public int partial_event(){
            return fixes.length;
        }
        public boolean isEnd(){
            return partial_event()+1==n_events;
        }
        public int partial_T(){
            return event_time(partial_event());
        }
        private int T() {
            return event_time(n_events-1);
        }
        private int event_time(int event) {
            return (int)(floyd.shortestDistance(0, event)/dt + 0.999999);
        }
        public double nextLB(){
            return (-floyd.shortestDistance(partial_event()+1, partial_event()));
        }
        public double nextUB(){
            return (+floyd.shortestDistance(partial_event(), partial_event()+1));
        }
        public int lb() {
            return (int)(nextLB()/dt + 0.999999);
        }
        public int ub(){
            return (int)(nextUB()/dt);
        }
        private boolean isInside(Episode a) {
            return a.type==Episode.START_IN && a.eS <= partial_event() ||
                   a.type==Episode.REMAIN_IN && a.eE <= partial_event() ||
                   a.type==Episode.END_IN && a.eE <= partial_event();
        }

        
    }

    
    @Override
    public void execute() throws Exception {//Algorithm 6 from Ono et. al 2013
        elapsed_time.start();
        
        //graph.print();
        
        LinkedList<pSuluGraph> queue = new LinkedList<pSuluGraph>();
        queue.add(start);
        double Incumbent = Double.POSITIVE_INFINITY;
        
        while(!queue.isEmpty()){
            System.out.println("=============================================================");
            pSuluGraph graph = queue.removeLast();
            graph.print();
            double lower_bound = obtainLowerBound(graph);
            if(lower_bound<Incumbent){
                if(graph.isEnd()){
                    //Incumbent = lower_bound;
                    System.out.println("end : "+graph.partial_event());
                }else{
                    //expand, Algorithm 7 from Ono et. al 2013
                    System.out.println("partial : "+graph.partial_event());
                    
                    int lb = graph.lb();
                    int ub = graph.ub();
                    for(int t=lb; t<=ub; t++){
                        pSuluGraph graph_new = new pSuluGraph(graph, t);
                        queue.addLast(graph_new);
                    }
                }
            }
        }   
        total_time = elapsed_time.time();
    }
    private double obtainLowerBound(pSuluGraph graph) throws IloException, Exception {//Algorithm 8 from Ono et. al 2013
        if(graph.isEnd() || has_episodes_with_non_convex_state_regions(graph)){
            return NIRA(graph);
        }else{
            return obtainLowreBoundFRR(graph);
        }
    }
    private boolean has_episodes_with_non_convex_state_regions(pSuluGraph graph) {
        for(OnoInstance.Episode a : inst.episodes){
            if(graph.isInside(a)){ //episode a is inside the parcial event
                if(!a.ra.O.isEmpty()){  //episode a has non-convex state region
                    return true;
                }
            }
        }
        return false;
    }
    private double NIRA(pSuluGraph graph) throws IloException {//Algorithm 1 from Ono et. al 2013
        System.out.println("NIRA()");
        //new SubProblem(graph);
        //double value = sub.solve_lower_bound();
        System.out.println("end NIRA(): "+0);
        return 0;
    }
    private double obtainLowreBoundFRR(pSuluGraph graph) throws IloException, Exception {//Algorithm 5 from Ono et. al 2013
        System.out.println("start obtainLowreBound-FRR()");
        if(graph.partial_T()<=0){
            System.out.println("end obtainLowreBound-FRR(): "+0);
            return 0;
        }else{
            SubProblem sub = new SubProblem(graph, false);//false,????
            double value = sub.solve_lower_bound();
            System.out.println("end obtainLowreBound-FRR(): "+value);
            return value;
        }
    }
    
    private class SubProblem{
        private static final int UAprox = 16;
        private static final int VAprox = 16;
        private static final int DeltaAprox = 12;
                
        private CplexExtended cpx;
        //private IloObjective O;
        //private LinkedList<IloConstraint> M;
        private LinkedList<IloRange> C;
        private LinkedList<IloRange> D;
        private LinkedList<IloRange> R;
        
        private IloNumVar u[][];
        private IloNumVar x[][];
        
        private IloNumExpr Dc[];
        private IloNumExpr Udelta;
        private IloNumExpr Urisk;
        
        private final int T;
        private final boolean isFRR;
        public SubProblem(pSuluGraph graph, boolean isFRR) throws IloException, Exception {
            this.T = graph.partial_T();
            this.isFRR = isFRR;
            System.out.println("create SubProblem with T = "+T);
            cpx = new CplexExtended();
            u = cpx.numVarArray(T, inst.N(), -max_control, +max_control, "u");
            x = cpx.numVarArray(T+1, inst.N()*2, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "x");
            Dc = new IloNumExpr[inst.chance_constraints.length]; 
            
            //------------------------------------[U : control uncertainty feasible region]-----------------------------------
            for (int t = 0; t < T; t++) {
                addControlRegion(t);
            }
            //------------------------------------[V : velocity deterministic feasible region]-----------------------------------
            for (int t = 0; t < T; t++) {
                for(int n=0; n<VAprox; n++){ //only vx,vy
                    IloNumExpr exp = null;
                    exp = cpx.SumProd(exp, Math.cos((2*Math.PI*n)/VAprox), x[t][2]);
                    exp = cpx.SumProd(exp, Math.sin((2*Math.PI*n)/VAprox), x[t][3]);
                    cpx.addLe(exp, max_velocity, "Vmax");
                }
            }
            
            //------------------------------------[ O : objective ]-------------------------------------
            IloNumExpr obj = null;
            //System.out.println("maxControl = "+approach.maxControl());
            for (int t = 0; t < T; t++) {
                obj = cpx.SumNumScalProd(obj, "u*u", 32, max_control, u[t]);
            }
            cpx.addMinimize(obj);
            
            //------------------------------------[ I : initial conditions ]-------------------------------------
            for(int i=0; i<inst.start_state.length; i++){
                x[0][i].setLB(inst.start_state[i]);
                x[0][i].setUB(inst.start_state[i]);
            }
            
            //------------------------------------[ M : plant model ]-------------------------------------
            for (int t = 0; t < T; t++) {
                for(int i=0; i<A.length; i++){
                    IloNumExpr exp = null;
                    for(int j=0; j<A[i].length; j++){
                        exp = cpx.SumProd(exp, A[i][j], x[t][j]);
                    }
                    for(int j=0; j<B[i].length; j++){
                        exp = cpx.SumProd(exp, B[i][j], u[t][j]);
                    }
                    cpx.addEq(x[t+1][i], exp, "DynamicLinear["+(i+1)+"]");
                }
            }
            
            //------------------------------------[ C : chance constraint ]-------------------------------------
            //---------------------- Pontos de passagem ------------------------
            for(Episode ep : inst.episodes){
                int c = ep.c;
                if(ep.ra.I.size()>0 && graph.isInside(ep)){
                    if(ep.type==Episode.START_IN){
                        Obstacle obs = inst.regions[ep.ra.I.getFirst().C.getFirst()];                    
                        int t = graph.event_time(ep.eS);
                        addWaypoint(obs, t, c);
                        System.out.printf("Start-In %s at %d\n", ep.name, t);
                    }else if(ep.type==Episode.END_IN){
                        Obstacle obs = inst.regions[ep.ra.I.getFirst().C.getFirst()];                    
                        int t = graph.event_time(ep.eE);
                        addWaypoint(obs, t, c);
                        System.out.printf("End-In %s at %d\n", ep.name, t);
                    }else if(ep.type==Episode.REMAIN_IN){
                        Obstacle obs = inst.regions[ep.ra.I.getFirst().C.getFirst()];    
                        int from = graph.event_time(ep.eS);
                        int to = graph.event_time(ep.eE);
                        for(int t=from; t<=to; t++){
                            addWaypoint(obs, t, c);
                        }
                        System.out.printf("Remain-In %s from %d to %d\n", ep.name, from, to);
                    }else{
                        throw new Exception("episode type invalid");
                    }
                }
            }

//            //--------------------------- Obstaculos ---------------------------
//            for(Episode ep : approach.inst.episodes){
//                int c = ep.c;
//                if(ep.ra.O.size()>0){
//                    for(NonConvex non_convex : ep.ra.O){
//                        for(int index: non_convex.C){
//                            Obstacle obs = approach.inst.regions[index];
//
//                            IloIntVar Zt[][] = new IloIntVar[approach.Waypoints()+1][];
//                            for(int t=0; t<approach.Waypoints()+1; t++){
//                                Zt[t] = cplex.boolVarArray(obs.Gj(), "Z");
//                                for(int i=0; i<obs.Gj(); i++){
//                                    obstacles.add(Zt[t][i]);
//                                }
//                                addObstacle(obs, t, c, Zt[t]);
//                            }
//
//                            for(int t=0; t<approach.Waypoints()+1; t++){
//                                avoid.N(approach, this, Yat[ep.index], Zt, t);
//                            }
//                        }
//                    }
//                }
//            }
            
            if(!isFRR){
                //------------------------------------[ R : risk allocation limits ]-------------------------------------
                for(int c = 0; c<inst.chance_constraints.length; c++){
                    cpx.addLe(cpx.Sum(Dc[c], Udelta), inst.chance_constraints[c], "Dc+dU <= Delta_c");
                }
            }
        }

        private double solve_lower_bound() throws IloException, Exception {
            if(cpx.solve()){
                double vX[][] = cpx.getValues(x);
                double vU[][] = cpx.getValues(u);
                System.out.println("--------------------[ X | U]------------------------");
                for(int t=0; t<vX.length; t++){
                    System.out.printf("[%2d] [ ", t);
                    for(int i=0; i<vX[t].length; i++){
                        System.out.printf("%12.8f ", vX[t][i]);
                    }
                    System.out.print("| ");
                    for(int i=0; t<vU.length && i<vU[t].length; i++){
                        System.out.printf("%12.8f ", vU[t][i]);
                    }
                    System.out.println("]");
                }
                System.out.println("-------------------------------------------------");
                return cpx.getNbinVars()>0 ? cpx.getBestObjValue() : cpx.getObjValue();
            }else{
                //throw new Exception("don't solve");
                return Double.POSITIVE_INFINITY;
            }
        }
        private double solve_uper_bound() throws IloException, Exception {
            if(cpx.solve()){
                return cpx.getObjValue();
            }
            throw new Exception("don't solve");
        }
        private void addControlRegion(int t) throws Exception {
            if(isFRR){
                addControlRegionFRR(t);
            }else{
                addControlRegionAlloc(t);
            }
        }
        private void addControlRegionFRR(int t) throws IloException, Exception{
            for(int n=0; n<UAprox; n++){ //only vx,vy
                double a[] = new double[]{Math.cos((2*Math.PI*n)/UAprox), Math.sin((2*Math.PI*n)/UAprox)};
                final double fixed_risk = pStateUncertainty.inv_erf(1-2*uUtil.minDbl(inst.chance_constraints))*Math.sqrt(2*uncertainty(Sigma[t], a));//approach.unc.risk_allocation(t, fixed_delta[c], obs.hyperplans[i].a);
                IloNumExpr exp = null;
                exp = cpx.SumProd(exp, a[0], u[t][0]);
                exp = cpx.SumProd(exp, a[1], u[t][1]);
                cpx.addLe(exp, max_control-fixed_risk, "Umax");
            }
            Udelta = null;
            Urisk = null;
        }
        private void addControlRegionAlloc(int t) throws IloException, Exception{
            final IloNumVar In[] = cpx.numVarArray(UAprox, 0, uUtil.minDbl(inst.chance_constraints), "U("+t+").Risk");
            final IloNumExpr c[] = new IloNumExpr[UAprox];
            for(int n=0; n<UAprox; n++){ //only ux,uy
                double a[] = new double[]{Math.cos((2*Math.PI*n)/UAprox), Math.sin((2*Math.PI*n)/UAprox)};
                double uncertainty = Math.sqrt(2*(uncertainty(K_Sigma[t], a)));
                c[n] = cpx.RiskAllocation(In[n], uncertainty, uUtil.minDbl(inst.chance_constraints), DeltaAprox, "U("+t+").Alloc");
            }
            for(int n=0; n<UAprox; n++){ //only vx,vy
                IloNumExpr exp = c[n];
                exp = cpx.SumProd(exp, Math.cos((2*Math.PI*n)/UAprox), u[t][0]);
                exp = cpx.SumProd(exp, Math.sin((2*Math.PI*n)/UAprox), u[t][1]);
                cpx.addLe(exp, max_control, "Umax");
            }
            Udelta = cpx.sum(In);
            Urisk = cpx.sum(c);
        }
        private double[] UncX(Obstacle obs, int t) throws Exception{
            double unc[] = new double[obs.Gj()];
            boolean isUnc = false;
            for(int i=0; i<obs.Gj(); i++){
                unc[i] = Math.sqrt(2*uncertainty(Sigma[t], obs.hyperplans[i].a));
                if(unc[i]>1e-6){
                    isUnc = true;
                }
            }
            return isUnc ? unc : null;
        }
        
        public void addWaypoint(Obstacle obs, int t, int c) throws Exception {
            if(isFRR){
                addWaypointFRR(obs, t, c);
            }else{
                addWaypointAlloc(obs, t, c);
            }
        }
        public void addWaypointFRR(Obstacle obs, int t, int c) throws Exception {
            for(int i=0; i<obs.Gj(); i++){
                //final double fixed_risk = approach.unc.risk_allocation(t, fixed_delta);
                final double fixed_risk = pStateUncertainty.inv_erf(1-2*inst.chance_constraints[c])*Math.sqrt(2*uncertainty(Sigma[t], obs.hyperplans[i].a));//approach.unc.risk_allocation(t, fixed_delta[c], obs.hyperplans[i].a);
                //a_i^T*x_t  ≤ b_i - c_(i,t) (δ_it)
                cpx.addLe(obs.hyperplans[i].scalProd(cpx, x[t]), obs.hyperplans[i].b-fixed_risk, "FrrIi");
            }
        }
        public void addWaypointAlloc(Obstacle obs, int t, int c) throws Exception {
            double unc[] = UncX(obs, t);
            if(unc!=null){
                for(int i=0; i<obs.Gj(); i++){
                    IloNumVar delta = cpx.numVar(0.0, Double.POSITIVE_INFINITY, "temp");
                    cpx.addWaypointUnc(obs.hyperplans[i], x[t], delta, unc[i], inst.chance_constraints[c], DeltaAprox);
                    Dc[c] = cpx.SumProd(Dc[c], 1.0, delta);
                }
                System.out.println("uncertainty state on waypoints");
            }else{
                for(int i=0; i<obs.Gj(); i++){
                    cpx.addWaypointDet(obs.hyperplans[i], x[t]);
                }
                System.out.println("deterministic state on  waypoints");
            }
        }
        public void addObstacle(Obstacle obs, int t, int c, IloIntVar Z[]) throws Exception {
            double unc[] = UncX(obs, t);
            if(unc!=null){
                IloNumVar delta = cpx.numVar(0.0, Double.POSITIVE_INFINITY, "temp");
                for(int i=0; i<obs.Gj(); i++){
                    cpx.addObstacleUnc(obs.hyperplans[i], x[t], Z[i], delta, unc[i], inst.chance_constraints[c], DeltaAprox);
                }
                Dc[c] = cpx.SumProd(Dc[c], 1.0, delta);
            }else{
                for(int i=0; i<obs.Gj(); i++){
                    cpx.addObstacleDet(obs.hyperplans[i], x[t], Z[i]);
                }
            }
        }

        
    }

    public static double uncertainty(Matrix Sigma, double a[]) throws Exception{
        double temp[] = new double[a.length];
        for(int i=0; i<a.length; i++){
            for(int k=0; k<a.length; k++){
                temp[i] += a[k] * Sigma.get(k, i);
            }
        }
        double sigma = 0;             // = a(j,t)^T * Sigma(t) * a(j,t)
        for(int k=0; k<a.length; k++){
            sigma += temp[k] * a[k];
        }
        return sigma;
    }
    @Override
    public void results(LinkerResults link) throws Exception {
        link.writeDbl("method-time", total_time);
    }
    public void paint(Graphics2DReal gr, double size) throws Exception {
        gr.paintOvalR(inst.start_state[0], inst.start_state[1], 0.01*size, 0.01*size, Color.ORANGE, Color.BLACK);
    }
    
    
}
