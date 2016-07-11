/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.HCCQSP;

import ProOF.CplexExtended.Hyperplane;
import ProOF.CplexOpt.CplexFull;
import ProOF.apl.UAV.HCCQSP.var.fControl;
import ProOF.apl.UAV.HCCQSP.var.fLiteral;
import ProOF.apl.UAV.HCCQSP.var.fState;
import ProOF.apl.UAV.HCCQSP.var.oControl;
import ProOF.apl.UAV.HCCQSP.var.oLiteral;
import ProOF.apl.UAV.HCCQSP.var.oState;
import ProOF.com.Linker.LinkerApproaches;
import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;
import java.util.Locale;

/**
 *
 * @author marcio
 */
public class HCCQSPMount extends CplexFull{
    private IloNumVar x[][];    //x(t):=[px, py, vx, vy]
    private IloNumVar u[][];    //u(t):=[ax, ay]
    private IloNumVar l[][];    //l(t):=[water, fire1, fire2,...]
    
    private Map M;

    
    private final double time = 60.0;
    private final int T = 25;
    private final double dt = time/T;
    private final double u_max = 1.0;
    private final double v_max = 1.0;
    
    //--------------------------------------- Initial Conditions -------------------------------------------
    private final double x0[] = new double[]{0.7, 2.8, 0, 0};      //x(t):=[px, py, vx, vy]
    private final boolean l0[] = new boolean[]{false, true, true};      //l(t):=[water, fire1, fire2]
    
    
    //--------------------------------------- Actions ------------------------------------------------------
    private final int N = 9; //actions steps
    private final int H = 5; //number of actions)
    private IloNumVar Hnh[][];  //action enable
    private IloNumVar Dnh[][];  //action duration
    private IloNumVar Tn[];     //step time
    private IloNumVar Zht[][];  //action scheduling
    private IloNumVar ZhtS[][]; //action scheduling start
    private IloNumVar ZhtE[][]; //action scheduling end
    
                                            // fly      fill    drop
    private final double d_lb[] = new double[]{0.01,    4, 4,   2, 2};
    private final double d_ub[] = new double[]{time,    8, 8,   5, 5};
    
    //---------------------------------------- Plan ---------------------------------------------------------
    //CCQSP
    private final int E = 3;    //number of events
    private final int S = 3;    //number of constraints
    private IloNumVar Te[];     //event time
    private IloNumVar Ke[];     //duration between events    
    private IloNumVar Yst[][];  //constraints scheduling
    
    
    public HCCQSPMount() throws IloException {}

    @Override
    public String name() {
        return "HCCQSP-mount";
    }

    private oState states[];
    private oControl controls[];
    private oLiteral literals[];
    
    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        link.add(fState.obj);
        link.add(fControl.obj);
        link.add(fLiteral.obj);
        
        states = link.needs(oState.class, new oState[0]);
        controls = link.needs(oControl.class, new oControl[0]);
        literals = link.needs(oLiteral.class, new oLiteral[0]);
    }
    
    
    @Override
    public void model() throws Exception {
        
        
        M = new Map();
        
        x = cpx.numVarArray(T+1, 4, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "x");
        u = cpx.numVarArray(T, 2, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "u");        
        l = cpx.boolVarArray(T+1, 1+M.set("forest").length, "l");
        
        Hnh = cpx.boolVarArray(N, H, "H");
        Dnh = cpx.intVarArray(N, H, 0, T, "D");
        //TODO modificar no PDF, tempo máximo Tn<=time
        Tn = cpx.numVarArray(N+1, 0, time, "Tn");
        
        
        Zht = cpx.boolVarArray(H, T+1, "Z");
        ZhtS = cpx.numVarArray(H, T+1, 0, 1, "ZS");
        ZhtE = cpx.numVarArray(H, T+1, 0, 1, "ZE");
        
        
        Te = cpx.numVarArray(E, 0, time, "Te");
        Ke = cpx.intVarArray(E-1, 0, T, "Ke");
        Yst = cpx.boolVarArray(S, T+1, "Y");
        //========================================= Teste ============================================
        //                              fly,    fill    fly     drop    fly     fill    fly     drop    fly
        int A[]     = new int[]     {   0,      1,      0,      3,      0,      2,      0,      4,      0};
        double D[]  = new double[]  {   5,      2,      6,      1,      5,      2,      2,      1,      6};
        for(int n=0; n<N; n++){
//            Hnh[n][A[n]].setLB(1);
//            Dnh[n][A[n]].setLB(D[n]);
//            Dnh[n][A[n]].setUB(D[n]);
        }
        
        
        //Tn[N].setUB(42);    //mission time
        //l[T][1].setUB(0);       //fire1 = false
        //l[T][2].setUB(0);       //fire2 = false
//        for(int i=0; i<x0.length; i++){ //ends at base
//            x[T][i].setLB(x0[i]);
//            x[T][i].setUB(x0[i]);
//        }
        
        //================================== Melhorias Preprocessadas===============================================
        Hnh[0][0].setLB(1); //fly
        cpx.addGe(cpx.sum(Hnh[1][1],Hnh[1][2]), 1);
        Hnh[2][0].setLB(1); //fly
        cpx.addGe(cpx.sum(Hnh[3][3],Hnh[3][4]), 1);
        Hnh[4][0].setLB(1); //fly
        cpx.addGe(cpx.sum(Hnh[5][1],Hnh[5][2]), 1);
        Hnh[6][0].setLB(1); //fly
        cpx.addGe(cpx.sum(Hnh[7][3],Hnh[7][4]), 1);
        Hnh[8][0].setLB(1); //fly
        
        //-------------------------- objective function ---------------------------------------
        //(4.59)
        IloNumExpr obj = null;
        for(int t=0; t<T; t++){
            obj = cpx.SumNumScalProd(obj, "sqr", 32, u_max, u[t]);
            //obj = cpx.SumProd(obj, 10, l[t][1],l[t][2]);
            //obj = cpx.SumProd(obj, -10, l[t][0]);
        }
        cpx.addMinimize(obj);
        
        
        //============================== START CONDITIONS ==================================================
        //-------------------------- states ---------------------------------------
        //(4.60)
        for(int i=0; i<x0.length; i++){
            x[0][i].setLB(x0[i]);
            x[0][i].setUB(x0[i]);
        }
        //-------------------------- literals ---------------------------------------
        //(4.61)
        for(int i=0; i<l0.length; i++){
            l[0][i].setLB(l0[i]?1:0);
            l[0][i].setUB(l0[i]?1:0);
        }
        
        //============================== ACTIONS ===========================================================
        //-------------------------- no máximo uma ação é executada por vez ---------------------------------------
        //TODO modificar no PDF
        //(4.65.1)
        for(int n=0; n<N; n++){
            IloNumExpr sum = null;
            for(int h=0; h<H; h++){
                sum = cpx.SumProd(sum, 1, Hnh[n][h]);
            }
            cpx.addLe(sum, 1.0, "4.65.1");
        }
        //-------------------------- Se nenhuma ação for executada então o fluxo de ações termina ---------------------------------------
        //TODO modificar no PDF
        //(4.65.2)
        for(int n=0; n<N-1; n++){
            IloNumExpr sum1 = null;
            for(int h=0; h<H; h++){
                sum1 = cpx.SumProd(sum1, 1, Hnh[n][h]);
            }
            IloNumExpr sum2 = null;
            for(int h=0; h<H; h++){
                sum2 = cpx.SumProd(sum2, 1, Hnh[n+1][h]);
            }
            cpx.addLe(sum2, sum1, "4.65.2");
        }
        //-------------------------- A variável Dnh \in Z+ é utilizada para calcular a duração da ação h no passo n ---------------------------------------
        //(4.66)
        for(int n=0; n<N; n++){
            for(int h=0; h<H; h++){
                cpx.addGe(cpx.prod(Dnh[n][h], dt), cpx.prod(Hnh[n][h], d_lb[h]), "4.66Ge");
                cpx.addLe(cpx.prod(Dnh[n][h], dt), cpx.prod(Hnh[n][h], d_ub[h]), "4.66Le");
            }
        }
        //-------------------------- melhorar bounds da variável Dnh -------------------
        //TODO modificar no PDF
        //(4.66.2) 
        for(int n=0; n<N; n++){
            for(int h=0; h<H; h++){
                Dnh[n][h].setUB((int)(d_ub[h]/dt+1e-6));
            }
        }
        //-------------------------- O tempo a cada passo n é dado por Tn>=0 -------------------
        //(4.67) 
        for(int n=0; n<N; n++){
            IloNumExpr sum = Tn[n];
            for(int h=0; h<H; h++){
                sum = cpx.SumProd(sum, dt, Dnh[n][h]);
            }
            cpx.addEq(Tn[n+1], sum, "4.67");
        }
        //-------------------------- O tempo no primeiro passo por definição é 0 --------------
        //TODO modificar no PDF
        //(4.67.1) 
        Tn[0].setUB(0);
        
        
        
        
        //-------------------------- O scheduling Zht \in {0, 1} das ações h executadas a cada instante de tempo t é dado por: ----------------
        //(4.68)
        for(int n=0; n<N; n++){
            for(int t=0; t<T+1; t++){
                IloNumVar y = cpx.IF_BetweenLT_Them_Y("(4.68.Betwenn)", Tn[n], t*dt, Tn[n+1]);
                for(int h=0; h<H; h++){
                    IloNumVar and = cpx.And("(4.68.AND)", y, Hnh[n][h]);
                    //cpx.addEq(and, Zht[h][t]);
                    cpx.addIF_Y_Them_Zi("(4.68.IF.TRUE)", and, Zht[h][t]);
                    //cpx.addIF_Y_Them_Zi("(4.68.IF.NOT)", cpx.Not(and), cpx.Not(Zht[h][t]));
                }
            }
        }
        //(4.69)
        for(int h=0; h<H; h++){
            for(int t=0; t<T+1; t++){
                cpx.addIF_Lt_Them_Y("(4.69)", cpx.Not(Zht[h][t]), cpx.sum(Tn[N], -t*dt));
            }
        }
            
        //-------------------------- No máximo uma ação h pode ser executada a cada instante t: -----------------------------
        //(4.70)
        for(int t=0; t<T+1; t++){
            IloNumExpr sum = null;
            for(int h=0; h<H; h++){
                sum = cpx.SumProd(sum, 1, Zht[h][t]);
            }
            cpx.addLe(sum, 1, "(4.70)");
            
            //Se nenhuma ação é executada manter os estados x(t+1) = x(t)
            //TODO modificar no PDF
            for(int i=0; t<T && i<x0.length; i++){
                cpx.addIF_Y_Them_Eq("(4.77).p", cpx.Not(sum), cpx.sum(x[t+1][i], cpx.prod(-1, x[t][i]))); 
            }
        }
        
        
        //Condições iniciais (start ou S) para a ação h ser executada são definidas, onde 1>= ZhtS >= 0 é verdade no instante em que a ação h é iniciada.
        //TODO modificar no PDF
        //(4.71)
        for(int h=0; h<H; h++){
            for(int t=0; t<T+1; t++){
                if(t==0){
                    cpx.addEq(ZhtS[h][t], Zht[h][t], "(4.71)");
                }else{
                    cpx.addAnd("(4.71)", ZhtS[h][t], Zht[h][t], cpx.Not(Zht[h][t-1]));
                }
//                if(t==0){
//                    cpx.addGe(ZhtS[h][t], Zht[h][t], "(4.71)");
//                }else{
//                    cpx.addGe(ZhtS[h][t], cpx.sum(Zht[h][t],cpx.prod(-1, Zht[h][t-1])), "(4.71)");
//                }
            }
        }
        //(4.72)
        for(int h=0; h<H; h++){
            for(int t=0; t<T+1; t++){
                if(h==1 || h==2){   //fill
                    //CondS: not water              water = l(t,0)
                    cpx.addIF_Y_Them_Zi("(4.72)", ZhtS[h][t], cpx.Not(l[t][0]));
                }else if(h==3 || h==4){ //drop
                    //CondS: water and fire            fire = l(t,1 or 2)
                    cpx.addIF_Y_Them_Zi("(4.72)", ZhtS[h][t], l[t][0], l[t][h-2]);
                }
            }
        }
        
        //Condições finais (end ou E) para a ação h ser executada são definidas, onde 1>= ZhtE >= 0 é verdade no instante em que a ação h é terminada.
        //TODO modificar no PDF
        //(4.73)
        for(int h=0; h<H; h++){
            for(int t=0; t<T+1; t++){
                if(t==0){
                    ZhtE[h][t].setUB(0);
                    //cpx.addEq(ZhtE[h][t], 0, "(4.73)");
                }else{
                    cpx.addAnd("(4.73)", ZhtE[h][t], Zht[h][t-1], cpx.Not(Zht[h][t]));
                }
            }
        }
        
        //(4.74)
        for(int h=0; h<H; h++){
            for(int t=0; t<T+1; t++){
                //dont have end conditions on this mission  Equation (4.74)
            }
        }
        
        
        //Condições de permanencia (remain ou R) para a ação h ocorrer, onde 1>= Zht >= 0  é verdade durante todo instante t em que a ação h é executada.
        //(4.76) 
        for(int h=0; h<H; h++){
            for(int t=0; t<T+1; t++){
                if(h==1 || h==2){   //fill
                    //CondR: x(t) \in r | r is some lake
                    Hyperplane hyperplanes[] = M.set("lake")[h-1].hyperplans;
                    cpx.addIF_Y_Them_StayIn("(4.76)", Zht[h][t], hyperplanes, new double[hyperplanes.length], x[t]);
                    
                    //CondR: v(t) = 0, the UAV must be stop
                    //cpx.addIF_Y_Them_Eq("(4.76).stop", Zht[h][t], x[t][2], x[t][3]);
                }else if(h==3 || h==4){ //drop
                    //CondR: x(t) \in r | r is some forest
                    Hyperplane hyperplanes[] = M.set("forest")[h-3].hyperplans;
                    cpx.addIF_Y_Them_StayIn("(4.76)", Zht[h][t], hyperplanes, new double[hyperplanes.length], x[t]);
                    
                    //CondR: v(t) = 0, the UAV must be stop
                    //cpx.addIF_Y_Them_Eq("(4.76).stop", Zht[h][t], x[t][2], x[t][3]);
                }
            }
        }
        
        //--------------------A dinâmica (Dyn) da ação h é aplicada a todo instante t em que a ação é executada ------------------------------------------
        
//        for(int t=0; t<T+1; t++){
//            x[t][2].setLB(-v_max);
//            x[t][2].setUB(+v_max);
//            x[t][3].setLB(-v_max);
//            x[t][3].setUB(+v_max);
//            if(t<T){
//                u[t][0].setLB(-u_max);
//                u[t][0].setUB(+u_max);
//                u[t][1].setLB(-u_max);
//                u[t][1].setUB(+u_max);
//            }
//        }
//        for(int t=0; t<T; t++){
//            IloNumExpr expr[] = new IloNumExpr[4];
//            expr[0] = cpx.sum(cpx.prod(-1, x[t+1][0]), x[t][0], cpx.prod(dt, x[t][2]),  cpx.prod(dt*dt/2, u[t][0]));
//            expr[1] = cpx.sum(cpx.prod(-1, x[t+1][1]), x[t][1], cpx.prod(dt, x[t][3]),  cpx.prod(dt*dt/2, u[t][1]));
//            expr[2] = cpx.sum(cpx.prod(-1, x[t+1][2]),          x[t][2],                cpx.prod(dt, u[t][0]));
//            expr[3] = cpx.sum(cpx.prod(-1, x[t+1][3]),          x[t][3],                cpx.prod(dt, u[t][1]));
//            for(int i=0; i<4; i++){
//                cpx.addEq(expr[i], 0, "Dyn."+i);
//            }
//        }
        

        //(4.77) --> (4.57) and (4.58)
        for(int h=0; h<H; h++){
            for(int t=0; t<T; t++){
                if(h==0){   
                    //Dynamic to FLY
                    //Dyn:  Zht --> x(t+1) = A x(t) + B u(t)        (4.57)
                    //      Zht --> |v(t)|<=v_max                   (4.58).v_max
                    //      Zht --> |u(t)|<=u_max                   (4.58).u_max
                    
                    IloNumExpr expr[] = new IloNumExpr[4];
                    expr[0] = cpx.sum(cpx.prod(-1, x[t+1][0]), x[t][0], cpx.prod(dt, x[t][2]),  cpx.prod(dt*dt/2, u[t][0]));
                    expr[1] = cpx.sum(cpx.prod(-1, x[t+1][1]), x[t][1], cpx.prod(dt, x[t][3]),  cpx.prod(dt*dt/2, u[t][1]));
                    expr[2] = cpx.sum(cpx.prod(-1, x[t+1][2]),          x[t][2],                cpx.prod(dt, u[t][0]));
                    expr[3] = cpx.sum(cpx.prod(-1, x[t+1][3]),          x[t][3],                cpx.prod(dt, u[t][1]));
                    cpx.addIF_Y_Them_Eq("(4.57)", Zht[h][t], expr);
                    
                    
                    cpx.addIF_Y_Them_Le("(4.58).v_max", Zht[h][t], cpx.sum(x[t][2], -v_max), cpx.sum(x[t][3], -v_max));
                    cpx.addIF_Y_Them_Ge("(4.58).v_min", Zht[h][t], cpx.sum(x[t][2], +v_max), cpx.sum(x[t][3], +v_max));
                    
                    cpx.addIF_Y_Them_Le("(4.58).u_max", Zht[h][t], cpx.sum(u[t][0], -u_max), cpx.sum(u[t][1], -u_max));
                    cpx.addIF_Y_Them_Ge("(4.58).u_min", Zht[h][t], cpx.sum(u[t][0], +u_max), cpx.sum(u[t][1], +u_max));
                }else{
                    //Dynamic to STAY
                    //Dyn:  Zht --> p(t+1) = p(t)                   (4.77).p
                    //      Zht --> |v(t)| = 0                      (4.77).v
                    //      Zht --> |v(t+1)| = 0                    (4.77).v
                    //      Zht --> |u(t)| = 0                      (4.77).u
                    cpx.addIF_Y_Them_Eq("(4.77).p", Zht[h][t],
                            cpx.sum(x[t+1][0], cpx.prod(-1, x[t][0])), 
                            cpx.sum(x[t+1][1], cpx.prod(-1, x[t][1])));
                    cpx.addIF_Y_Them_Eq("(4.77).v", Zht[h][t], x[t][2], x[t][3]);
                    cpx.addIF_Y_Them_Eq("(4.77).v", Zht[h][t], x[t+1][2], x[t+1][3]);
                    cpx.addIF_Y_Them_Eq("(4.77).u", Zht[h][t], u[t][0], u[t][1]);
                }
            }
        }
        


        
        
        //--------Efeitos (Eff) discretos finais (end ou E) da ação h são estabelecidos, onde l(t) é estado do literal a cada instante de tempo t.
        for(int t=0; t<T+1; t++){
            //(4.80)
            IloNumVar ZE[] = new IloNumVar[H];
            for(int h=0; h<H; h++){
                ZE[h] = ZhtE[h][t];
            }
            
            IloNumExpr orZE = cpx.Or("Or(ZE)", ZE);
            for(int i=0; i<l0.length; i++){
                if(t>0){
                    //manter literais
                    cpx.addIF_Y_Them_Z_Eq_W("(4.79)", cpx.Not(orZE), l[t][i], l[t-1][i]);
                }
            }

            for(int h=0; h<H; h++){
                if(h==0){   //fly
                    for(int i=0; i<l0.length; i++){
                        if(t>0){
                            //manter literais
                            cpx.addIF_Y_Them_Z_Eq_W("(4.79)", ZhtE[h][t], l[t][i], l[t-1][i]);
                        }
                    }
                }else if(h==1 || h==2){//fill
                    //water = true
                    cpx.addIF_Y_Them_Z_Eq_W("(4.78)", ZhtE[h][t], l[t][0], true);
                    if(t>0){
                        //fire(1) mantem
                        cpx.addIF_Y_Them_Z_Eq_W("(4.79)", ZhtE[h][t], l[t][1], l[t-1][1]);
                        //fire(2) mantem
                        cpx.addIF_Y_Them_Z_Eq_W("(4.79)", ZhtE[h][t], l[t][2], l[t-1][2]);
                    }     
                }else if(h==3){//drop(forest1)
                    //water = false
                    cpx.addIF_Y_Them_Z_Eq_W("(4.78)", ZhtE[h][t], l[t][0], false);
                    //fire(1) = false
                    cpx.addIF_Y_Them_Z_Eq_W("(4.78)", ZhtE[h][t], l[t][1], false);
                    
                    if(t>0){
                        //fire(2) mantem
                        cpx.addIF_Y_Them_Z_Eq_W("(4.79)", ZhtE[h][t], l[t][2], l[t-1][2]);
                    }
                }else if(h==4){//drop(forest2) 
                    //water = false
                    cpx.addIF_Y_Them_Z_Eq_W("(4.78)", ZhtE[h][t], l[t][0], false);
                    //fire(2) = false
                    cpx.addIF_Y_Them_Z_Eq_W("(4.78)", ZhtE[h][t], l[t][2], false);
                    
                    if(t>0){
                        //fire(1) mantem
                        cpx.addIF_Y_Them_Z_Eq_W("(4.79)", ZhtE[h][t], l[t][1], l[t-1][1]);
                    }
                }else{
                    throw new Exception("this error can not be");
                }
            }
        }
        
        
        //============================== PLAN ===========================================================
        //-------------------------- O tempo em que o evento (e) ocorre Te>=0 -------------------
        //
        Te[0].setUB(0);
        //(4.86)
        for(int e=0; e<E-1; e++){
            cpx.addEq(Te[e+1], cpx.sum(Te[e], cpx.prod(dt, Ke[e])), "4.86");
        }
        
        //-------------------------- As restrições de tempo dos episódios precisam ser obedecidas --------------------
        //e0 -> e1 \in [20, 50]
        //e1 -> e2 \in [0, +inf]
        //e0 -> e3 \in [0, 60]
        //(4.87)
        cpx.addRange(20, cpx.minus(Te[1], Te[0]), 50, "(4.87).extinguish");
        cpx.addRange(0, cpx.minus(Te[2], Te[0]), 60, "(4.87).safe");
        
        //(4.88)    start constraints
        
        //(4.89)    end constraints
        for(int t=0; t<T+1; t++){
            cpx.addIF_Eq_Them_Y("(4.89)", Yst[0][t], cpx.sum(Te[1], -t*dt));    //not fire1 and not fire2
            cpx.addIF_Eq_Them_Y("(4.89)", Yst[1][t], cpx.sum(Te[2], -t*dt));    //go base
        }
        
        //TODO modificar no PDF
        //start and end episodes needs to be true only one time
        for(int s=0; s<2; s++){
            IloNumExpr sum = null;
            for(int t=0; t<T+1; t++){
                sum = cpx.SumProd(sum, 1, Yst[s][t]);
            }
            cpx.addEq(sum, 1, "(4.89).one_time");
        }
        
        //(4.90)    remain constraints
        for(int t=0; t<T+1; t++){
            cpx.addIF_Between_Them_Y("(4.90)", Yst[2][t], Te[0], t*dt, Te[2]);
        }
        
        
        //(4.92)
        for(int t=0; t<T+1; t++){
            cpx.addIF_Y_Them_Z_Eq_W("(4.92)", Yst[0][t], l[t][1], false);   //Yst -> fire1 = false
            cpx.addIF_Y_Them_Z_Eq_W("(4.92)", Yst[0][t], l[t][2], false);   //Yst -> fire2 = false
        }
        
        
        //(4.91)
        for(int t=0; t<T+1; t++){   //stay in
            cpx.addIF_Y_Them_StayIn("(4.91)", Yst[1][t], M.set("base")[0].hyperplans, null, x[t]);    //go base
        }

        for(Map.Region r : M.set("NFZ")){
            cpx.addIF_Y_Them_StayOut("(4.91)", Yst[2], r.hyperplans, null, x);    
        }
        
//        for(int t=0; t<T+1; t++){   //stay out
//            for(Map.Region r : M.set("NFZ")){
//                cpx.addIF_Y_Them_StayOut("(4.91)", Yst[2][t], r.hyperplans, null, x[t]);    
//            }
//        }
    }

    @Override
    public void print() throws Exception {
        super.print(); //To change body of generated methods, choose Tools | Templates.
        
        double v_x[][] = cpx.getValues(x, true);
        cpx.print("states", "%15g", v_x);
        
        double v_u[][] = cpx.getValues(u, true);
        cpx.print("controls", "%15g", v_u);
        
        double v_l[][] = cpx.getValues(l, true);
        cpx.print("literals", "%15g", v_l);
        
        double v_Hnh[][] = cpx.getValues(Hnh, true);
        cpx.print("actions", "%15g", v_Hnh, "%15s", "fly", "fill(lake1)", "fill(lake2)", "drop(forest1)", "drop(forest2)");
        
        double v_Dnh[][] = cpx.getValues(Dnh, true);
        for(int n=0; n<N; n++){
            for(int h=0; h<H; h++){
                v_Dnh[n][h] = v_Dnh[n][h]*dt;
            }
        }
        cpx.print("duration", "%15g", v_Dnh, "%15s", "fly", "fill(lake1)", "fill(lake2)", "drop(forest1)", "drop(forest2)");
        for(int h=0; h<H; h++){
            System.out.printf("[%6.2f|%6.2f] ", d_lb[h], d_ub[h]);
        }
        System.out.println();
        
        double v_Tn[] = cpx.getValues(Tn, true);
        cpx.print("Tn", "%15g", v_Tn);
        
        String times[] = new String[T+1];
        for(int t=0; t<T+1; t++){
            times[t] = "t="+(int)(t*dt);
        }
        double v_Zht[][] = cpx.getValues(Zht, true);
        cpx.print("Zht", "%15g", v_Zht, "%15s", times);
        double v_ZhtS[][] = cpx.getValues(ZhtS, true);
        cpx.print("v_ZhtS", "%15g", v_ZhtS);
        double v_ZhtE[][] = cpx.getValues(ZhtE, true);
        cpx.print("v_ZhtE", "%15g", v_ZhtE);
        
        
        System.out.printf("%15s %15s %15s %15s %15s %15s %15s %15s %15s %15s %15s\n", "time", "px", "py", "vx", "vy", "ax", "ay", "action", "water", "fire1", "fire2");
        for(int t=0; t<T+1; t++){
            String actions[] = new String[]{"fly", "fill(lake1)", "fill(lake2)", "drop(forest1)", "drop(forest2)", "empty"};
            int action = H;
            double max = 0.01;
            for(int h=0; h<H; h++){
                if(v_Zht[h][t]>max){
                    max = v_Zht[h][t];
                    action = h;
                }
            }
            System.out.printf("%15s %15g %15g %15g %15g %15g %15g %15s %15g %15g %15g\n", 
                    "t = "+((int)(t*dt*100))/100, v_x[t][0], v_x[t][1], v_x[t][2], v_x[t][3], t<T?v_u[t][0]:0, t<T?v_u[t][1]:0, actions[action], v_l[t][0], v_l[t][1], v_l[t][2]);
        }
        
        double v_Te[] = cpx.getValues(Te, true);
        cpx.print("Te", "%15g", v_Te);
        
        double v_Yst[][] = cpx.getValues(Yst, true);
        cpx.print("Yst", "%15g", v_Yst, "%15s", times);
        
        System.out.println("Objective = "+ cpx.getObjValue());
        System.out.println("Status    = "+ cpx.getStatus());
    }

    
    
    public static void main(String[] args) throws IloException, Exception {
        Locale.setDefault(Locale.ENGLISH);
        HCCQSPMount hccqsp = new HCCQSPMount();
        hccqsp.model();
        hccqsp.cpx.setParam(IloCplex.DoubleParam.TiLim, 180);
        hccqsp.cpx.setParam(IloCplex.IntParam.Threads, 1);
        if(hccqsp.cpx.solve()){
            System.out.println("solved, status: "+hccqsp.cpx.getStatus());
            hccqsp.print();
        }else{
            System.out.println("not solved, status: "+hccqsp.cpx.getStatus());
        }
    }
    
}
