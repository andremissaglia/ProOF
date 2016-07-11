package ProOF.opt.abst.problem.meta;

import ProOF.com.Stream.StreamPrinter;
import ProOF.opt.abst.problem.meta.codification.Codification;
import java.util.Objects;


/**
 *
 * @author Márcio
 * @param <Prob>
 * @param <Obj>
 * @param <Codif>
 * @param <Sol>
 */
public abstract class Solution<
        Prob extends Problem,
        Obj extends Objective, 
        Codif extends Codification,
        Sol extends Solution
    > implements Comparable<Sol> {

    protected final Obj objective;
    protected final Codif codification;
    
    public Solution(Obj objective, Codif codification) throws Exception {
        this.objective = objective;
        this.codification = codification;
        this.objective.start();
    }
    /**
     * 
     * @param prob
     * @return
     * @throws Exception 
     */
    public abstract Sol build(Prob prob) throws Exception;
    
    /**
     * 
     * @param prob
     * @param codification
     * @return
     * @throws Exception 
     */
    public abstract Sol build(Prob prob, Codif codification) throws Exception;

    protected final void evaluate(Prob prob) throws Exception{
        this.objective.evaluate(prob, this.codification);
    }
    public final boolean penalized() throws Exception{
        return this.objective.penalized();
    }
    public final double compareToAbs(Sol other){
        return this.objective.compareToAbs(other.objective);
    }
    public final double compareToRel(Sol other){
        return this.objective.compareToRel(other.objective);
    }
    public final double deviation(Sol other) throws Exception{
        return this.objective.deviation(other.objective);
    }
    public final double gap(Sol other) throws Exception{
        return this.objective.gap(other.objective);
    }
    
    public final Solution minimum(Sol other){
        int cp = compareTo(other);
        return cp <= 0 ? this : other;
    }
    public final Solution maximum(Sol other){
        int cp = compareTo(other);
        return cp >= 0 ? this : other;
    }
    public final boolean GT(Sol other){
        return compareTo(other) > 0;
    }
    public final boolean GE(Sol other){
        return compareTo(other) >= 0;
    }
    public final boolean EQ(Sol other){
        return compareTo(other) == 0;
    }
    public final boolean LE(Sol other){
        return compareTo(other) <= 0;
    }
    public final boolean LT(Sol other){
        return compareTo(other) < 0;
    }
    
    public final Codif codif(){
        return codification;
    }
    public final Obj obj(){
        return objective;
    }
    @Override
    public int compareTo(Sol other) {
        if(other==null){
            return -1;
        }
        return this.objective.compareTo(other.objective);
    }
    @Override
    public String toString() {
        return objective.toString();
    }
//
//    @Override
//    public boolean equals(Object obj) {
//        return super.equals(obj) || EQ((Sol) obj); //To change body of generated methods, choose Tools | Templates.
//    }
//
//    @Override
//    public int hashCode() {
//        int hash = 5;
//        hash = 53 * hash + Objects.hashCode(this.objective);
//        return hash;
//    }
    /**
     * 
     * @param prob
     * @param source
     * @throws Exception 
     */
    public void copy(Prob prob, Sol source) throws Exception {
        this.codification.copy(prob, source.codification);
        this.objective.copy(prob, source.objective);
    }
    /**
     * 
     * @param prob
     * @return
     * @throws Exception 
     */
    public final Sol clone(Prob prob) throws Exception{
        Sol sol = Solution.this.build(prob);
        sol.copy(prob, this);
        return sol;
    }

    public boolean copyIfBetter(Prob prob, Sol source) throws Exception {
        if(source.LT(this)){
            this.copy(prob, source);
            return true;
        }
        return false;
    }

    public void printer(Problem prob, StreamPrinter stream) throws Exception {
        this.objective.printer(prob, stream, codification);
        this.codification.printer(prob, stream);
    }
    
    
    
    
    public static int minIndex(Solution... sol) {
        int index = 0;
        for(int i=1; i<sol.length; i++){
            if(sol[i].LT(sol[index])){
                index= i;
            }
        }
        return index;
    }
    public static int maxIndex(Solution... sol) {
        int index = 0;
        for(int i=1; i<sol.length; i++){
            if(sol[i].GT(sol[index])){
                index= i;
            }
        }
        return index;
    }
}
