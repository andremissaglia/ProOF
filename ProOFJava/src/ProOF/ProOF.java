/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF;

import ProOF.apl.factorys.fRun;
import ProOF.com.model.CallBack;
import ProOF.com.model.DrawPanel;
import ProOF.com.model.Model;
import ProOF.com.model.dVertex;
import ProOF.com.model.Mount;
import ProOF.com.runner.Runner;
import ProOF.opt.abst.problem.meta.Best;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.event.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.LinkedList;
import java.util.Locale;
import java.util.Scanner;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;

/**
 *
 * @author marcio
 */
public class ProOF {
    
    
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) throws FileNotFoundException, Exception {
       /* Locale.setDefault(Locale.ENGLISH);
        String name = "brazil58";
        TSPInstance inst = new TSPInstance();
        Scanner sc = new Scanner(new File("./TSP/"+name+".tsp"));
        inst.optimal = Double.parseDouble(sc.nextLine());
        inst.N = Integer.parseInt(sc.nextLine());
        inst.Cij = new double[inst.N][inst.N];
//        double cx[] = new double[inst.N];
//        double cy[] = new double[inst.N];
//        for(int n=0; n<inst.N; n++){
//            String line = sc.nextLine();
//            line = line.replaceAll(" ", ",");
//            line = line.replaceAll(",,", ",");line = line.replaceAll(",,", ",");line = line.replaceAll(",,", ",");
//            line = line.replaceAll(",,", ",");line = line.replaceAll(",,", ",");
//            line = line.replaceAll(",,", ",");line = line.replaceAll(",,", ",");
//
//            if(line.charAt(0)==','){
//                line = line.substring(1);
//            }
//            System.out.println(line);
//            String v[] = line.split(",");
//            System.out.println(v.length);
//            cx[n] = Double.parseDouble(v[1]);
//            cy[n] = Double.parseDouble(v[2]);
//        }
//        for(int i=0; i<inst.N; i++){
//            for(int j=0; j<inst.N; j++){
//                inst.Cij[i][j] = Math.sqrt(Math.pow(cx[i]-cx[j], 2) + Math.pow(cy[i]-cy[j], 2));
//            }
//        }
        for(int i=0; i<inst.N; i++){
            for(int j=i+1; j<inst.N; j++){
                inst.Cij[i][j] = sc.nextDouble();
                inst.Cij[j][i] = inst.Cij[i][j];
            }
        }
        sc.close();
        
        PrintStream out = new PrintStream(new File("./TSP/"+name+".txt"));
        out.println("Valor_otimo");
        out.println(inst.optimal);
        out.println("Total_de_cidades");
        out.println(inst.N);
        out.println("Matriz_dos_custos_dos_caminhos");
        for(int i=0; i<inst.N; i++){
            for(int j=0; j<inst.N; j++){
                out.printf("%12g ", inst.Cij[i][j]);
            }
            out.println();
        }
        out.close();
        */
        boolean local = false;
        if (args == null || args.length == 0) {
            local = true;
            //args = new String[]{"model"};            
           // args = new String[]{"mount"};            
            //args = new String[]{"run", "./config/Earth/job_full", "./config/Earth/"};
            //args = new String[]{"run", "./config/CCQSP/jobUV", "./config/CCQSP/"};
           // args = new String[]{"run", "./config/CCQSP/job2States", "./config/CCQSP/"};
            //args = new String[]{"run", "./config/teste/job_local/running/pfpj", "./config/teste/input/"};
            //args = new String[]{"run", "./config/CCQSP/job_local/waiting/wxsl", "./config/CCQSP/input/"};
            //args = new String[]{"run", "./config/dynamic/full", "./config/dynamic/input/"};
            
            //args = new String[]{"enterprise","C:/ProOFJava/path1/input.yaml"};
            
            
            //args = new String[]{"run", "F:\\zProOF\\client_space\\job_local\\waiting\\guji", "F:\\zProOF\\client_space\\input\\"};
            //args = new String[]{"run", "F:\\Dropbox\\ProOF\\ProOFClient\\work_space\\job_local\\finished\\skyl", "F:\\Dropbox\\ProOF\\ProOFClient\\work_space\\input\\"};
            
            args = new String[]{"run", "../work_space/job_local/finished/xdzg", "../work_space/input"};
            //args = new String[]{"run", "../ProOFClient/work_space/job_local/waiting/snxx", "../ProOFClient/work_space/input"};
            
//args = new String[]{"run", "../ProOFClient/results/MILPC_T20/job_local/waiting/loyl", "../ProOFClient/results/MILPC_T20/input"};
            //args = new String[]{"run", "./config/CCQSP/nprh-true2", "./config/CCQSP/input"};
            //args = new String[]{"run", "./config/OnofreTests/dzrp", "./config/OnofreTests/input"};
        }
        if(args.length > 1 && args[0].equals("enterprise")){
            args = Enterprise(args);
        }
        try{
            starting(args, local);
        }catch(Throwable ex){
            ex.printStackTrace(System.err);
            PrintStream log = new PrintStream(new File("log_error.txt"));
            ex.printStackTrace(log);
            log.close();
        }
        if(!local){
            System.exit(0);
        }
    }

    private static void starting(String[] args, boolean local) throws Exception {
        Locale.setDefault(Locale.ENGLISH);
        if (args == null || args.length < 1) {
            throw new Exception("don't have arguments");
        } else if (args[0].equals("model")) {
            Model.PRINT = true;
            Model model = new Model();
            model.create(fRun.obj);
            model.savePof("model.pof");
            model.saveSgl("model.sgl");
        } else if (args[0].equals("mount")) {
            Mount.PRINT = false;
            Mount task = new Mount();
            task.create(fRun.obj);
            task.savePof("task.pof");
            task.saveSgl("task.sgl");
        } else if (args[0].equals("run")) {
            Runner.PRINT = false;
            Runner.LOCAL = local;
            Best.force_finish(true);
            Runner runner = new Runner(new File(args[1]), new File(args[2]), fRun.obj);
            runner.run();
        } else {
            throw new Exception(String.format("arg[0]='%s' is not recognized.", args[0]));
        }
    }

    public static double inv_erf(double x, double lookup_arg[], double lookup_inv_erf[]){
        for(int i=0; i<=52; i++){
            if(x<lookup_arg[i+1]){
                double M = (lookup_inv_erf[i+1] - lookup_inv_erf[i])/(lookup_arg[i+1]-lookup_arg[i]);
                double w = x-lookup_arg[i];
                
                //System.out.printf("x = %g | M = %g | w = %g | f = %g\n", x, M, w, lookup_inv_erf[i] + w*M);
                
                return lookup_inv_erf[i] + w*M;
            }
        }
        return lookup_inv_erf[52];
    }
    private static void main4() throws FileNotFoundException, Exception {
        Runner runner = new Runner(new File("D:/fFramework/ProOF/ProOFClient_vA/work_space/job/aahu.job"), new File("D:/fFramework/ProOF/ProOFClient_vA/work_space/input/"), fRun.obj);
        runner.run();
    }

    public static void main3(String[] args) throws FileNotFoundException {
        // TODO code application logic here

        Scanner sc = new Scanner(new File("GraphModel.txt"));
        LinkedList<Integer> list = new LinkedList<Integer>();
        String code = "";
        while (sc.hasNextLine()) {
            String line = sc.nextLine();
            //line = line.replaceAll("\t", "");
            //line = line.replaceAll(" ", "");
            code += line;
            list.addLast(line.length());
        }
        sc.close();

        System.out.println(code);

        //while(code.matches("(node\\[)(.*)")){

        //    System.out.println();
        //}
        String RgInt = "(([1-9][0-9]+)|([0-9]))";
        String RgName = "[a-zA-Z_][(\\w)]+";

        String head[] = new String[]{
            "node", "\\[", "id", "=", RgInt, ",", "name", "=", RgName, "\\]", ".*"
        };
        String make_head = make(head);



        //String s = String.format("_node_[_id_]", "node", "[", "id", "=", RgInt, ",", "name", "=", RgName, "]");

        //System.out.println(code.matches("((\\s)*node(\\s)*\\[(\\s)*id(\\s)*=(\\s)*"+RgInt+"(\\s)*,(\\s)*name(\\s)*=(\\s)*"+RgName+"(\\s)*\\])(.*)"));

        System.out.println(code.matches(make_head));


        //System.out.println("."+sc.next());
        //while(sc.){
        //System.out.println(sc.next());
        //}
        /*sc.close();
         while(sc.hasNext("[n][o][d][e]")){//"[a-zA-Z0-9_]+"
         System.out.println("."+sc.next("[node]"));
         }*/


        // main2(args);
    }

    public static String make(String... block) {
        String r = "(\\s)*";
        for (String b : block) {
            r += "(" + b + ")(\\s)*";
        }
        System.out.println(r);
        return r;
    }

    public static void main2(String[] args) throws FileNotFoundException {
        // TODO code application logic here


        args = new String[]{"model"};
        if (args[0].equals("model")) {
            final Model model = new Model();
            model.create(fRun.obj);

            model.savePof("model.pof");
            model.saveSgl("model.sgl");

            final DrawPanel draw = new DrawPanel(new CallBack() {
                @Override
                public void main() {
                }
            });
            draw.setProject(model);

            draw.setPreferredSize(new Dimension(970, 650));
            draw.setBackground(Color.WHITE);
            final JFrame frame = new JFrame("Frame");
            frame.setLayout(new FlowLayout(FlowLayout.CENTER));
            frame.add(draw);

            //frame.setSize(1000, 700);
            frame.setSize(new Dimension(1000, 700));
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setVisible(true);

            frame.addComponentListener(new ComponentAdapter() {
                @Override
                public void componentResized(final ComponentEvent e) {
                    System.out.printf("componentResized[%d, %d]\n", frame.getWidth(), frame.getHeight());
                    draw.setPreferredSize(new Dimension(frame.getWidth() - 30, frame.getHeight() - 50));
                    SwingUtilities.updateComponentTreeUI(frame);
                }
            });
            frame.addKeyListener(new KeyAdapter() {
                @Override
                public void keyReleased(KeyEvent e) {
                    if (e.getKeyCode() == KeyEvent.VK_SPACE) {
                        dVertex.paint_lines++;
                        draw.repaint();
                    }
                }
            });
        }
    }

    public static void setLookAndFeel(String lookAndFeelds, JFrame frame) {
        try {
            UIManager.setLookAndFeel(lookAndFeelds);
            SwingUtilities.updateComponentTreeUI(frame);
            //fileChooserInstances.updateUI();
            //fileChooserBatchs.updateUI();
        } catch (ClassNotFoundException ex) {
            JOptionPane.showMessageDialog(null, "Erro: ClassNotFoundException", "Atualizando LookAndFeel", JOptionPane.INFORMATION_MESSAGE);
        } catch (InstantiationException ex) {
            JOptionPane.showMessageDialog(null, "Erro: InstantiationException", "Atualizando LookAndFeel", JOptionPane.INFORMATION_MESSAGE);
        } catch (IllegalAccessException ex) {
            JOptionPane.showMessageDialog(null, "Erro: IllegalAccessException", "Atualizando LookAndFeel", JOptionPane.INFORMATION_MESSAGE);
        } catch (UnsupportedLookAndFeelException ex) {
            JOptionPane.showMessageDialog(null, "Erro: UnsupportedLookAndFeelException", "Atualizando LookAndFeel", JOptionPane.INFORMATION_MESSAGE);
        }
    }

    
    public static String between(String line, String start, String end){
        line = line.substring(line.indexOf(start)+start.length());
        line = line.substring(0, line.indexOf(end));
        return line;
    }
    
    private static LinkedList<Parameter> catalogue = new LinkedList<Parameter>();
    public static Parameter find(String name){
        for(Parameter p : catalogue){
            if(name.equals(p.name)){
                return p;
            }
        }
        return null;
    }
    public static String[] Enterprise(String[] args) throws FileNotFoundException, Exception {
        File path1 = new File(args[1]);
        if(path1.exists()){
            if(!path1.isFile()){
                throw new Exception("it is not a file '"+path1.getCanonicalPath()+"'");
            }
        }else{
            throw new Exception("file '"+path1.getCanonicalPath()+"' not exists");
        }
        Scanner input_catalogue = new Scanner(new File(path1.getParentFile(), "catalogue"));
        while(input_catalogue.hasNextLine()){
            catalogue.addLast(new Parameter(input_catalogue));
        }
        input_catalogue.close();
        
        Scanner input = new Scanner(path1);

        String env = input.nextLine();
        String start = input.nextLine();
        String goal = input.nextLine();
        String fly_altitude = input.nextLine();
        String executor = input.nextLine();
        LinkedList<Parameter> list = new LinkedList<Parameter>();
        while(input.hasNextLine()){
            String line = input.nextLine();
            String name = line.substring(0,line.indexOf(":"));
            String value = line.substring(line.indexOf(":")+1).replace(" ", "");
            Parameter param = find(name);
            if(param==null){
                throw new Exception("In line '"+line+"', the name parameter '"+name+"' doesn't has found\ncatalogue = "+catalogue);
            }else{
                param.value = value;
                list.addLast(param);
            }
        }
//        String delta = input.nextLine();
//        String std_position = input.nextLine();
//        String std_velocity = input.nextLine();
//        String inc_factor = input.nextLine();
//        String time_horizon = input.nextLine();
//        
//        String waypoints = input.nextLine();
//        String max_velocity = input.nextLine();
//        String max_control = input.nextLine();
//        String ti_lim = input.nextLine();
//        String enable_plot = input.nextLine();
//        String save_png = input.nextLine();        
        
        
        input.close();
        
        create_config(path1, start, goal, fly_altitude);
        copy_ProOF(path1, executor, list);
        copy_map(path1, env);
        
        args = new String[]{"run", new File(path1.getParentFile(), "ProOF").getCanonicalPath(), path1.getParentFile().getCanonicalPath()+File.separator};
        //System.out.println("args[1] = "+args[1]);
        //System.out.println("args[2] = "+args[2]);
        return args;
    }
    private static void copy_map(File path1, String env) throws FileNotFoundException{
        env = between(env, "\"[", "]\"");
        //System.out.println(env);
        
        PrintStream Copy = new PrintStream(new File(path1.getParentFile(), "map"));
        Scanner toCopy = new Scanner(new File(env));
        while(toCopy.hasNextLine()){
            Copy.println(toCopy.nextLine());
        }
        toCopy.close();
        Copy.close();
    }
    
    private static class Parameter{
        private final String name;
        private final String class_name;
        private final int ref_line;
        private String value;
        public Parameter(Scanner catalogue) {
            String line = catalogue.nextLine();
            String v[] = line.split(":");
            this.name = v[0];
            this.ref_line = Integer.parseInt(v[1]);
            this.class_name = v[2];
        }

        @Override
        public String toString() {
            return name;
        }
    }
    
    private static Parameter[] filtro(String line, LinkedList<Parameter> list){
        LinkedList<Parameter> r = new LinkedList<Parameter>();
        for(Parameter p : list){
            if(line.equals(p.class_name)){
                r.addLast(p);
            }
        }
        Parameter  vet[] = r.toArray(new Parameter[r.size()]);
        for(int i=1; i<vet.length; i++){
            int j=i-1;
            Parameter aux = vet[i];
            while(j>=0 && aux.ref_line<vet[j].ref_line){
                vet[j+1] =  vet[j];
                j--;
            }
            vet[j+1] = aux;
        }
//        for(int i=0; i<vet.length; i++){
//            System.out.printf("[%d] : %20s %d\n", i, vet[i].name, vet[i].ref_line);
//        }
        return vet;
    }
    
    private static void copy_ProOF(File path1, String executor, LinkedList<Parameter> list) throws FileNotFoundException{
        executor = executor.substring(executor.indexOf(":")+1).replace(" ", "");
        
//        System.out.println(delta);
//        System.out.println(time_horizon);
//        System.out.println(executor);
        
        PrintStream Copy = new PrintStream(new File(path1.getParentFile(), "ProOF"));
        Scanner toCopy = new Scanner(new File(path1.getParentFile(), executor));
        while(toCopy.hasNextLine()){
            String line = toCopy.nextLine();
            
            Parameter[] filtro = filtro(line, list);
            if(filtro.length>0){
                int count = 1;
                Copy.println(line); count++;
                Copy.println(toCopy.nextLine()); count++;
                Copy.println(toCopy.nextLine()); count++;
                for(Parameter p : filtro){
                    while(p.ref_line>count){
                        Copy.println(toCopy.nextLine()); count++;
                    }
                    Copy.println(p.value); count++;
                    toCopy.nextLine();
                }
            }else{
                Copy.println(line);
            }
                
//            if(line.equals("ProOF.apl.UAV.abst.uncertainty.Delta")){
//                change(toCopy, Copy, line, 3, delta);
//            }else if(line.equals("ProOF.apl.UAV.gen.linear.vehicle.parts.fLinearDynamic$VelocityDt")){
//                change(toCopy, Copy, line, 4, time_horizon);
//            }else if(line.contains("ProOF.apl.UAV.mission.Blackmore.fBlackmoreInstance$")){
//                change(toCopy, Copy, line, 3, "config", "map");
//            }else{
//                Copy.println(line);
//            }
        }
        toCopy.close();
        Copy.close();
    }
    private static void create_config(File path1, String start, String goal, String fly_altitude) throws FileNotFoundException{
        start = between(start, "(", ")").replace(" ", "");
        goal = between(goal, "(", ")").replace(" ", "");
        fly_altitude = fly_altitude.substring(fly_altitude.indexOf(":")+1);
//        System.out.println(start);
//        System.out.println(goal);
        
        PrintStream out = new PrintStream(new File(path1.getParentFile(), "config"));
        out.println(start);
        out.println(goal);
        out.println(fly_altitude);
        out.println("<TrueName>");
        out.println("config");
        out.close();
    }
    
    private static void change(Scanner toCopy, PrintStream Copy, String curLine, int index, String... newLine) {
        if(index<=0){
            Copy.println(newLine[0]);
            for(int i=1; i<newLine.length; i++){
                Copy.println(newLine[i]);
                toCopy.nextLine();  //ignore
            }
        }else{
            Copy.println(curLine);
            for(int i=1; i<index; i++){
                Copy.println(toCopy.nextLine());
            }
            Copy.println(newLine[0]);
            toCopy.nextLine();  //ignore
            for(int i=1; i<newLine.length; i++){
                Copy.println(newLine[i]);
                toCopy.nextLine();  //ignore
            }
        }
    }
    
    
}
