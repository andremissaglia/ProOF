/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.apl.UAV.map.Obstacle2D;
import ProOF.apl.UAV.map.Obstacle3DHalf;
import ProOF.apl.UAV.map.Point3D;
import ProOF.apl.UAV.map.PointGeo;
import ProOF.apl.UAV.map.Region2D;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.com.language.Factory;
import ProOF.utilities.uIO;
import java.awt.Color;
import java.awt.Font;
import java.awt.geom.Point2D;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.LinkedList;
import java.util.Locale;
import java.util.Scanner;

/**
 *
 * @author marcio
 */
public class fBlackmoreInstance extends Factory<BlackmoreInstance>{
    public static final fBlackmoreInstance obj = new fBlackmoreInstance();
    
    @Override
    public String name() {
        return "Instance Reader";
    }
    @Override
    public BlackmoreInstance build(int index) throws Exception {
        switch(index){
            case 0 : return new Instance2D();
            case 1 : return new Instance3DhalfHard();
            case 2 : return new Instance3DhalfRmd();
            case 3 : return new Instance2DRover();
            case 4 : return new Instance2Dhalf();
            case 5 : return new Instance2DEmterprise();
            case 6 : return new Instance3DEarth();
            case 7 : return new InstanceRealTests();
        }
        return null;
    }

    private static abstract class Instance extends BlackmoreInstance {
        protected File config;
        protected File map;
        
        protected String map_name;
        protected final Font font = new Font(Font.SANS_SERIF, Font.BOLD, 24);

  
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            config = link.File("Configuration for "+name(), null, "sgl");
            map = link.File("Map for "+name(), null, "sgl");
        }
        @Override
        public void load() throws FileNotFoundException, Exception {
            Scanner sc = new Scanner(config);
            loadConfig(sc);
            sc.close();
            sc = new Scanner(map);
            loadMap(sc);
            sc.close();
            
            rect = rectangle();
        }
        protected void loadConfig(Scanner sc) throws FileNotFoundException{
            start_state = uIO.ReadVectorDouble(sc);
            end_point = uIO.ReadVectorDouble(sc);
        }
        protected abstract void loadMap(Scanner sc ) throws Exception;

        @Override
        public String map_mame() {
            return map_name;
        }
        @Override
        public void results(LinkerResults link) throws Exception {
            super.results(link); //To change body of generated methods, choose Tools | Templates.
            link.writeString("map_name", map_name);
        }
    }
    
    private static class Instance2D extends Instance {
        @Override
        public String name() {
            return "Instance 2D";
        }
        @Override
        public int N() {
            return 2;
        }
        protected void loadMap(Scanner sc) throws FileNotFoundException{
            int J = uIO.ReadInt(sc);    //number of obstacles
            obstacles = new Obstacle[J];
            for(int j=0; j<J; j++){
                obstacles[j] = new Obstacle2D(""+j, sc);
            }
            map_name = uIO.ReadTrueName(sc);
        }
    }
    private static class Instance2Dhalf extends Instance {
        @Override
        public String name() {
            return "Instance 2Dhalf";
        }
        @Override
        public int N() {
            return 2;
        }
        protected void loadMap(Scanner sc) throws FileNotFoundException{
            int J = uIO.ReadInt(sc)/2;    //number of obstacles
            obstacles = new Obstacle[J];
            for(int j=0; j<J; j++){
                obstacles[j] = new Obstacle2D(""+j, sc);
            }
            map_name = uIO.ReadTrueName(sc);
            rect = rectangle();
        }
    }
    private static class Instance3DhalfHard extends Instance {
        @Override
        public String name() {
            return "Instance 3Dhalf-hard";
        }
        @Override
        public int N() {
            return 3;
        }
        private static int h(int J){
            if(J<=6){
                return 3; 
            }else{
                return J/2-3 + h(J-2);
            }
        }
        protected void loadMap(Scanner sc) throws FileNotFoundException{
            int J = uIO.ReadInt(sc);    //number of obstacles
            obstacles = new Obstacle[J+1];
            for(int j=0; j<J; j++){
                if(j==3){
                    double max = 6.0;
                    double h = 4.5;
                    int rgb = (int) (255 - (h*(200))/max);
                    Color color = new Color(rgb,rgb,rgb);
                    String name = String.format(Locale.ENGLISH, "%1.1f", h);
                    obstacles[j] = new Obstacle3DHalf(h, color, name,  font, sc);
                }else{
                    obstacles[j] = new Obstacle3DHalf(h(J), 2.0, 0.0, -5.0, font, sc);//3 4 6 9 13   
                }
                
            }
            map_name = uIO.ReadTrueNameWithOutExt(sc, "sgl");
            obstacles[J] = new Obstacle3DHalf(0, "ground", font);    //ground
            rect = rectangle();
        }
    }
    private static class Instance3DhalfRmd extends Instance {
        @Override
        public String name() {
            return "Instance 3Dhalf-rmd";
        }
        @Override
        public int N() {
            return 3;
        }
        protected void loadMap(Scanner sc) throws FileNotFoundException{
            int J = uIO.ReadInt(sc);    //number of obstacles
            obstacles = new Obstacle[J+1];
            for(int j=0; j<J/2; j++){
                obstacles[j] = new Obstacle3DHalf(Color.LIGHT_GRAY,"[ "+j+" ]", font, sc);
                //obstacles[j] = new Obstacle2D("[ "+j+" ]", sc);
            }
            for(int j=J/2; j<J; j++){
                double max = 5.0;
                double init = 2.0;
                int k = j-J/2;
                double h = init + k*(max-init)/(J-J/2-1);
                int rgb = (int) (255 - ((h-init)*(200))/(max-init));
                Color color = new Color(0,rgb,0);
                String name = String.format(Locale.ENGLISH, "%1.1f", h);
                obstacles[j] = new Obstacle3DHalf(h, color, name,  font, sc);
            }
            map_name = uIO.ReadTrueNameWithOutExt(sc, "sgl");
            obstacles[J] = new Obstacle3DHalf(0, "ground", font);    //ground
            rect = rectangle();
        }
    }
    
    private static class InstanceRealTests extends Instance {
        @Override
        public String name() {
            return "Instance RealTests";
        }
        @Override
        public int N() {
            return 3;
        }
        protected void loadMap(Scanner sc) throws FileNotFoundException{
            int J = uIO.ReadInt(sc);    //number of obstacles
            double height_min = uIO.ReadDouble(sc);
            double height_max = uIO.ReadDouble(sc);
            obstacles = new Obstacle[J+1];
            for(int j=0; j<J; j++){
                sc.nextLine();
                String name = sc.nextLine();
                String height = sc.nextLine();
                if(height.equals("+INF")){
                    obstacles[j] = new Obstacle3DHalf(Color.LIGHT_GRAY,"[ "+j+" ]", font, sc, false);
                }else{
                    double h = Double.parseDouble(height);
                    int rgb = (int) (255 - ((h-height_min)*(200))/(height_max-height_min));
                    Color color = new Color(0,rgb,0);
                    obstacles[j] = new Obstacle3DHalf(h, color, name,  font, sc, false);
                }
            }
            map_name = uIO.ReadTrueNameWithOutExt(sc, "sgl");
            obstacles[J] = new Obstacle3DHalf(height_min, "ground", font);    //ground
            rect = rectangle();
        }
    }
    
    private static class Instance2DRover extends BlackmoreInstance {
        private File config;
        private File map;
        
        
        private String map_name;
        
        @Override
        public String name() {
            return "Instance 2D Rover";
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            config = link.File("Configuration for Rover 2D", null, "txt");
            map = link.File("Map for Rover 2D", null, "txt");
        }
        @Override
        public int N() {
            return 2;
        }
        @Override
        public void load() throws FileNotFoundException, Exception {
            loadConfig();
            loadMap();
        }
        private void loadConfig() throws FileNotFoundException{
            Scanner sc = new Scanner(config);
            start_state = uIO.ReadVectorDouble(sc);
            end_point = uIO.ReadVectorDouble(sc);
//            
//            
//            start_state = new double[dynamic.n()];
//            start_state[0] = sc.nextDouble();
//            start_state[1] = sc.nextDouble();
//            start_state[1] = sc.nextDouble();
//            start_state[1] = sc.nextDouble();
//            sc.nextLine();
//            
//            end_point = new double[2];
//            end_point[0] = sc.nextDouble();
//            end_point[1] = sc.nextDouble();
//            sc.nextLine();

            sc.close();
        }
        private void loadMap() throws FileNotFoundException{
            Scanner sc = new Scanner(map);
            String text = "";
            while(sc.hasNextLine()){
                String line = sc.nextLine();
                if(line.equals("<TrueName>")){
                    map_name = sc.nextLine();
                }else{
                    text += line;
                }
            }
            sc.close();
            
            System.out.println("text = {"+text+"}");
            text = text.replaceAll("]", "@");
            System.out.println("text@ = {"+text+"}");
            text = text.replaceAll("@ ", "@");
            text = text.replaceAll("@\t", "@");
            System.out.println("text@ = {"+text+"}");
            
            text = text.replaceAll("\\[", "");
            System.out.println("text[ = {"+text+"}");
            String obs[] = text.split("@");
            obstacles = new Obstacle[obs.length];
            for(int j=0; j<obs.length; j++){
                System.out.println("obs["+j+"] = {"+obs[j]+"}");
                String corners[] = obs[j].split(";");
                Point2D points[] = new Point2D[corners.length];
                for(int i=0; i<corners.length; i++){
                    System.out.println("corners["+i+"] = {"+corners[i]+"}");
                    Scanner sc2 = new Scanner(corners[i]);
                    points[corners.length-1-i] = new Point2D.Double(sc2.nextDouble(), sc2.nextDouble());
                    sc2.close();
                }
                obstacles[j] = new Obstacle2D(""+j, points);
            }
            rect = rectangle();
        }
        @Override
        public String map_mame() {
            return map_name;
        }
        @Override
        public void results(LinkerResults link) throws Exception {
            super.results(link); //To change body of generated methods, choose Tools | Templates.
            link.writeString("map_name", map_name);
        }
        
    }
    
    public static class Instance2DEmterprise extends BlackmoreInstance {
        private File config;
        private File map;
        
        private String map_name;
        
        public double fly_altitude;
        
        @Override
        public String name() {
            return "Instance 2D Emterprise";
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            config = link.File("Configuration for Blackmore 2D Ono", null, "txt");
            map = link.File("Map for Blackmore 2D Ono", null, "txt");
        }
        @Override
        public int N() {
            return 2;
        }
        @Override
        public void load() throws FileNotFoundException, Exception {
            loadConfig();
            loadMap();
        }
        private void loadConfig() throws FileNotFoundException{
            Scanner sc = new Scanner(config);
            String line = sc.nextLine();
            start_state = new double[4];
            String v[] = line.split(",");
            for(int i=0; i<v.length; i++){
                start_state[i] = Double.parseDouble(v[i]);
            }
            line = sc.nextLine();
            v = line.split(",");
            end_point = new double[v.length];
            for(int i=0; i<v.length; i++){
                end_point[i] = Double.parseDouble(v[i]);
            }

            fly_altitude = sc.nextDouble();
            sc.nextLine();
            
            sc.close();
        }
        
        private String addPolygon(String text, String name, LinkedList<Obstacle> obstacles){
            text = text.substring(text.indexOf("corners:\n")+9);
            //System.out.println("text = {"+text.replaceAll("\n", "@")+"}");
            LinkedList<Point2D> list = new LinkedList<Point2D>();
            while(text.length()>0 && text.charAt(0)=='-'){
                text = addPoint(text, list);
            }
            //System.out.println("list points = "+list);
            //System.out.println("text = {"+text.replaceAll("\n", "@")+"}");
            
            obstacles.addLast(new Obstacle2D(name, list.toArray(new Point2D[list.size()])));
            
            return text;
        }
        private String addRectangle(String text, String name, LinkedList<Obstacle> obstacles){
            LinkedList<Point2D> center = new LinkedList<Point2D>();
            text = addPoint(text, center, "center:[", ",", "]\n");
            LinkedList<Double> sizes = new LinkedList<Double>();
            text = addDouble(text, sizes, "length:", "\n");
            text = addDouble(text, sizes, "width:", "\n");
            text = addDouble(text, sizes, "rotation:", "\n");
//            System.out.println("center = "+center);
//            System.out.println("sizes = "+sizes);
            
            obstacles.addLast(new Obstacle2D(name, center.getFirst(), sizes.get(0), sizes.get(1), sizes.get(2)));
            
            return text;
        }
        private String addPoint(String text, LinkedList<Point2D> list){
            return addPoint(text, list, "-[", ",", "]\n");
        }
        private String addPoint(String text, LinkedList<Point2D> list, String start, String split, String end){
            String point = text.substring(text.indexOf(start)+start.length(), text.indexOf(end));
            String cord[] = point.split(split);
            list.addLast(new Point2D.Double(Double.parseDouble(cord[0]), Double.parseDouble(cord[1])));
            return text.substring(text.indexOf(end)+end.length());
        }
        private String addDouble(String text, LinkedList<Double> list, String start, String end){
            String val = text.substring(text.indexOf(start)+start.length(), text.indexOf(end));
            list.addLast(Double.parseDouble(val));
            return text.substring(text.indexOf(end)+end.length());
        }
        private void loadMap() throws FileNotFoundException, Exception{
            Scanner sc = new Scanner(map);
            String text = "";
            while(sc.hasNextLine()){
                String line = sc.nextLine();
                if(line.equals("<TrueName>")){
                    map_name = sc.nextLine();
                }else{
                    text += line+"\n";
                }
            }
            sc.close();
            
            LinkedList<Obstacle> obs = new LinkedList<Obstacle>();
            
            //System.out.println("text = {"+text.replaceAll("\n", "@")+"}");
            text = text.substring(text.indexOf("obstacles:\n")+11, text.indexOf("features:")).replaceAll(" ", "");
            //System.out.println("text = {"+text.replaceAll("\n", "@")+"}");
            while(text.contains(":\n")){
                String name = text.substring(0, text.indexOf(":\n"));
                //System.out.printf("name(%s):\n", name);
                text = text.substring(text.indexOf("shape:")+6);
                //System.out.println("text = {"+text.replaceAll("\n", "@")+"}");
                String shape = text.substring(0, text.indexOf("\n"));
                //System.out.printf("shape(%s):\n", shape);
                if(shape.equals("polygon")){
                    text = addPolygon(text, name, obs);
                }else if(shape.equals("rectangle")){
                    text = addRectangle(text, name, obs);
                }else{
                    throw new Exception("shape = '"+shape+"' is not valid");
                }
            }
//            
//            obstacles = new Obstacle[obs.length];
//            for(int j=0; j<obs.length; j++){
//                System.out.println("obs["+j+"] = {"+obs[j]+"}");
//                String corners[] = obs[j].split(";");
//                Point2D points[] = new Point2D[corners.length];
//                for(int i=0; i<corners.length; i++){
//                    System.out.println("corners["+i+"] = {"+corners[i]+"}");
//                    Scanner sc2 = new Scanner(corners[i]);
//                    points[corners.length-1-i] = new Point2D.Double(sc2.nextDouble(), sc2.nextDouble());
//                    sc2.close();
//                }
//                obstacles[j] = new Obstacle2D(""+j, font, points);
//            }
            //obstacles = new Obstacle[0];            
            obstacles = obs.toArray(new Obstacle[obs.size()]);
            rect = rectangle();
        }
        @Override
        public String map_mame() {
            return map_name;
        }
        @Override
        public void results(LinkerResults link) throws Exception {
            super.results(link); //To change body of generated methods, choose Tools | Templates.
            link.writeString("map_name", map_name);
        }
        
    }
    
    private static class GeoPath{
        private final String name;
        private final String type;
        private final LinkedList<PointGeo> points;

        public GeoPath(Scanner sc) {
            this.name = sc.nextLine();
            this.type = sc.nextLine();
            this.points = readPoints(sc);
        }
        public GeoPath(String name, String type, LinkedList<PointGeo> points) {
            this.name = name;
            this.type = type;
            this.points = points;
        }
        private static GeoPath nextPolygon(Scanner sc){
            if(sc.hasNextLine()){
                String name = sc.nextLine();
                if(sc.hasNextLine() && !name.equals("<end-file>")){
                    String type = sc.nextLine();
                    LinkedList<PointGeo> points = readPoints(sc);
                    points.removeLast();
                    return new GeoPath(name, type, points);
                }
            }
            return null;
        }
        
        private static LinkedList<PointGeo> readPoints(Scanner sc){
            LinkedList<PointGeo> list = new LinkedList<PointGeo>();
            String points[] = sc.nextLine().split(" ");
            for(String p : points){
                list.addFirst(new PointGeo(p));
            }
            return list;
        }
    }
    
    private static class Instance3DEarth extends BlackmoreInstance {
        protected File map;
        
        protected String map_name;
        protected final Font font = new Font(Font.SANS_SERIF, Font.BOLD, 24);
        
        
        @Override
        public String name() {
            return "Instance 3D Earth";
        }
        @Override
        public int N() {
            return 3;
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            map = link.File("Map for "+name(), null, "sgl");
        }
        
        private double[] calcStart(GeoPath start, double pitch, double velocity, double dh) {
            Point3D p1 = new Point3D(base, start.points.getFirst());
            Point3D p2 = new Point3D(base, start.points.getLast());
            
            if(p2.h>=0){
                double ang = p2.minus(p1).angle();
                return new double[]{
                    p2.x, p2.y, Math.max(p2.h,dh), 
                    Math.cos(ang)*velocity, 
                    Math.sin(ang)*velocity,
                    0
                };
            }else{
                double step = (-p2.h+dh)/Math.tan(pitch*Math.PI/180.0);
                double ang = p2.minus(p1).angle();
                return new double[]{
                    p2.x + Math.cos(ang)*step, 
                    p2.y + Math.sin(ang)*step,
                    dh, 
                    Math.cos(ang)*velocity, 
                    Math.sin(ang)*velocity,
                    0
                };
            }
        }
        private double[] calcEnd(GeoPath end, double pitch, double velocity, double dh) {
            Point3D p1 = new Point3D(base, end.points.getFirst());
            Point3D p2 = new Point3D(base, end.points.getLast());
            
            if(p1.h>=0){
                double ang = p2.minus(p1).angle();
                return new double[]{
                    p1.x, p1.y, Math.max(p1.h,dh), 
                    Math.cos(ang)*velocity, 
                    Math.sin(ang)*velocity,
                    0
                };
            }else{
                double step = (-p1.h+dh)/Math.tan(pitch*Math.PI/180.0);
                double ang = p2.minus(p1).angle();
                return new double[]{
                    p1.x - Math.cos(ang)*step, 
                    p1.y - Math.sin(ang)*step,
                    dh, 
                    Math.cos(ang)*velocity, 
                    Math.sin(ang)*velocity,
                    0
                };
            }
        }
        
        @Override
        public void load() throws FileNotFoundException, Exception {
            Scanner sc = new Scanner(map);
            
            sc.nextLine();
            final double base_alt = Double.parseDouble(sc.nextLine());
            
            sc.nextLine();
            final double base_vel = Double.parseDouble(sc.nextLine());
            
            GeoPath start = new GeoPath(sc);
            GeoPath end = new GeoPath(sc);
            
            LinkedList<GeoPath> paths = new LinkedList<GeoPath>();
            GeoPath path = GeoPath.nextPolygon(sc);
            while(path!=null){
                paths.addLast(path);
                path = GeoPath.nextPolygon(sc);
            }
            map_name = uIO.ReadTrueName(sc);
            sc.close();
            
            base = new PointGeo(start.points.getFirst().longitude, start.points.getFirst().latitude, base_alt);
            
            start_state = calcStart(start, 20.0, base_vel, 50);
            end_point = calcEnd(end, 20.0, base_vel, 50);
            
            LinkedList<Obstacle> obs = new LinkedList<Obstacle>();
            for(GeoPath pa : paths){
                LinkedList<Point2D> points = new LinkedList<Point2D>();
                for(PointGeo g : pa.points){
                    Point3D p = new Point3D(base, g);
                    points.addLast(new Point2D.Double(p.x, p.y));
                }
                obs.addLast(new Region2D(pa.name, true, points.toArray(new Point2D[points.size()])));
                
            }
            obs.addLast(new Obstacle3DHalf(0, "ground", font));
            obstacles = obs.toArray(new Obstacle[obs.size()]);
            
            Si = new Point3D(base, start.points.getFirst());
            System.out.printf("start-i = [%g\t%g\t%g]\n", Si.x, Si.y, Si.h);
            Sf = new Point3D(base, start.points.getLast());
            System.out.printf("start-f = [%g\t%g\t%g]\n", Sf.x, Sf.y, Sf.h);
            
            Ei = new Point3D(base, end.points.getFirst());
            System.out.printf("end-i = [%g\t%g\t%g]\n", Ei.x, Ei.y, Ei.h);
            Ef = new Point3D(base, end.points.getLast());
            System.out.printf("end-f = [%g\t%g\t%g]\n", Ef.x, Ef.y, Ef.h);
            
        }
        @Override
        public String map_mame() {
            return map_name;
        }
        @Override
        public void results(LinkerResults link) throws Exception {
            super.results(link); //To change body of generated methods, choose Tools | Templates.
            link.writeString("map_name", map_name);
        }
    }
}
