/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.com.Results;

import ProOF.com.Linker.LinkerResults;
import ProOF.com.Stream.StreamPrinter;

/**
 *
 * @author marcio
 * @param <T>
 */
public final class Result<T> {
    private final String name;
    private final String format;
    private final T value;

    public Result(String name, String format, T value) {
        this.name = name;
        this.format = format;
        this.value = value;
    }
    public Result(String name, T value) {
        this(name, null, value);
    }
    public final void printer(StreamPrinter stream) throws Exception {
        if (value instanceof Integer){
            if(format==null){
                stream.printInt(name, (Integer)value);
            }else{
                stream.printInt(name, format, (Integer)value);
            }
        }else if (value instanceof Long){
            if(format==null){
                stream.printLong(name, (Long)value);
            }else{
                stream.printLong(name, format, (Long)value);
            }
        }else if (value instanceof Double){
            if(format==null){
                stream.printDbl(name, (Double)value);
            }else{
                stream.printDbl(name, format, (Double)value);
            }
        }else if (value instanceof String){
            if(format==null){
                stream.printString(name, (String)value);
            }else{
                stream.printString(name, format, (String)value);
            }
        }else {
            throw new UnsupportedOperationException("Not supported yet.");
        }
    }
    public final void results(LinkerResults link) throws Exception {
        if (value instanceof Integer){
            link.writeInt(name, (Integer)value);
        }else if (value instanceof Long){
            link.writeLong(name, (Long)value);
        }else if (value instanceof Double){
            link.writeDbl(name, (Double)value);
        }else if (value instanceof String){
            link.writeString(name, (String)value);
        }else {
            throw new UnsupportedOperationException("Not supported yet.");
        }
    }
    
    public static Result<Integer> Int(String name, String format, int value){
        return new Result<>(name, format, value);
    }
    public static Result<Integer> Int(String name, int value){
        return new Result<>(name, value);
    }
    public static Result<Long> Long(String name, String format, long value){
        return new Result<>(name, format, value);
    }
    public static Result<Long> Long(String name, long value){
        return new Result<>(name, value);
    }
    public static Result<Double> Dbl(String name, String format, double value){
        return new Result<>(name, format, value);
    }
    public static Result<Double> Dbl(String name, double value){
        return new Result<>(name, value);
    }
    public static Result<String> String(String name, String format, String value){
        return new Result<>(name, format, value);
    }
    public static Result<String> String(String name, String value){
        return new Result<>(name, value);
    }
}
