/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.com.runner;

import ProOF.com.language.Approach;

/**
 *
 * @author marcio
 * @param <No>
 */
public class FactoryChoise<No extends Approach> {
    private final String class_name;
    private final String name;
    private final int index;
    private final No node;

    public FactoryChoise(String class_name, String name, int index, No node) {
        this.class_name = class_name;
        this.name = name;
        this.index = index;
        this.node = node;
    }

    public String class_name() {
        return class_name;
    }
    public int index() {
        return index;
    }
    public String name() {
        return name;
    }
    public No node() {
        return node;
    }
    @Override
    public String toString() {
        return node.toString();
    }
}
