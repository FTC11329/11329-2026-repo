package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class Tuple {
    Object o1;
    Object o2;
    public Tuple(Object o1, Object o2) {
        this.o1 = o1;
        this.o2 = o2;
    }

    public Object get1() {
        return o1;
    }

    public Object get2() {
        return o2;
    }

    public void set1(Object o1) {
        this.o1 = o1;
    }

    public void set2(Object o2) {
        this.o2 = o2;
    }
}

