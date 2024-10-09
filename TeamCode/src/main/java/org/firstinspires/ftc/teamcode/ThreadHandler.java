package org.firstinspires.ftc.teamcode;

public class ThreadHandler {
    Thread thread;


    public ThreadHandler() {

        thread = null;
    }

    public void queue(Thread new_thread) {


        if (thread == null) {
            thread = new_thread;
        } else {
            if (live()) {
                return;
            }
            thread = new_thread;
        }

        thread.start();
    }

    public boolean live() {
        if (thread != null) {
            return thread.isAlive();
        }
        return false;
    }

    public void th_kill() {
        if (thread != null) {
            if (live()) {
                thread.interrupt();
            }
        }
    }
}
