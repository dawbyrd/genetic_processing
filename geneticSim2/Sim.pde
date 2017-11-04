import java.util.*;

import com.sun.xml.internal.ws.policy.spi.PolicyAssertionValidator;
import processing.core.PApplet;
import processing.core.PVector;


class Sim {

    static final int WIDTH = 1080;
    static final int HEIGHT = 720;
    public Obstacle[] obstacles;
    public boolean terminate = false;
    int N;
    float carSpeed;
    PApplet parent;
    Car curr;
    NeuralNet net;
    float range;
    float fitness = 0;    //This is the fitness of the car and its neural network
    boolean fit;

    //Creates a new Sim, composed of the obstacles, the car(including its Neural Network) and its attributes
    public Sim(Obstacle[] obst, float[] start, PApplet p, NeuralNet n, boolean fit, float sp, float r) {
        parent = p;
        obstacles = obst;
        curr = new Car(start[0], start[1], 0, p, fit);
        N = obstacles.length;
        carSpeed = sp;
        range = r;

        this.net = new NeuralNet(parent, false);
        this.net.config(60);
        this.net.setWeights(n.getWeights());

        this.fit = fit;
    }

    //This checks to see if the car hit the goal or an obstacle
    public boolean checkHit() {
        for (int i = 0; i < N; i++) {
            float intercept = (obstacles[i].isGoal) ? 30 : 60;

            if (obstacles[i].getVec().dist(curr.pos) < 30) {
                obstacles[i].hit();

                if (obstacles[i].isGoal) {
                    fitness += 100;
                } else {
                    fitness = 0;
                }
                return true;
            }
        }
        return false;
    }

    public float[] checkSensor() {

        float[] out = new float[32];

        for (int theta = 0; theta < 30; theta++) {

            float alpha = (2 * parent.PI / 30) * (theta);

            PVector endpoint = new PVector(curr.state[0] + 50 * parent.cos(alpha),  curr.state[1]+ 50 * parent.sin(alpha));

            float distance = 50;

            for (int i = 0; i < N - 1; i++) {

                PVector temp = intersect(obstacles[i], curr.pos, endpoint, 0);

                distance = Math.min(distance, (temp != null) ? temp.dist(curr.pos) : 50);
            }

            out[theta] = distance;

            parent.stroke(255);

            parent.point(curr.state[0] + distance*parent.cos(alpha), curr.state[1] + distance*parent.sin(alpha));

        }

        return out;

    }

//        for (int i = 0; i < N; i++) {
//            if (obstacles[i].isGoal) {
//                out.add(parent.atan((obstacles[i].getVec().y - curr.pos.y) / (obstacles[i].getVec().x - curr.pos.x)) -
//                        curr.state[2]);
//                out.add((float) obstacles[i].getVec().dist(curr.pos));
//            }
//        }
//        for (int i = 0; i < N; i++) {
//            if (obstacles[i].getVec().dist(curr.pos) < 100 && !obstacles[i].isGoal) {
//                out.add(parent.atan((obstacles[i].getVec().y - curr.pos.y) / (obstacles[i].getVec().x - curr.pos.x)));
//                out.add((float) obstacles[i].getVec().dist(curr.pos));
//            }
//        }
//        for (int i = out.size(); i < 10; i++) {
//            out.add(0f);
//        }
//        return out;
//    }

    public PVector intersect(Obstacle m, PVector A, PVector B, int d) {

        PVector mid = A.add(B).mult(0.5f);

        if (mid.dist(m.getVec()) < 20) {

            if(d==25) return mid;

            return intersect(m, A, mid, d + 1);
        } else {

            if(d==25) return null;

            return intersect(m, mid, B, d + 1);
        }

    }

    public void update(boolean override) {

        if (!terminate) {
            if (checkHit() || override) {
                fitness += (100 - obstacles[N - 1].getVec().dist(curr.pos));

                terminate = true;
            } else {
                if (obstacles[N - 1].getVec().dist(curr.pos) <= 40) {
                    //fitness+=1;
                }
                for (int i = 0; i < N - 1; i++) {
                    if (obstacles[i].getVec().dist(curr.pos) <= 40) fitness -= 1;
                }

                float[] input = checkSensor();

                input[30] = (curr.pos.x - obstacles[N-1].xPos);
                input[31] = curr.pos.y - obstacles[N-1].yPos;

                net.setInputs(input);

                float[] vel = net.getOutput();

                curr.update(2 * vel[0], 2 * vel[1]);
                curr.draw();
            }
        }
    }
}