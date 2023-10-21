package org.firstinspires.ftc.teamcode.subsystems.stateMachineController;

public class Edge {
    Class<State> to;
    EdgeCallback condition;

    /* Here's a crash course on Lambdas because I had to learn what they
     * were and I learn things best by explaining things back to people.
     * (I have no doubt you've noticed this, Milo)
     *
     * "EdgeCallback condition = () -> StateMachineTest.gamepad.a;"
     * sets condition to an instance of an anonymous class (never explicitly
     * defined, does not have a name) that is an implementation of EdgeCallback,
     * a functional interface, with the only method defined as a method that
     * returns "StateMachineTest.gamepad.a"
     *
     * This IS equivalent to
     * condition = () - > {
     *      return StateMachineTest.gamepad.a;
     * };
     *
     * This is NOT equivalent to
     * "value = StateMachineTest.gamepad.a;
     * condition = () -> value;"
     * because "() ->" defines the function, not the return value,
     * which means reevaluate the method each time we run it,
     *
     * This means that we can save code without evaluating it
     * and that we can pass around unevaluated functions that
     * can be evaluated later, multiple times.
     */

    /* Create a new Edge by writing eg.
    * "edge = new Edge(IdleState, () -> StateMachineTest.gamepad.a); */

    /* condition is an instance of an implementation of EdgeCallback. */
    public Edge (Class to, EdgeCallback condition) {
        this.to = to;
        this.condition = condition;
    }

    @FunctionalInterface
    public interface EdgeCallback {
        boolean getCallback();
    }

    public boolean getCallback() {
        return condition.getCallback();
    }
}
