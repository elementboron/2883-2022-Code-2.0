package frc.robot;

public class CommonMethodExtensions {

    public static double tweener(double base, double target, double current)
    {
        return base * (current/target);
    }

    public static double tweenerReverse(double base, double target, double current)
    {
        return base * (1-(current/target));
    }
    
}
