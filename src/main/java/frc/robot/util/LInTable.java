package frc.robot.util;

public class LInTable {
    private final double[][] table;

    public static enum BoundBehavior {
        EXCEPTION, HOLD, LINEAR
    }

    private final BoundBehavior boundBehavior;

    public LInTable(double[][] table, BoundBehavior boundBehavior){
        if (table.length == 0){
            throw new IllegalArgumentException("Table is too small.");
        }

        double previousNumber = table[0][0] - 1;
        for (double[] item : table){
            if (item[0] <= previousNumber){
                throw new IllegalArgumentException("X values must be in order and not repeat.");
            }
        }

        this.table = table;
        this.boundBehavior = boundBehavior;
    }

    public double interpolate(double x, int yIndex){
        if (table.length == 1) return table[0][yIndex];

        if (x < table[0][0]){
            switch (boundBehavior) {
                case EXCEPTION:
                    throw new IllegalArgumentException("X too low");
                case HOLD:
                    return table[0][yIndex];
                case LINEAR:
                    double m = (table[0][yIndex]-table[1][yIndex]) / (table[0][0]-table[1][0]);
                    return m * (x-table[0][0]) + table[0][yIndex];
            }
        }

        if (x > table[table.length - 1][0]){
            switch (boundBehavior) {
                case EXCEPTION:
                    throw new IllegalArgumentException("X too high");
                case HOLD:
                    return table[table.length - 1][yIndex];
                case LINEAR:
                    double m = (table[table.length - 2][yIndex]-table[table.length - 1][yIndex]) / (table[table.length - 2][0]-table[table.length - 1][0]);
                    return m * (x-table[table.length - 2][0]) + table[table.length - 2][yIndex];
            }
        }

        for (int i = 1; i < table.length; i++){
            if (x < table[i][0]){
                double m = (table[i][yIndex]-table[i-1][yIndex]) / (table[i][0]-table[i-1][0]);
                return m * (x-table[i][0]) + table[i][yIndex];
            }
        }
        return 0;
    }

    public double[][] getTable(){
        double[][] copy = new double[table.length][table[0].length];
        for (int i = 0; i < table.length; i++){
            copy[i] = table[i].clone();
        }
        return copy;
    }
}
