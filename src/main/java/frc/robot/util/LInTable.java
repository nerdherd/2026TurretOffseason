package frc.robot.util;

public class LInTable {
    private final double[][] table;
    private final double[][] mTable;

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
            previousNumber = item[0];
        }

        this.table = table;
        this.boundBehavior = boundBehavior;

        double[][] mTable = new double[table.length - 1][table[0].length - 1];
        for (int i = 1; i < table.length; i++){
            for (int yIndex = 0; yIndex < table[0].length - 1; yIndex++){
                mTable[i-1][yIndex] = (table[i-1][yIndex+1]-table[i][yIndex+1]) / (table[i-1][0]-table[i][0]);
            }
        }

        this.mTable = mTable;
    } 

    public double[] interpolate(double x, int[] yIndex){
        double[] result = new double[yIndex.length];
        
        if (table.length == 1){
            for (int i = 0; i < yIndex.length; i++){
                result[i] = table[0][yIndex[i]];
            }
            return result;
        }

        if (x < table[0][0]){
            switch (boundBehavior) {
                case EXCEPTION:
                    throw new IllegalArgumentException("X too low");
                case HOLD:
                    for (int i = 0; i < yIndex.length; i++){
                        result[i] = table[0][yIndex[i]];
                    }
                    return result;
                case LINEAR:
                    for (int j = 0; j < yIndex.length; j++){
                        double m = mTable[0][yIndex[j]-1];
                        result[j] = m * (x-table[0][0]) + table[0][yIndex[j]];
                    }
                    return result;
            }
        }

        if (x > table[table.length - 1][0]){
            switch (boundBehavior) {
                case EXCEPTION:
                    throw new IllegalArgumentException("X too high");
                case HOLD:
                    for (int i = 0; i < yIndex.length; i++){
                        result[i] = table[table.length - 1][yIndex[i]];
                    }
                    return result;
                case LINEAR:
                    for (int j = 0; j < yIndex.length; j++){
                        double m = mTable[mTable.length-1][yIndex[j]-1];
                        result[j] = m * (x-table[table.length - 1][0]) + table[table.length - 1][yIndex[j]];
                    }
                    return result;
            }
        }

        for (int i = 1; i < table.length; i++){
            if (x <= table[i][0]){
                for (int j = 0; j < yIndex.length; j++){
                    double m = mTable[i-1][yIndex[j]-1];
                    result[j] = m * (x-table[i][0]) + table[i][yIndex[j]];
                }
                return result;
            }
        }

        throw new IllegalArgumentException("Somehow guardrails failed.");
    }

    public double[][] getTable(){
        double[][] copy = new double[table.length][table[0].length];
        for (int i = 0; i < table.length; i++){
            copy[i] = table[i].clone();
        }
        return copy;
    }
}
