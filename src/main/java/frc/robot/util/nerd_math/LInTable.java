package frc.robot.util.nerd_math;

public class LInTable {
    // Helper types
    private record Point(double x, double y, double dy) {}

    public static enum BoundBehavior {
        EXCEPTION, HOLD, LINEAR
    }

    // Variables

    private final BoundBehavior boundBehavior;

    private Point[] table;

    // Initializers

    public LInTable(double[] x, double[] y, BoundBehavior boundBehavior) {
        if (x.length != y.length) {
            throw new IllegalArgumentException("x and y tables must have the same size");
        }
        this.boundBehavior = boundBehavior;

        double previous = x[0] - 1;
        for (double xval : x) {
            if (xval <= previous) throw new IllegalArgumentException("X values must be in order and not repeat.");
            previous = xval;
        }

        table = new Point[x.length];

        for (int i = 0; i < x.length; i++){
            if (i==0) table[i] = new Point(x[i], y[i], Double.NaN);
            else table[i] = new Point(x[i], y[i],
                (y[i-1]-y[i])/(x[i-1]-x[i])
            );
        }
    }

    public LInTable(double[] x, double[] y) {
        this(x, y, BoundBehavior.LINEAR);
    }

    // Interpolate

    public double interpolate(double x) {
        if (table.length == 0) {
            // TODO: log this probably
            return 0;
        }
        if (table.length == 1){
            return table[0].y();
        }
        if (x < table[0].x()) {
            switch (boundBehavior) {
                case EXCEPTION -> throw new IllegalArgumentException("X too low");
                case HOLD -> {
                    return table[0].y();
                }
                case LINEAR -> {
                    double m = table[1].dy();
                    return m * (x-table[0].x()) + table[0].y();
                }
            }
        }
        if (x > table[table.length - 1].x()){
            switch (boundBehavior) {
                case EXCEPTION -> throw new IllegalArgumentException("X too high");
                case HOLD -> {
                    return table[table.length-1].y();
                }
                case LINEAR -> {
                    double m = table[table.length-1].dy();
                    return m * (x-table[table.length - 1].x()) + table[table.length - 1].y();
                }
            }
        } else if (x == table[table.length - 1].x()) 
            return table[table.length - 1].y();

        int left = 0;
        int right = table.length - 1;
        int resultIndex = -1;

        while (left <= right){
            int mid = (left + right) / 2;

            double midX = table[mid].x();

            if (x < midX){
                right = mid - 1;
                resultIndex = mid;
            } else {
                left = mid + 1;
            }
        }

        if (resultIndex == -1) throw new IllegalArgumentException("Somehow failed to find x.");

        double m = table[resultIndex].dy();
        return m * (x-table[resultIndex].x()) + table[resultIndex].y();
    }
}
