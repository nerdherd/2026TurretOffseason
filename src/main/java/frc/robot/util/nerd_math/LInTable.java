package frc.robot.util.nerd_math;

import frc.robot.util.nerd_logging.NerdLog;

/**
 * A table of points to be linearly interpolated between
 * Basically: a straight line is drawn between every point, and this exposes a simple function to sample those lines
 */
public class LInTable {
    // Helper types
    private record Point(double x, double y, double dy) {}

    /**
     * how the table should behave before the first point or after the last point
     */
    public static enum BoundBehavior {
        /**
         * Throw an exception
         */
        EXCEPTION, 
        /**
         * Keep the same value as the boundary points (flattens off at the start and end)
         */
        HOLD, 
        /**
         * Continue the lines past the bounds
         */
        LINEAR
    }

    // Variables

    private final BoundBehavior boundBehavior;

    private Point[] table;

    private boolean warnedEmpty = false;

    // Initializers

    /**
     * Creates a LInTable
     * @param x an array of x values
     * @param y an array of y values
     * @param boundBehavior how the table should behave before the first point or after the last point (see BoundBehavior)
     */
    public LInTable(double[] x, double[] y, BoundBehavior boundBehavior) {
        if (boundBehavior == null)
            throw new IllegalArgumentException("bound behavior must not be null");
        if (x == null)
            throw new IllegalArgumentException("x array must not be null");
        if (y == null)
            throw new IllegalArgumentException("y array must not be null");

        if (x.length != y.length)
            throw new IllegalArgumentException("x and y arrays must have the same size");

        this.boundBehavior = boundBehavior;

        table = new Point[x.length];
        if (x.length == 0) return;

        double previous = Double.NEGATIVE_INFINITY;
        for (double xval : x) {
            if (xval <= previous) throw new IllegalArgumentException("X values must be in order and not repeat");
            if (!Double.isFinite(xval)) throw new IllegalArgumentException("X values must be valid doubles");
            previous = xval;
        }
        for (double yval : y)
            if (!Double.isFinite(yval))
                throw new IllegalArgumentException("Y values must be valid doubles");

        for (int i = 0; i < x.length; i++){
            if (i==0) table[i] = new Point(x[i], y[i], Double.NaN);
            else table[i] = new Point(x[i], y[i],
                (y[i-1]-y[i])/(x[i-1]-x[i])
            );
        }
    }

    /**
     * Creates a LInTable, defaulting to boundBehavior = {@link BoundBehavior#LINEAR}
     * @param x an array of x values
     * @param y an array of y values
     */
    public LInTable(double[] x, double[] y) {
        this(x, y, BoundBehavior.LINEAR);
    }

    // Interpolate

    /**
     * Interpolates the table
     * @param x the x value to sample at
     * @return the y value at that x
     */
    public double interpolate(double x) {
        if (Double.isNaN(x)) throw new IllegalArgumentException("X must not be NaN");
        if (table.length == 0) {
            if (!warnedEmpty) {
                NerdLog.reportWarning("LInTable has no values, returning 0");
                warnedEmpty = true;
            }
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
                default -> {
                    throw new IllegalArgumentException("Unhandled LInTable BoundBehavior");
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
                default -> {
                    throw new IllegalArgumentException("Unhandled LInTable BoundBehavior");
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
