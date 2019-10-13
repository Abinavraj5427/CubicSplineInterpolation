 class Spline{
    private double x1, y1, dx1, dy1;
    private double x2, y2, dx2, dy2;
    private double ax, bx, cx, dx;
    private double ay, by, cy, dy;
    private double t1 = 0, t2 = 1;

    public Spline(double x1, double y1, double theta1, double x2, double y2, double theta2){
        this.x1 = x1;
        this.y1 = y1;
        this.dx1 = Math.cos(theta1);
        this.dy1 = Math.sin(theta1);
        this.x2 = x2;
        this.y2 = y2;
        this.dx2 = Math.cos(theta2);
        this.dy2 = Math.sin(theta2);
        calculateCoef();
    }

    public void calculateCoef(){
        double[][] matrix1 = 
        {
            //original matrix 1
            {3*t1*t1, 6*t1, 1, 0},
            {t1*t1*t1, t1*t1, t1, 1},
            {t2*t2*t2, t2*t2, t2, 1},
            {3*t2*t2, 6*t2, 1, 0},
        };
        double[][] imatrix1 = invert(matrix1);
        double[][] matrix2 = 
        {
            {dx1, dy1},
            {x1, y1},
            {x2, y2},
            {dx2, dy2},
        };

        double[][] coeff = multiplyMatrices(imatrix1, matrix2, 4,4,2);
        this.ax = coeff[0][0];
        this.ay = coeff[0][1];
        this.bx = coeff[1][0];
        this.by = coeff[1][1];
        this.cx = coeff[2][0];
        this.cy = coeff[2][1];
        this.dx = coeff[3][0];
        this.dy = coeff[3][1];
    }


    //matrix multiplication (dot product)
    public static double[][] multiplyMatrices(double[][] firstMatrix, double[][] secondMatrix, int r1, int c1, int c2) {
        double[][] product = new double[r1][c2];
        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c2; j++) {
                for (int k = 0; k < c1; k++) {
                    product[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
        return product;
    }

    public static double[][] invert(double a[][]) 
    {
        int n = a.length;
        double x[][] = new double[n][n];
        double b[][] = new double[n][n];
        int index[] = new int[n];
        for (int i=0; i<n; ++i) 
            b[i][i] = 1;
 
 // Transform the matrix into an upper triangle
        gaussian(a, index);
 
 // Update the matrix b[i][j] with the ratios stored
        for (int i=0; i<n-1; ++i)
            for (int j=i+1; j<n; ++j)
                for (int k=0; k<n; ++k)
                    b[index[j]][k]
                    	    -= a[index[j]][i]*b[index[i]][k];
 
 // Perform backward substitutions
        for (int i=0; i<n; ++i) 
        {
            x[n-1][i] = b[index[n-1]][i]/a[index[n-1]][n-1];
            for (int j=n-2; j>=0; --j) 
            {
                x[j][i] = b[index[j]][i];
                for (int k=j+1; k<n; ++k) 
                {
                    x[j][i] -= a[index[j]][k]*x[k][i];
                }
                x[j][i] /= a[index[j]][j];
            }
        }
        return x;
    }
 
// Method to carry out the partial-pivoting Gaussian
// elimination.  Here index[] stores pivoting order.
 
    public static void gaussian(double a[][], int index[]) 
    {
        int n = index.length;
        double c[] = new double[n];
 
 // Initialize the index
        for (int i=0; i<n; ++i) 
            index[i] = i;
 
 // Find the rescaling factors, one from each row
        for (int i=0; i<n; ++i) 
        {
            double c1 = 0;
            for (int j=0; j<n; ++j) 
            {
                double c0 = Math.abs(a[i][j]);
                if (c0 > c1) c1 = c0;
            }
            c[i] = c1;
        }
 
 // Search the pivoting element from each column
        int k = 0;
        for (int j=0; j<n-1; ++j) 
        {
            double pi1 = 0;
            for (int i=j; i<n; ++i) 
            {
                double pi0 = Math.abs(a[index[i]][j]);
                pi0 /= c[index[i]];
                if (pi0 > pi1) 
                {
                    pi1 = pi0;
                    k = i;
                }
            }
 
   // Interchange rows according to the pivoting order
            int itmp = index[j];
            index[j] = index[k];
            index[k] = itmp;
            for (int i=j+1; i<n; ++i) 	
            {
                double pj = a[index[i]][j]/a[index[j]][j];
 
 // Record pivoting ratios below the diagonal
                a[index[i]][j] = pj;
 
 // Modify other elements accordingly
                for (int l=j+1; l<n; ++l)
                    a[index[i]][l] -= pj*a[index[j]][l];
            }
        }
    }

    public void printCoef(){
        System.out.printf("%.3f %.3f %.3f %.3f\n", ax, bx, cx, dx);
        System.out.printf("%.3f %.3f %.3f %.3f\n", ay, by, cy, dy);
    }

    public double[] getPoint(double t){
        double x = ax*Math.pow(t,3)+bx*Math.pow(t,2)+cx*Math.pow(t,1)+dx;
        double y = ay*Math.pow(t,3)+by*Math.pow(t,2)+cy*Math.pow(t,1)+dy;
        return new double[]{x,y};
    }

    public double getAngle(double t){
        double dx = 3*ax*Math.pow(t,2)+2*bx*Math.pow(t,1)+cx;
        double dy = 3*ay*Math.pow(t,2)+2*by*Math.pow(t,1)+cy;
        return Math.atan(dy/dx);
    }


}

public class CubicSpline{
    public static void main(String[] args){
        //System.out.print("A");
        Spline cs = new Spline(0,0,Math.toRadians(30),0,1,Math.toRadians(60));
        cs.printCoef();
        System.out.println(cs.getPoint(1)[0] +", "+ cs.getPoint(1)[1]);
        System.out.println(cs.getAngle(1));

    }
}