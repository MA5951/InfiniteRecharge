/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Path;

/**
 * Add your docs here.
 */
public class Path {
    public static double[][] mainPath; // the array we us as the main Path in the MApath

    public static double[][] roulettePath1 = { 
      new double[]{0.87, 4.5, 0.3, 15, 1 , 1},
      new double[]{1.390, -32.28, 0.3, 15, 1 , 1},
      new double[]{1.900, -67.46, 0.3, 15, 1 , 1},
      new double[]{2.400, -77.49, 0.3, 15, 1 , 1},
      new double[]{2.820, -62.85, 0.3, 15, 1 , 1},
      new double[]{3.270, -11, 0.3, 15, 1 , 1},
      new double[]{3.6, 0, 0.3, 2, 1 , 1},
      new double[]{7, 0, 0.05, 3, 1, 1}
      };

      public static double[][] toRondevousAndBack = { 
        new double[]{-4, 0, 0.2, 10, 1, 1},
        new double[]{-4, -22, 0.3, 5, 1, 1},
        new double[]{-2.8, -22, 0.2, 10, 1, 1},
        new double[]{-2.8, 16, 0.3, 5, 1, 1},
        new double[]{-4, 16, 0.1, 10, 1, 1},
        new double[]{-4, -42, 0.3, 5, 1, 1},
         new double[]{-0.3, -42, 0.1, 10, 1, 1},


      };

          public static double[][] try_path = {
            new double[]{0.8 , -5, 0.3, 10, 0.2, 1},
            new double[]{1.35, -32.54,0.3, 10, 0.25 , 0.65 },
            new double[]{1.84, -68.24,0.3, 10,0.25 , 0.65 },
            new double[]{2.33, -78.06,0.3, 10,0.25 , 0.65 },
            new double[]{2.72, -62.32,0.3, 10,0.25 , 0.65 },
            new double[]{3.16, 0,0.1, 2 ,0.2 , 0.4 },
            new double[]{4.00, 0,0.3, 2 ,0.2 , 0.4 },
            new double[]{6.9, 0, 0.1, 2, 0.2, 0.45},
            new double[]{3.18, 0.0, 0.3, 5, 0.2, 0.4},
            new double[]{2.21, -4.2, 0.3, 5, 0.25 , 0.65},
            new double[]{1.58, -35.82, 0.3, 5, 0.25 , 0.65},
            new double[]{1.04, -65.12, 0.3, 5, 0.25 , 0.65},
            new double[]{0.570, -74.53, 0.3, 5, 0.25 , 0.65},
            new double[]{0.1, -52.27, 0.3, 2, 0.25 , 0.65},
            new double[]{-0.1, -25.27, 0.3, 2, 0.25 , 0.65},
            new double[]{-0.5, 0, 0.3, 2, 0.2, 0.4},
            new double[]{-3, 0, 0.05, 2, 0.2, 0.4}
 
              };

            
              

}

 
