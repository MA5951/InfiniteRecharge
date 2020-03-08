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

    public static double[][] roulettePath = { 
      new double[]{0, 90, 0.1, 60, 1, 1},
      new double[]{0, 110, 0.05, 7, 1, 1},
      new double[]{1.9, 110, 0.3, 5, 1 , 1},
      new double[]{1.9, 180, 0.3, 6 , 1 , 1},
      new double[]{6.6, 180, 0.3, 5, 0.8 , 1},
      new double[]{6.6, -90, 0.3, 60, 0.6 , 1},
      new double[]{6.6, -20, 0.3, 5, 1 , 1},
      new double[]{10, -20, 0.3, 5, 1 , 1},
      };

      
    public static double[][] roulettePath1 = { 
      new double[]{1, 70, 0.1, 5, 1, 1},
      };

      public static double[][] roulettePath2 = { 
        new double[]{1, 0, 0.3, 60, 0.8,1},
        new double[]{1, -110, 0.3, 5, 0.8,1},
        new double[]{1.5, -110, 0.3, 5, 0.8,1},
        new double[]{6.7, -90, 0.1, 5, 0.7, 1},
        new double[]{6.7, -0, 0.1, 60, 0.7, 1},
        new double[]{6.7, 70, 0.1, 5, 1, 1},
        new double[]{10, 70, 0.1, 5, 1, 1},
        };



      public static double[][] standart = { 
        new double[]{0.2, 0, 0.05, 5, 1 , 1},
        };

        public static double[][] standart1 = { 
          new double[]{-1, 0, 0.05, 5, 1 , 1},
          };
        
      public static double[][] enemyRoultte = { 
        new double[]{3.1, 0, 0.3, 5, 0.7 , 1},
        new double[]{2.9, 20, 0.3, 5, 1 , 1},
        new double[]{-1.6, 60, 0.3, 4, 1 , 1},
        new double[]{-1.6, -145, 0.3, 4, 1 , 1},
      };


}
