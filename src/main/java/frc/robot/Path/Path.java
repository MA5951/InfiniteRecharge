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
      new double[]{0, 90, 0.1, 60, 1, 1},
      new double[]{0, 110, 0.05, 5, 1, 1},
      new double[]{1.8, 110, 0.3, 5, 1 , 1},
      new double[]{1.8, 180, 0.3, 4 , 1 , 1},
      new double[]{6.3, 180, 0.3, 5, 0.6 , 1},
      new double[]{6.3, -20, 0.3, 4, 0.6 , 1},
      new double[]{11, -20 , 0.3, 5, 0.6 , 1},
      
      };

      public static double[][] enemyRoultte = {
        new double[]{3, 0, 0.3, 5, 1 , 1},
        new double[]{0, 20, 0.3, 4, 0.6 , 1},
        new double[]{0, 180, 0.3, 4, 0.6 , 1},
      };

      public static double[][] toRondevousAndBack = { 
        new double[]{-4, 0, 0.2, 10, 1, 1},
        new double[]{-4, -21, 0.3, 8, 1, 1},
        new double[]{-2.8, -21, 0.15, 7, 1, 1},
        new double[]{-2.8, 16, 0.3, 8, 1, 1},
        new double[]{-4, 16, 0.1, 7, 1, 1},
        new double[]{-4, -25, 0.3, 5, 1, 1},
         new double[]{-0.5, -25, 0.05, 7, 1, 1},


      };

}
