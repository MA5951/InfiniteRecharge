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
      new double[]{0, 110, 0.05, 6, 1, 1},
      new double[]{1.8, 110, 0.3, 5, 1 , 1},
      new double[]{1.8, 180, 0.3, 6 , 1 , 1},
      new double[]{6.4, 180, 0.3, 5, 0.6 , 1},
      new double[]{6.4, -90, 0.3, 60, 0.6 , 1},
      new double[]{6.4, 0, 0.3, 5, 0.6 , 1},
      new double[]{7.5, 0, 0.3, 5, 1 , 1},
      
      };

      public static double[][] standart = { 
        new double[]{-1, 0, 0.05, 5, 1 , 1},
       
        };
        
      public static double[][] enemyRoultte = {
        new double[]{2.6, 0, 0.3, 5, 1 , 1},
        new double[]{-1.4, 60, 0.3, 4, 1 , 1},
        new double[]{-1.4, -145, 0.3, 4, 1 , 1},
      };
/*
      public static double[][] toRondevousAndBack = { 
        new double[]{0, -20, 0.3, 5, 1 , 1},
        new double[]{0, -90, 0.1, 60, 1, 1},
        new double[]{0, -180, 0.05, 5, 1, 1},
        new double[]{4.6, -180, 0.3, 5, 0.9 , 1},
        new double[]{4.3, -90, 0.3, 60, 1 , 1},
        new double[]{4.3, -10, 0.3, 5 , 1 , 1},
        new double[]{6.4, -10, 0.3, 5, 0.6 , 1},

      };
*/

}
