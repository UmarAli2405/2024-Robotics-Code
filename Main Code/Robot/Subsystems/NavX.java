// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// // Umar and Usman

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.SerialPort;


// public class NavX extends SubsystemBase {
  
//     private static NavX instance = null;
  
//     public static AHRS ahrs;


//   public NavX() { 
//     super();
//     ahrs = new AHRS(SerialPort.Port.kUSB);
//   }


//   private static NavX getInstance(){
//     if (instance == null){
//         System.out.println("NavX init");
//         instance = new NavX();
//     }
//     return instance;

//   }

  


  


//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }