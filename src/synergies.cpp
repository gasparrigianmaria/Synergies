/* Code provided by Gian Maria Gasparri Ph.D. Centro E. Piaggio 09/07/2017
 * 
 * This code allows the robot to transit from one speed to another one. 
 *
 * In the specific, trajectory references are returned by a synergy mapping function
 * which takes the speed and foot height desired values as inputs. 
 * A service allows the user to modify the desired speed. 
 * (IN FUTURE this service will be activated by a speed control.)
 * 
 * Once the desired speed is modified the new trajectories are provided. 
 * Hence previous signals and newest ones has to be coherently concatenated to provide
 * a transient as smooth as possible. The transition occurs  when the norm position error is minimum. 
 * (FUTURE WORK will consider also the speed, i.e. the robot state.)
 * 
 * 
 * The code can be shared in several sections
 * 1 - INIT - Create publisher message and service in order to provide signlas and to modify them
 *     when necessary.
 * 2 - INIT - Create matrix and vector support.
 * 3 - TRAJ - Obtain interpolated trajectories exploiting a synergy mapping function. This function
 *     returns resampled data as function of the publishing rate.
 * 4 - TRANS - Find the best transition instant.
 * 5 - TRANS - Wait for it and then switch.
 * 6 - PUB - Publish new trajectories
 * 
*/




#include <iostream>
#include <vector>
#include </lib/eigen-eigen/Eigen/Dense>

#include <unistd.h>
#include <string>
#include <array>
#include <math.h>   
#include <boost/concept_check.hpp>
#include <std_msgs/String.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <synergies/traj_state.h>
#include <synergies/spline.h>
#include "synergies/synergy_values.h"
#include <synergies/speed_srv.h>
#include <synergies/synergy_map_fun.h>




const double pi=3.141592;   
double Current_Speed; 


// Service to modify speed   
bool setSpeedCallback(synergies::speed_srv::Request& request, synergies::speed_srv::Response& response)
{
   ROS_INFO("Desired Speed : %lf ", request.speed);
   Current_Speed=request.speed;
 
   return true;
}



int main(int argc, char** argv){
// Initialize  
    ros::init(argc,argv,"Synergies");
    ros::NodeHandle n;
 
//     Set publisher to publish trajectories into "synergies/Trajectories
//      std::cout<<"Publisher"<<std::endl; 
    ros::Publisher pub_traj= n.advertise<synergies::traj_state>("synergies/Trajectories",100);
    std::cout<<"Publisher"<<std::endl; 
//     ros::Subscriber sub_traj=n.subscribe<synergies::joint_state>("synergies/Trajectories",&TrajectoryCallBack());
    
    // Create service to enable speed change by user cmd
    ros::ServiceServer srv_setSpeed = n.advertiseService("synergies/Set_Speed", &setSpeedCallback);
    std::cout<<"Service"<<std::endl; 
    
    
    double speed;
    double foot_h=0.025; //1.5 cm

// Initialize synergies, i.e. the synergies SVD matrices are written in U_mu_1/2 with gloabl visibility
    initialize_synergies();    

    
   Eigen::MatrixXd First_mu=U_mu_1;
   Eigen::MatrixXd First_Synergy=U_1;
   Eigen::MatrixXd Second_mu=U_mu_2;
   Eigen::MatrixXd Second_Synergy=U_2;

   Eigen::MatrixXd New_Traj,Old_Traj,New_Traj_supp;
    
   bool first_time=true; 
    
   synergies::traj_state msg;
//      std::cout<<"message"<<std::endl; 
     
//      std::vector<double> cane(2);
//      cane[0]=1;
//      cane[1]=2;
//     msg.joint_pos=cane;
//     msg.joint_pos(1)=1;
//     msg.joint_pos[2]=2;
    
//  Choose rate 
   
    
    int index=0;
    
    std::vector<double> traj_d(6);
    
    bool flag_pub_traj=true;
    bool flag_concatenate=false;
    
    double Step_old,T_old,Ts_old,des_Ts,curr_Ts;
    double Step_new,T_new,Ts_new;
    
    double exp_val=0.56;
    double l=0.3;
    double g=9.81;
    
    double traj_buffer_size;
    Eigen::VectorXd res_traj;
    std::vector<double> res_traj_2(60);
    int Instant,Current_Instant(0);
    
    des_Ts=0.006;
    
    ros::Rate rate_while=1/des_Ts;
    
    int new_num, old_num, min_num, pub_num;
    
    bool flag_exit_create_concatenate=false;
    
    while(ros::ok()){
      
//     Call Mapping function 
      if(first_time){
	      	// speed is the correct desired speed
		  speed=0.04;
	  Current_Speed=speed;
	  
// 	Step_old=pow(speed/(sqrt(g*l)),exp_val)*l;
// 	T_old=Step_old/speed; //This is the time interval within one step has to be done
// 	Ts_old=T_old/30; // 30 is the number of samples of one step
	
	Step_new=pow(Current_Speed/(sqrt(g*l)),exp_val)*l;
	T_new=Step_new/Current_Speed;
	Ts_new=T_new/30; // 30 is the sample number of one step
	
	curr_Ts=Ts_new;
	
	new_num=std::round(curr_Ts/des_Ts)*60;
	old_num=new_num;
	std::cout<<"new_num "<<new_num<<std::endl;
	std::cout<<"old_num "<<old_num<<std::endl;
// 	  New_Traj=From_Syn_To_Traj(speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu);
	  std::cout<<"des "<<des_Ts<<" curr "<<curr_Ts<<std::endl;
	  New_Traj=From_Syn_To_Traj_Interpolated_Sampled(speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu,des_Ts,curr_Ts);
// 	  Eigen::MatrixXd New_Traj_=From_Syn_To_Traj_Interpolated_Sampled(speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu,des_Ts,curr_Ts);
	  
// 	  for(int i=0;i<6;i++){
// 	    for(int j=0;j<60;j++){
// 	  std::cout<<New_Traj(j,i)-New_Traj_(j,i)<<std::endl;
// 	    }
// 	  }
	  Old_Traj=New_Traj;
	  
	  

	  first_time=false;
	  std::cout<<"Activation minimum speed "<<std::to_string(Current_Speed)<<std::endl;
      }else{
	  if(speed!=Current_Speed && flag_concatenate==false){
	    flag_exit_create_concatenate=true; //you need to concatenate when a change occurs
	          	// speed is the correct desired speed
// 	Step_old=pow(speed/(sqrt(g*l)),exp_val)*l;
// 	T_old=Step_old/speed; //This is the time interval within one step has to be done
// 	Ts_old=T_old/30; // 30 is the number of samples of one step
	
	Step_new=pow(Current_Speed/(sqrt(g*l)),exp_val)*l;
	T_new=Step_new/Current_Speed;
	Ts_new=T_new/30; // 30 is the sample number of one step
	
	curr_Ts=Ts_new;
	
	old_num=new_num;
	new_num=std::round(curr_Ts/des_Ts)*60;    
	    
	    std::cout<<"Updating speed from "<<std::to_string(speed)<<" to "<<std::to_string(Current_Speed)<<std::endl;

	    Old_Traj=New_Traj;
// 	    New_Traj=From_Syn_To_Traj(Current_Speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu);
	    New_Traj=From_Syn_To_Traj_Interpolated_Sampled(Current_Speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu,des_Ts,curr_Ts);
	    
// 	    	    	for(int i=0;i<1;i++){
// 	std::cout<<"OLD "<<Old_Traj.row(i)<<std::endl;  
// 
// 	std::cout<<"NEW "<<New_Traj.row(i)<<std::endl;  
// 			}
	    
	    flag_concatenate=true;
	        
// 	    pub_traj.publish(msg);
	  }
      }
      
      

	
          
      
//       If you have to concatenate do it, otherwise repeat the previous cycle
      
      if(flag_concatenate && flag_exit_create_concatenate){
	

	// find the sample time at which the error of the norm of the robot state is minimum
	if(old_num<new_num){
	  min_num=old_num;
	}else{
	  min_num=new_num;
	}
	std::cout<<"old_num "<<old_num<<std::endl;
	std::cout<<"new_num"<<new_num<<std::endl;
	std::cout<<"before for min_num "<<min_num<<std::endl;
	std::cout<<"Old Trj dimensions "<<sizeof(Old_Traj)<<std::endl;
	res_traj.resize(min_num);
	for(int i=0;i<min_num;i++){
// 	res_traj_2[i]=(Old_Traj.row(i)-New_Traj.row(i)).norm(); //vector double
	res_traj(i)=(Old_Traj.row(i)-New_Traj.row(i)).norm(); //vectorXd
// 	std::cout<<std::to_string(res_traj(i))<<std::endl;
	}
	std::cout<<"after for min_num"<<std::endl;
// 	std::cout<<"val min "<<res_traj.minCoeff()<<std::endl;
// 	Find the minimum
// 	std::ptrdiff_t i_g, j_g;
	Eigen::Index i_g, j_g;
	double val_1=res_traj.minCoeff(&i_g,&j_g); // This is the value of the error
	Instant=i_g; // This is the instant at which you should switch the signals
		
	std::cout<<"Instant "<<i_g<<" value "<<val_1<<" "<<res_traj(i_g)<<std::endl;
	
	//If you passed the perfect instant you have another chance till the err is less than 2 times the first error
// // 	if(Current_Instant>Instant){
// // 	std::cout<<"inside if current instant"<<std::endl;
// // 	  	Eigen::Index i_g_2, j_g_2;
// // 		/*res_traj.tail(60-Current_Instant)*/;
// // 	double val_2=(res_traj.tail(min_num-Current_Instant)).minCoeff(&i_g_2,&j_g_2);
// // 
// // 	    if(val_2<=2*val_1){	
// // 	    std::cout<<"Current Instant "<<Current_Instant<<"New  Instant "<<Current_Instant+i_g_2<<" value "<<val_2<<" "<<res_traj(Current_Instant+i_g_2)<<std::endl;
// // 	    Instant=Current_Instant+i_g_2;}
// // 	}
	
std::cout<<"before for concatenate trajectories"<<std::endl;
	  // Concatenate matrices in New Traj supp
	New_Traj_supp.resize(new_num,6);
	  for(int i=0;i<new_num;i++){
	    if(i<Instant){
	      std::cout<<"i<Instant"<<std::endl;
	      std::cout<<"Old "<<Old_Traj.row(i)<<std::endl;
	      std::cout<<"New "<< New_Traj_supp.row(i)<<std::endl;
	      
	    New_Traj_supp.row(i)=Old_Traj.row(i);
	      }
	    else{
	      std::cout<<"else "<<std::endl;
	    New_Traj_supp.row(i)=New_Traj.row(i);
	    }
	    
	  }
	  std::cout<<"after for concatenate trajectories"<<std::endl;
	flag_exit_create_concatenate=false;
      } // end if concatenate
      
 // WE HAVE TO CONSIDER THAT WE'RE GOING TO SEND INTERPOLATED SIGNALS, YOU CAN WAIT 2 STEPS OR SWITCH THEM AS SOON AS POSSIBLE
 // HERE IN THE FOLLOWING I FIND THE INSTANT AND COMPOSE THE NEW TRAJ. IT IS NECESSARY TO CONSIDER INTERPOLATION 
 // BETWEEN OLD TRAJ E NEW TRAJ.
 
     if(flag_pub_traj){
       
       if(flag_concatenate==true){pub_num=old_num;}else{pub_num=new_num;}
       
	if(index==pub_num) index=0;
	
	for(int i=0;i<6;i++){
	  // if you need to concatenate
	  
	  
	    if((Current_Instant==Instant)&&(flag_concatenate==true)){
	     speed=Current_Speed;
	     flag_concatenate=false; 
	    }
	    
	    if (flag_concatenate==true){
	      traj_d[i]=Old_Traj(index,i);
	    }else{
	      traj_d[i]=New_Traj(index,i);
	    }
	    
	} //end for i 
	  
// // 	  if(flag_concatenate){
// // 	    
// // 	    
// // 	    if(Current_Instant>Instant){
// // 	      traj_d[i]=Old_Traj(index,i);
// // 	      }
// // 	    else{
// // 	      traj_d[i]=New_Traj_supp(index,i);
// // 	      if(index==pub_num-1 && i==5){
// // 		speed=Current_Speed;
// // 		flag_concatenate=false;
// // 		// there is no need to concatenate furthermore
// // 		// From the last instant the 
// // 	      }
// // 	    }
// // 
// // 	  }else{//if you do not need to concatenate just publish the New_Traj
// // 	  traj_d[i]=New_Traj(index,i);
// // 	  }
//        std::cout<<std::to_string(New_Traj(index,i))<<std::endl;
	
//     std::cout<<std::to_string(New_Traj(0,0))<<std::endl;
	msg.joint_pos=traj_d;  
	pub_traj.publish(msg);
	} // end if pub flag_pub_traj
	Current_Instant=index;
// 	std::cout<<Current_Instant<<std::endl;
	index+=1;

       
      
    
    ros::spinOnce();
    rate_while.sleep();
    } //end while

return 0;
  
}