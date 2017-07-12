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

//--------------------------INIT----------------------------
  
// Initialize  
    ros::init(argc,argv,"Synergies");
    ros::NodeHandle n;
 
//     Set publisher to publish trajectories into "synergies/Trajectories
    ros::Publisher pub_traj= n.advertise<synergies::traj_state>("synergies/Trajectories",100);
    std::cout<<"Publisher"<<std::endl; 
//     ros::Subscriber sub_traj=n.subscribe<synergies::joint_state>("synergies/Trajectories",&TrajectoryCallBack());
    
    // Create service to enable speed change by user cmd
    ros::ServiceServer srv_setSpeed = n.advertiseService("synergies/Set_Speed", &setSpeedCallback);
    std::cout<<"Service"<<std::endl; 
    
    
    double speed;
    double foot_h=0.015; //1.5 cm

// Initialize synergies, i.e. the synergies SVD matrices are written in U_mu_1/2 with gloabl visibility
    initialize_synergies();    

    
   Eigen::MatrixXd First_mu=U_mu_1;
   Eigen::MatrixXd First_Synergy=U_1;
   Eigen::MatrixXd Second_mu=U_mu_2;
   Eigen::MatrixXd Second_Synergy=U_2;

   Eigen::MatrixXd New_Traj,Old_Traj,New_Traj_supp;
    
   bool first_time=true; 
    
   synergies::traj_state msg;

//  Choose rate 
   
    
    int index=0;
    
    std::vector<double> traj_d(6);
    
    bool flag_pub_traj=true;
    bool flag_concatenate_started=false;
    
    double Step_old,T_old,Ts_old,pub_Ts,curr_Ts;
    double Step_new,T_new,Ts_new;
    
    double exp_val=0.56;
    double l=0.3;
    double g=9.81;
    
    double traj_buffer_size;
    Eigen::VectorXd res_traj;
    std::vector<double> res_traj_2(60);
    int Instant(0),Current_Instant(0);
    
    pub_Ts=0.006;
    
    ros::Rate rate_while=1/pub_Ts;
    
    int new_num, old_num, min_num, pub_num;
    
    bool flag_concatenate_completed=true;
    bool one_time=true;
    
    tk::spline new_s;
    tk::spline old_s;
    tk::spline trans_s; //transition
    
    Eigen::MatrixXd New_Traj_pub;
    Eigen::MatrixXd Old_Traj_pub;
    Eigen::MatrixXd Middle_Traj_pub;
    Eigen::MatrixXd Output_Traj;
    Eigen::MatrixXd Err_Traj;
    
//------------------------END INIT----------------------------    
    
    
    while(ros::ok()){
      
//--------------------------TRAJ---------------------------      
      
//     Call Mapping function 
      if(first_time){
	  // speed is the correct desired speed
	  speed=0.04;
	  Current_Speed=speed;
	  
	  Step_new=pow(Current_Speed/(sqrt(g*l)),exp_val)*l;
	  T_new=Step_new/Current_Speed;
// 	  std::cout<<"T_new "<<T_new<<std::endl;
	  Ts_new=T_new/30; // 30 is the sample number of one step (!)
	  
// 	  Current Sample Time
	  curr_Ts=Ts_new;
// 	  New data dimensions as function of speed and Ts
	  new_num=std::round(T_new/pub_Ts)*2; //because we have two steps in the analyzed trajectories (!)
	  old_num=new_num;
	  std::cout<<"new_num "<<new_num<<std::endl;
	  std::cout<<"old_num "<<old_num<<std::endl;
// 	  New_Traj=From_Syn_To_Traj(speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu);
	  std::cout<<"des "<<pub_Ts<<" curr "<<curr_Ts<<std::endl;
// 	  New_Traj=From_Syn_To_Traj_Interpolated_Sampled_TEST(speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu,new_num,curr_Ts,pub_Ts);
	  New_Traj=From_Syn_To_Traj( speed, foot_h, First_Synergy, Second_Synergy,First_mu,Second_mu);
	  Old_Traj=New_Traj;
	      
	  first_time=false;
	  std::cout<<"Activation minimum speed "<<std::to_string(Current_Speed)<<std::endl;
	  	  
	  New_Traj_pub=Interpolate_and_Resample(New_Traj, Ts_new, pub_Ts);
	  Old_Traj_pub=New_Traj_pub;
	  std::cout<<"New Traj size "<<New_Traj_pub.rows()<<" "<<New_Traj_pub.rows()<<std::endl;
	  std::cout<<"Old Traj size "<<Old_Traj_pub.rows()<<" "<<Old_Traj_pub.rows()<<std::endl;
// 	  pub_num=Old_Traj_pub.rows();
// 	  std::cout<<pub_num<<std::endl;
	  
	  
	  	  
      }else{
	// if it is not the first time and speed has not changed nothing have to be done
	// Otherwise if speed is modified we have to:
	// 1 - Create two batches the first composed of the old and new signals the second just of the new ones
	// 2 - If it is too late continue to publish the old one, then publish the mixed batch and hence the new one.
	// the second step could take awhile hence if concatenate operation is started you do not have to modify stuff
	if(speed!=Current_Speed && one_time==true){
// 	if(speed!=Current_Speed && flag_concatenate_started==false){
	  
	  //a change occurs hence you need to concatenate
	      Step_old=Step_new;
	      T_old=T_new; //This is the time interval within one step has to be done
// 	      std::cout<<"T_old "<<T_old<<std::endl;
	      Ts_old=Ts_new; // 30 is the number of samples of one step
	      
	      Step_new=pow(Current_Speed/(sqrt(g*l)),exp_val)*l;
	      T_new=Step_new/Current_Speed;
	      Ts_new=T_new/30; // 30 is the sample number of one step
	      
	      Old_Traj=New_Traj;
	      New_Traj=From_Syn_To_Traj(Current_Speed, foot_h, First_Synergy, Second_Synergy, First_mu, Second_mu);
	      
// 	      Prepare the signal batches
	      New_Traj_pub=Interpolate_and_Resample(New_Traj, Ts_new, pub_Ts);
	      Old_Traj_pub=Interpolate_and_Resample(Old_Traj, Ts_old, pub_Ts);
	      
// 	      Find when you have to move from old to new signals
	      Eigen::Index i_g, j_g;
/*	      
	      for(int i=0;i<6;i++){
		for(int j=0;j<60;j++){
		  Err_Traj(j,i)=fabs(Old_Traj(j,i)-New_Traj(j,i));
		}
	      }*/
	      
// 	      double val_1=Err_Traj.minCoeff(&i_g,&j_g); // This is the value of the error
// 	      double val_1=(Old_Traj-New_Traj).minCoeff(&i_g,&j_g); // This is the value of the error
	      double val_1;
	      double sum_val(0);
	      std::vector<double> err(6);
	      Eigen::VectorXd ERR(60);
	      for(int j=0;j<60;j++){
		sum_val=0;
		for(int i=0;i<6;i++){
		val_1=(Old_Traj(j,i)-New_Traj(j,i)); // This is the value of the error
// 		std::cout<<"val "<<val_1<<std::endl;
		sum_val+=fabs(val_1);
// 		std::cout<<"err[i] "<<err[i]<<std::endl;
		}
		
// 		ERR(j)= std::accumulate(err.begin(), err.end(), 0);
		ERR(j)= sum_val;
// 		sum_val=0;
// 		std::cout<<"ERR(j) "<<ERR(j)<<std::endl;
		
	      }
// 	      val_1=(Old_Traj-New_Traj).minCoeff(&i_g,&j_g); // This is the value of the error
	      val_1=ERR.minCoeff(&i_g,&j_g); // This is the value of the error

	      Instant=i_g; // This is the instant at which you should switch the signals
	      std::cout<<"val "<<val_1<<std::endl;
	      std::cout<<"Instant "<<i_g<<std::endl;
	      if(Instant==0) Instant=1;
// 	      std::cout<<"New Traj size"<<New_Traj_pub.rows()<<" "<<New_Traj_pub.rows()<<std::endl;
// 	      std::cout<<"Old Traj size"<<Old_Traj_pub.rows()<<" "<<Old_Traj_pub.rows()<<std::endl;
	      
// 	      Create the mixed batch
	      Middle_Traj_pub=Interpolate_and_Resample_Middle(New_Traj, Ts_new, Old_Traj, Ts_old, Instant, pub_Ts);

// 	      Now all the signals have been created hence i have not to enter in this statement again
	      flag_concatenate_started=true;
	      flag_concatenate_completed=false;
	      } //end if speed change occurs
	      
      } //end if else first time and speed change occurs

// // //      if(flag_pub_traj){
       
// // //        if(speed==Current_Speed){
// // // 	 Output_Traj=Old_Traj_pub;
// // // 	 pub_num=Old_Traj_pub.rows();
// // //       }else{
// // // 	    if(flag_concatenate_started==true && index>Instant){
// // // // 	      std::cout<<"too_soon"<<std::endl;
// // // 	      Output_Traj=Old_Traj_pub;
// // // 	      pub_num=Old_Traj_pub.rows();
// // // 	      //need to wait the next step, i.e. continue with current Output_Traj
// // // 	    }else{
// // // // 	      std::cout<<"now_ok"<<std::endl;
// // // 	      Output_Traj=Middle_Traj_pub;
// // // 	      pub_num=Middle_Traj_pub.rows();
// // // 	      //While index < Instant pub_num is the same
// // // // 	      flag_concatenate_completed=false;
// // // 	      	      
// // // 	    }
// // // 	    
// // // 	    if(index==Instant){std::cout<<"Index==Instant"<<std::endl;
// // // 	      pub_num=Middle_Traj_pub.rows();
// // // 	      flag_concatenate_started=false;
// // // 	    }
// // // // 	    
// // // // 	    if(flag_concatenate_started==false && index==pub_num && flag_concatenate_completed==false){
// // // if(flag_concatenate_started==false && index==pub_num){
// // // 	      Output_Traj=New_Traj_pub;
// // // 	      Old_Traj_pub=Output_Traj;
// // // // 	      
// // // 	      flag_concatenate_completed=true;
// // // // 	      flag_concatenate_started=false;
// // // // 	      
// // // 	      pub_num=New_Traj_pub.rows();
// // // 	      speed=Current_Speed;
// // // 	    }
// // // 	    
// // //       } // end if else speed==Current_Speed

// // // // 	      std::cout<<"now_ok"<<std::endl;
// // // 	      Output_Traj=Middle_Traj_pub;
// // // 	      pub_num=Middle_Traj_pub.rows();
// // // 	      //While index < Instant pub_num is the same
// // // // 	      flag_concatenate_completed=false;
// // // 	      	      
// // // 	    }
// // // 	    
// // // 
// // // // 	    
// // // // 	    if(flag_concatenate_started==false && index==pub_num && flag_concatenate_completed==false){
// // // if(flag_concatenate_started==false && index==pub_num){
// // // 	      Output_Traj=New_Traj_pub;
// // // 	      Old_Traj_pub=Output_Traj;
// // // // 	      
// // // 	      flag_concatenate_completed=true;
// // // // 	      flag_concatenate_started=false;
// // // // 	      
// // // 	      pub_num=New_Traj_pub.rows();
// // // 	      speed=Current_Speed;
// // // 	    }
// // // 	    
// // //       } // end if else speed==Current_Speed
     
//      double interp_Instant=Instant;
     double interp_Instant=std::round(Instant*Ts_old/pub_Ts);   
     if(flag_pub_traj){ 
       
		  if(speed==Current_Speed){
		    Output_Traj=Old_Traj_pub;
		    pub_num=Old_Traj_pub.rows();
		  }else{ // else if speed!= Current_Speed
			one_time=false; //new speed variations are disabled
	    
		      if(flag_concatenate_started){ 
		  
			    if(index>interp_Instant){
	      // 		      std::cout<<"too_soon"<<std::endl;
				Output_Traj=Old_Traj_pub;
				pub_num=Old_Traj_pub.rows();
			    //need to wait the next step, i.e. continue with current Output_Traj
			    }else{ 	   
				  if(index==interp_Instant){
				      std::cout<<"Index==interp_Instant"<<std::endl;
				      pub_num=Middle_Traj_pub.rows();
				      Output_Traj=Middle_Traj_pub;
				      flag_concatenate_started=false;
				      flag_concatenate_completed=false;
				      std::cout<<"pub num "<<pub_num<<std::endl;
				  }else{
				      Output_Traj=Old_Traj_pub;
				      pub_num=Old_Traj_pub.rows();
			      }
			    }
		      }
		  }
	    
	    if(flag_concatenate_started==false &&  flag_concatenate_completed==false){
		if(index==pub_num){
		    index=0;
		    Output_Traj=New_Traj_pub;
		    Old_Traj_pub=Output_Traj;
		  
		    flag_concatenate_completed=true;   
		    pub_num=New_Traj_pub.rows();
		    speed=Current_Speed;
		    one_time=true;
	      }
	    }     
	  
	    if(index==pub_num) index=0;
    	std::cout<<"pub_num "<<pub_num<<std::endl;	
    // 	std::cout<<"Instant "<<Instant<<std::endl;
    // 	std::cout<<"before for"<<std::endl;
    	  std::cout<<"index "<<index<<" Output_Traj dim "<<Output_Traj.rows()<<" "<<Output_Traj.cols()<<std::endl;	
	    for(int i=0;i<6;i++){
		traj_d[i]=Output_Traj(index,i);
	    } //end for i 
    // 	std::cout<<"after for"<<std::endl;	  
    // std::cout<<"index "<<index<<std::endl;
//---------------------------------------PUB----------------------------------

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