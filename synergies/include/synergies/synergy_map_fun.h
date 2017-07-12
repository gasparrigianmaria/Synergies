#include <iterator>
Eigen::MatrixXd From_Syn_To_Traj(double speed,double foot_h,Eigen::MatrixXd First_Synergy,Eigen::MatrixXd Second_Synergy,Eigen::MatrixXd First_mu,Eigen::MatrixXd Second_mu){

// This fix the error in matlab code  
// speed=speed/sqrt(9.81*0.3);
std::cout<<"Matlab bug fixed in c++ map function, please comment this line"<<std::endl;
  
double Traj_1_p00=-1.3887;
double Traj_1_p10=-2.831;
double Traj_1_p01=-78.3301;
double Traj_1_p20=-1.0401;
double Traj_1_p11=38.6574;
double Traj_1_p02=528.7842;

double Traj_1_gain=Traj_1_p00 + Traj_1_p10*speed + Traj_1_p01*foot_h + Traj_1_p20*pow(speed,2) + Traj_1_p11*speed*foot_h + Traj_1_p02*pow(foot_h,2);

double Pose_1_p00=-0.060553;
double Pose_1_p10=-2.07;
double Pose_1_p01=-23.5047;
double Pose_1_p20=-1.0251;
double Pose_1_p11=-17.8385;
double Pose_1_p02=135.1791;

double Pose_1_gain=Pose_1_p00 + Pose_1_p10*speed + Pose_1_p01*foot_h + Pose_1_p20*pow(speed,2) + Pose_1_p11*speed*foot_h + Pose_1_p02*pow(foot_h,2);

double Traj_2_p00=-1.1263;
double Traj_2_p10=14.1893;
double Traj_2_p01=-91.1295;
double Traj_2_p20=-14.2743;
double Traj_2_p11=42.7017;
double Traj_2_p02=478.2962;

double Traj_2_gain=Traj_2_p00 + Traj_2_p10*speed + Traj_2_p01*foot_h + Traj_2_p20*pow(speed,2) + Traj_2_p11*speed*foot_h + Traj_2_p02*pow(foot_h,2);

double Pose_2_p00=-0.01608;
double Pose_2_p10=-0.10148;
double Pose_2_p01=8.3063;
double Pose_2_p20=0.38192;
double Pose_2_p11=-18.946;
double Pose_2_p02=-108.35;

double Pose_2_gain=Pose_2_p00 + Pose_2_p10*speed + Pose_2_p01*foot_h + Pose_2_p20*pow(speed,2) + Pose_2_p11*speed*foot_h + Pose_2_p02*pow(foot_h,2);


Eigen::MatrixXd out_Traj = First_Synergy*Traj_1_gain + Second_Synergy*Traj_2_gain;
// Eigen::MatrixXd out_Traj = First_Synergy;

Eigen::MatrixXd New_out_Traj(60,6);

for(int i=0;i<6;i++){
//   std::cout<<"bias "<<Pose_1_gain*First_mu(i)<<std::endl;
  for(int j=0;j<60;j++)
    New_out_Traj(j,i)=out_Traj(i*60+j)+Pose_1_gain*First_mu(i)+Pose_2_gain*Second_mu(i);
}

// New_out_Traj+=Pose_1_gain*First_mu.transpose();// +Pose_2_gain*Second_mu;

// Eigen::MatrixXd out;
return New_out_Traj;
// out=transpose(reshape(First_Synergy*Traj_1_gain + Second_Synergy*Traj_2_gain,60,6))+Pose_1_gain*First_mu+Pose_2_gain*Second_mu;
}



Eigen::MatrixXd From_Syn_To_Traj_Interpolated_Sampled_TEST(double speed,double foot_h,Eigen::MatrixXd First_Synergy,Eigen::MatrixXd Second_Synergy,Eigen::MatrixXd First_mu,Eigen::MatrixXd Second_mu, int num, double curr_Ts, double des_Ts){
// This fix the error in matlab code  
speed=speed/sqrt(9.81*0.3);
std::cout<<"Matlab bug fixed in c++ map function, please comment this line"<<std::endl;
  
double Traj_1_p00=-1.3887;
double Traj_1_p10=-2.831;
double Traj_1_p01=-78.3301;
double Traj_1_p20=-1.0401;
double Traj_1_p11=38.6574;
double Traj_1_p02=528.7842;

double Traj_1_gain=Traj_1_p00 + Traj_1_p10*speed + Traj_1_p01*foot_h + Traj_1_p20*pow(speed,2) + Traj_1_p11*speed*foot_h + Traj_1_p02*pow(foot_h,2);

double Pose_1_p00=-0.060553;
double Pose_1_p10=-2.07;
double Pose_1_p01=-23.5047;
double Pose_1_p20=-1.0251;
double Pose_1_p11=-17.8385;
double Pose_1_p02=135.1791;

double Pose_1_gain=Pose_1_p00 + Pose_1_p10*speed + Pose_1_p01*foot_h + Pose_1_p20*pow(speed,2) + Pose_1_p11*speed*foot_h + Pose_1_p02*pow(foot_h,2);

double Traj_2_p00=-1.1263;
double Traj_2_p10=14.1893;
double Traj_2_p01=-91.1295;
double Traj_2_p20=-14.2743;
double Traj_2_p11=42.7017;
double Traj_2_p02=478.2962;

double Traj_2_gain=Traj_2_p00 + Traj_2_p10*speed + Traj_2_p01*foot_h + Traj_2_p20*pow(speed,2) + Traj_2_p11*speed*foot_h + Traj_2_p02*pow(foot_h,2);

double Pose_2_p00=-0.01608;
double Pose_2_p10=-0.10148;
double Pose_2_p01=8.3063;
double Pose_2_p20=0.38192;
double Pose_2_p11=-18.946;
double Pose_2_p02=-108.35;

double Pose_2_gain=Pose_2_p00 + Pose_2_p10*speed + Pose_2_p01*foot_h + Pose_2_p20*pow(speed,2) + Pose_2_p11*speed*foot_h + Pose_2_p02*pow(foot_h,2);


Eigen::MatrixXd out_Traj = First_Synergy*Traj_1_gain + Second_Synergy*Traj_2_gain; //Dimensions 360x1 


    std::cout<<"out_Traj dimensions "<<out_Traj.rows()<<" "<<out_Traj.cols()<<std::endl;
    std::cout<<"num "<<num<<std::endl;
    std::vector<double> X(60),X_n(num),Y(60);
    
//     The time vectors old and new
    X[0]=curr_Ts;
    X_n[0]=0;
    
    for(int i=1;i<60;i++){
      X[i]=X[i-1]+curr_Ts; // curr_Ts is the sample time of the trajectory that we want to implement
    }
    
    for(int i=1;i<num;i++){
    X_n[i]=X_n[i-1]+des_Ts; //des_Ts is the time at which we want to publish
    }
    
    //at the end the time should be the same
    
    std::cout<<"Final time X "<<X[59]<<std::endl;
    std::cout<<"Final time X_n "<<X_n[num-1]<<std::endl;

//     for(int i=0;i<60;i++){
//       X[i]=i;
//     }
//     
//     for(int i=0;i<num;i++){
//     X_n[i]=i;
//     }
    
  std::vector<double> NoT(60);

 Eigen::MatrixXd New_out_Traj(num,6);
 
    for(int i=0;i<6;i++){
      
      for(int j=0;j<60;j++){
	// typecasting
	NoT[j]=out_Traj(i*60+j)+Pose_1_gain*First_mu(i)+Pose_2_gain*Second_mu(i);
      }
      
      // Create the spline object
      tk::spline s;
      s.set_points(X,NoT);
     
      
      
      for(int j=0;j<num;j++){
	// typecasting
	New_out_Traj(j,i)=s(X_n[j]);
      }
      
      
    }
    
    std::cout<<"spline completed"<<std::endl;
    std::cout<<"New_out_Traj "<<New_out_Traj.row(0)<<std::endl;
    std::cout<<" "<<New_out_Traj.row(num-1)<<std::endl;

// New_out_Traj+=Pose_1_gain*First_mu.transpose();// +Pose_2_gain*Second_mu;

//     to provide splines

    
//     vector<double> v2;
// v2.resize(v1.size());
// VectorXd::Map(&v2[0], v1.size()) = v1;

//   std::vector<std::complex<double> > v2(10);
//   VectorXcd v3 = VectorXcd::Map(v2.data(), v2.size());
    
//     for(int i=0;i<6;i++){
//       for(int j=0;j<60;j++)
// 	// typecasting
// 	New_out_Traj(j,i)=s.set_points(X[j],New_out_Traj(i,j));
// }
//     
//     s.set_points(X,Y);    // currently it is required that X is already sorted

// Eigen::MatrixXd out;
return New_out_Traj;
// out=transpose(reshape(First_Synergy*Traj_1_gain + Second_Synergy*Traj_2_gain,60,6))+Pose_1_gain*First_mu+Pose_2_gain*Second_mu;
}


//-----------------------------------------------------------



Eigen::MatrixXd From_Syn_To_Traj_Interpolated_Sampled(double speed,double foot_h,Eigen::MatrixXd First_Synergy,Eigen::MatrixXd Second_Synergy,Eigen::MatrixXd First_mu,Eigen::MatrixXd Second_mu, int num, double curr_Ts, double des_Ts){

// This fix the error in matlab code  
speed=speed/sqrt(9.81*0.3);
std::cout<<"Matlab bug fixed in c++ map function, please comment this line"<<std::endl;
  
double Traj_1_p00=-1.3887;
double Traj_1_p10=-2.831;
double Traj_1_p01=-78.3301;
double Traj_1_p20=-1.0401;
double Traj_1_p11=38.6574;
double Traj_1_p02=528.7842;

double Traj_1_gain=Traj_1_p00 + Traj_1_p10*speed + Traj_1_p01*foot_h + Traj_1_p20*pow(speed,2) + Traj_1_p11*speed*foot_h + Traj_1_p02*pow(foot_h,2);

double Pose_1_p00=-0.060553;
double Pose_1_p10=-2.07;
double Pose_1_p01=-23.5047;
double Pose_1_p20=-1.0251;
double Pose_1_p11=-17.8385;
double Pose_1_p02=135.1791;

double Pose_1_gain=Pose_1_p00 + Pose_1_p10*speed + Pose_1_p01*foot_h + Pose_1_p20*pow(speed,2) + Pose_1_p11*speed*foot_h + Pose_1_p02*pow(foot_h,2);

double Traj_2_p00=-1.1263;
double Traj_2_p10=14.1893;
double Traj_2_p01=-91.1295;
double Traj_2_p20=-14.2743;
double Traj_2_p11=42.7017;
double Traj_2_p02=478.2962;

double Traj_2_gain=Traj_2_p00 + Traj_2_p10*speed + Traj_2_p01*foot_h + Traj_2_p20*pow(speed,2) + Traj_2_p11*speed*foot_h + Traj_2_p02*pow(foot_h,2);

double Pose_2_p00=-0.01608;
double Pose_2_p10=-0.10148;
double Pose_2_p01=8.3063;
double Pose_2_p20=0.38192;
double Pose_2_p11=-18.946;
double Pose_2_p02=-108.35;

double Pose_2_gain=Pose_2_p00 + Pose_2_p10*speed + Pose_2_p01*foot_h + Pose_2_p20*pow(speed,2) + Pose_2_p11*speed*foot_h + Pose_2_p02*pow(foot_h,2);


Eigen::MatrixXd out_Traj = First_Synergy*Traj_1_gain + Second_Synergy*Traj_2_gain; //Dimensions 360x1 


    std::cout<<"out_Traj dimensions "<<out_Traj.rows()<<" "<<out_Traj.cols()<<std::endl;
    std::cout<<"num "<<num<<std::endl;
    std::vector<double> X(60),X_n(num),Y(60);
    
//     The time vectors old and new
    X[0]=curr_Ts;
    X_n[0]=0;
    
    for(int i=1;i<60;i++){
      X[i]=X[i-1]+curr_Ts; // curr_Ts is the sample time of the trajectory that we want to implement
    }
    
    for(int i=1;i<num;i++){
    X_n[i]=X_n[i-1]+des_Ts; //des_Ts is the time at which we want to publish
    }
    
    //at the end the time should be the same
    
    std::cout<<"Final time X "<<X[59]<<std::endl;
    std::cout<<"Final time X_n "<<X_n[num-1]<<std::endl;

//     for(int i=0;i<60;i++){
//       X[i]=i;
//     }
//     
//     for(int i=0;i<num;i++){
//     X_n[i]=i;
//     }
    
  std::vector<double> NoT(60);

 Eigen::MatrixXd New_out_Traj(num,6);
 
    for(int i=0;i<6;i++){
      
      for(int j=0;j<60;j++){
	// typecasting
	NoT[j]=out_Traj(i*60+j)+Pose_1_gain*First_mu(i)+Pose_2_gain*Second_mu(i);
      }
      
      // Create the spline object
      tk::spline s;
      s.set_points(X,NoT);
     
      
      
      for(int j=0;j<num;j++){
	// typecasting
	New_out_Traj(j,i)=s(X_n[j]);
      }
      
      
    }
    
    std::cout<<"spline completed"<<std::endl;
    std::cout<<"New_out_Traj "<<New_out_Traj.row(0)<<std::endl;
    std::cout<<" "<<New_out_Traj.row(num-1)<<std::endl;

// New_out_Traj+=Pose_1_gain*First_mu.transpose();// +Pose_2_gain*Second_mu;

//     to provide splines

    
//     vector<double> v2;
// v2.resize(v1.size());
// VectorXd::Map(&v2[0], v1.size()) = v1;

//   std::vector<std::complex<double> > v2(10);
//   VectorXcd v3 = VectorXcd::Map(v2.data(), v2.size());
    
//     for(int i=0;i<6;i++){
//       for(int j=0;j<60;j++)
// 	// typecasting
// 	New_out_Traj(j,i)=s.set_points(X[j],New_out_Traj(i,j));
// }
//     
//     s.set_points(X,Y);    // currently it is required that X is already sorted

// Eigen::MatrixXd out;
return New_out_Traj;
// out=transpose(reshape(First_Synergy*Traj_1_gain + Second_Synergy*Traj_2_gain,60,6))+Pose_1_gain*First_mu+Pose_2_gain*Second_mu;
}


Eigen::MatrixXd Interpolate_and_Resample(Eigen::MatrixXd Trajectory, double Traj_Ts, double Pub_Ts){
      
  int Traj_num=Trajectory.rows();// this is 60x6
  int n_joints=Trajectory.cols();
//   std::cout<<"Traj 0,0 "<<Trajectory(0,0)<<std::endl;
  
  int pub_num=std::round(Traj_num*Traj_Ts/Pub_Ts);
  
//   std::cout<<"Traj_dim "<<Traj_num<<" n_joints "<<n_joints<<" pub_num "<<pub_num<<std::endl;
  std::vector<double> X_traj(Traj_num),X_pub(pub_num),Traj_pub(pub_num), Traj(Traj_num);
  
  Eigen::MatrixXd out_Traj(pub_num,n_joints);
    
//     The time vectors old and new
	  	  
	  for(int i=0;i<Traj_num;i++){
	    X_traj[i]=(i+1)*Traj_Ts; // curr_Ts is the sample time of the trajectory that we want to implement
	  }
	  
	  for(int i=0;i<pub_num;i++){
	    X_pub[i]=(i+1)*Pub_Ts; // curr_Ts is the sample time of the trajectory that we want to implement
	  }
	  
	  tk::spline s;
	  for(int i=0;i<n_joints;i++){
	    
	    for(int j=0;j<Traj_num;j++){
	      Traj[j]=Trajectory(j,i);
	    }
	      
	    s.set_points(X_traj,Traj);
	    
	    for(int j=0;j<pub_num;j++){
	    out_Traj(j,i)=s(X_pub[j]);
	    }
	    
	  }
	  
return out_Traj;
    }

    
    
    
   Eigen::MatrixXd Interpolate_and_Resample_Middle(Eigen::MatrixXd New_Traj, double Ts_new, Eigen::MatrixXd Old_Traj, double Ts_old, int Instant, double Pub_Ts){
     
     int Traj_num=Old_Traj.rows();// this is 60x6 and it is the same of the new
     int n_joints=Old_Traj.cols();
     
     int pub_num=std::round(((Instant-1)*Ts_old+(Traj_num-Instant+1)*Ts_new)/Pub_Ts);
     // -1 'cause we switch the signals at i==Instant, i.e. i==Instant New_signals(i)
     
     std::vector<double> X_traj(Traj_num),X_pub(pub_num),Traj_pub(pub_num), Traj(Traj_num);
    
     Eigen::MatrixXd Middle_Traj_pub(pub_num,n_joints);
     
     for(int i=0;i<Traj_num;i++){
       if(i<Instant){
	    X_traj[i]=(i+1)*Ts_old; // curr_Ts is the sample time of the trajectory that we want to implement
       }else{
	    X_traj[i]=X_traj[Instant-1]+(i-Instant+1)*Ts_new;   
	  }
     }
//      for(int i=0;i<Traj_num;i++){
//      std::cout<<X_traj[i]<<std::endl;
//      }
	  
      for(int i=0;i<pub_num;i++){
	    X_pub[i]=(i+1)*Pub_Ts; // curr_Ts is the sample time of the trajectory that we want to implement
	  }
	  

      tk::spline s; 
      
      for(int i=0;i<n_joints;i++){
	
	for(int j=0;j<Traj_num;j++){
	  if(j<Instant){
	     Traj[j]=Old_Traj(j,i);
	  }else{
	     Traj[j]=New_Traj(j,i);  
	    }  
	}
	
	s.set_points(X_traj,Traj);
	
	for(int j=0;j<pub_num;j++){
	    Middle_Traj_pub(j,i)=s(X_pub[j]);
	    }
	    
	
      }
     
      
     return Middle_Traj_pub;
  }
    