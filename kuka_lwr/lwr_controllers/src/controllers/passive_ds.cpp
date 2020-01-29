#include "controllers/passive_ds.h"
#include <tf/transform_broadcaster.h>
#include <control_toolbox/filters.h>

namespace controllers{

static const double thrott_time = 3.0;

Passive_ds::Passive_ds(ros::NodeHandle &nh, controllers::Change_ctrl_mode &change_ctrl_mode):
    Base_controllers(lwr_controllers::CTRL_MODE::CART_PASSIVE_DS),
    change_ctrl_mode(change_ctrl_mode)
{

    /// ROS topic

    sub_command_vel_       = nh.subscribe("passive_ds_command_vel",      1, &Passive_ds::command_cart_vel,     this ,ros::TransportHints().reliable().tcpNoDelay());
    sub_command_orient_    = nh.subscribe("passive_ds_command_orient",   1 ,&Passive_ds::command_orient,       this ,ros::TransportHints().reliable().tcpNoDelay());
    sub_command_force_     = nh.subscribe("passive_ds_command_wrench",    1, &Passive_ds::command_wrench,   this ,ros::TransportHints().reliable().tcpNoDelay());
    sub_eig_               = nh.subscribe("passive_ds_eig",              1 ,&Passive_ds::command_damping_eig,  this ,ros::TransportHints().reliable().tcpNoDelay());
    sub_stiff_             = nh.subscribe("passive_ds_stiffness",        1 ,&Passive_ds::command_rot_stiff,    this ,ros::TransportHints().reliable().tcpNoDelay());
    sub_damp_              = nh.subscribe("passive_ds_damping",          1 ,&Passive_ds::command_rot_damp,     this ,ros::TransportHints().reliable().tcpNoDelay());
    sub_command_nullspace_    = nh.subscribe("passive_ds_command_nullspace",  1 ,&Passive_ds::command_nullspace,       this ,ros::TransportHints().reliable().tcpNoDelay());

    pub_twist_ = nh.advertise<geometry_msgs::Twist>("twist", 1);
    pub_damping_matrix_ = nh.advertise<std_msgs::Float32MultiArray>("passive_ds_damping_matrix", 1);
    /// Passive dynamical system

    passive_ds_controller.reset(new DSController(3,50.0,50.0));

    nd5 = ros::NodeHandle(nh.getNamespace()+"/ds_param");

    dynamic_server_ds_param.reset(new       dynamic_reconfigure::Server< lwr_controllers::passive_ds_paramConfig>(nd5));


    dynamic_server_ds_param->setCallback(    boost::bind(&Passive_ds::ds_param_callback,     this, _1, _2));
    dynamic_server_ds_param->getConfigDefault(config_cfg);

    for(std::size_t i = 0; i < 9; i++){
        err_orient.data[i] = 0;
    }

    dx_linear_des_.resize(3);
    dx_linear_msr_.resize(3);
    dx_angular_msr_.resize(3);
    dx_angular_des_.resize(3);

    F_ee_des_.resize(6);
    wrench_des_.resize(6);
    wrench_des_.setConstant(0.0f);
    nullspace_command.setConstant(0.0f);
    nullspace_torque.setConstant(0.0f);

    bFirst = false;

    rot_stiffness = config_cfg.rot_stiffness;
    rot_damping   = config_cfg.rot_damping;

    rot_des_ = KDL::Rotation::RPY(0,0,0);

   // qd << -0.004578103311359882, 0.7503823041915894, -0.059841930866241455, -1.6525769233703613,
   //       0.06038748472929001, 0.7602048516273499, 1.5380386114120483;
   // qd << 0.0f, 0.0f, 0.0f, -2.094, 0.0f, 1.047f, 1.57f;

   // qd << -0.05035164952278137, 0.586331844329834, -0.10019383579492569, -1.8283051252365112, 1.500396490097046, 1.7245664596557617, 0.6142861247062683;
   qd << 0.7853982958343035, 1.2217446372146226, -1.2217300551038797, -1.5707990882732297, 0.7853982891845535, 1.5707970122758415, 1.9831067620046383e-05;
     _torqueLimits << 176.0f, 176.0f, 100.0f, 100.0f, 100.0f, 38.0f,38.0f;
     _jointVelocityLimits << 110.0f,110.0f,128.0f,128.0f,204.0f,184.0f,184.0f;
     _jointVelocityLimits *=M_PI/180.0f;
     // _jointLimits << 170.0f,120.0f,170.0f,120.0f,170.0f,120.0f,170.0f;
     _jointLimits << 150.0f,100.0f,150.0f,100.0f,150.0f,100.0f,150.0f;
     _jointLimits *=M_PI/180.0f;

    /// ROS pub debug

    pub_F_                 = nh.advertise<std_msgs::Float64MultiArray>("F_ee",10);
    torque_pub_            = nh.advertise<std_msgs::Float64MultiArray>("tau_pds",10);
    F_msg_.data.resize(6);
    tau_msg_.data.resize(7);

    bDebug      = config_cfg.debug;
    bSmooth     = config_cfg.bSmooth;
    smooth_val_ = config_cfg.smooth_val;
    _useNullSpace = config_cfg.useNullSpace;
    _jointLimitsGain = config_cfg.jointLimitsGain;
    _desiredJointsGain = config_cfg.desiredJointsGain;
    _jointVelocitiesGain = config_cfg.jointVelocitiesGain;
    _wrenchGain = config_cfg.wrenchGain;

    _sqp = new qpOASES::SQProblem(14,7);

    auto options = _sqp->getOptions();
    // options.printLevel = qpOASES::PL_NONE;
    options.printLevel = qpOASES::PL_LOW;
    // options.enableFarBounds = qpOASES::BT_TRUE;
    // options.enableFlippingBounds = qpOASES::BT_TRUE;
    options.enableRamping = qpOASES::BT_FALSE;
    options.enableNZCTests = qpOASES::BT_FALSE;
    // options.enableRegularisation = qpOASES::BT_TRUE;
    options.enableDriftCorrection = 0;
    options.terminationTolerance = 1e-6;
    options.boundTolerance = 1e-4;
    options.epsIterRef = 1e-6;
    _sqp->setOptions(options);

    _initQP = true;
}


void Passive_ds::stop(){
    ROS_INFO_STREAM("stopping [PASSIVE_DS]");
    bFirst      = false;
}

void Passive_ds::ds_param_callback(lwr_controllers::passive_ds_paramConfig& config,uint32_t level){
    passive_ds_controller->set_damping_eigval(config.damping_eigval0,config.damping_eigval1);
    rot_stiffness = config.rot_stiffness;
    rot_damping   = config.rot_damping;
    bDebug        = config.debug;
    bSmooth       = config.bSmooth;
    smooth_val_   = config.smooth_val;
    _useNullSpace = config.useNullSpace;
    _jointLimitsGain = config.jointLimitsGain;
    _desiredJointsGain = config.desiredJointsGain;
    _jointVelocitiesGain = config.jointVelocitiesGain;
    _wrenchGain = config.wrenchGain;
    _nullspaceCommandGain = config.nullspaceCommandGain;
    if(_useQP == false && config.useQP == true)
    {
        _initQP = true;
    }
     _useQP = config.useQP;
     _useKDLInertiaMatrix = config.useKDLInertiaMatrix;
     _useCoriolis = config.useCoriolis;
     _alpha1 = config.alpha1;
     _alpha2 = config.alpha2;
     _alpha3 = config.alpha3;


    config_cfg    = config;
}


void Passive_ds::update(KDL::Wrench &wrench, KDL::JntArray& tau_cmd, const KDL::Jacobian &J, const KDL::JntArrayAcc& joint_msr_ , const KDL::Twist x_msr_vel_, const KDL::Rotation& rot_msr_, const KDL::Vector& p, Eigen::MatrixXd inertiaMatrix, Eigen::VectorXd coriolis){


    F_ee_des_.setZero();

    /// set desired linear velocity
    dx_linear_des_(0)   = x_des_vel_(0);
    dx_linear_des_(1)   = x_des_vel_(1);
    dx_linear_des_(2)   = x_des_vel_(2);

    dx_angular_des_(0) = x_des_vel_.rot(0);
    dx_angular_des_(1) = x_des_vel_.rot(1);
    dx_angular_des_(2) = x_des_vel_.rot(2);

    /// set measured linear and angular velocity
    if(bSmooth)
    {
        dx_linear_msr_(0)   = x_msr_vel_.vel(0);
        dx_linear_msr_(1)   = x_msr_vel_.vel(1);
        dx_linear_msr_(2)   = x_msr_vel_.vel(2);

        dx_angular_msr_(0)  = x_msr_vel_.rot(0);
        dx_angular_msr_(1)  = x_msr_vel_.rot(1);
        dx_angular_msr_(2)  = x_msr_vel_.rot(2);
    }else{
       dx_linear_msr_(0)    = filters::exponentialSmoothing(x_msr_vel_.vel(0), dx_linear_msr_(0),smooth_val_);
       dx_linear_msr_(1)    = filters::exponentialSmoothing(x_msr_vel_.vel(1), dx_linear_msr_(1),smooth_val_);
       dx_linear_msr_(2)    = filters::exponentialSmoothing(x_msr_vel_.vel(2), dx_linear_msr_(2),smooth_val_);

       dx_angular_msr_(0)   = filters::exponentialSmoothing(x_msr_vel_.rot(0), dx_angular_msr_(0),smooth_val_);
       dx_angular_msr_(1)   = filters::exponentialSmoothing(x_msr_vel_.rot(1), dx_angular_msr_(1),smooth_val_);
       dx_angular_msr_(2)   = filters::exponentialSmoothing(x_msr_vel_.rot(2), dx_angular_msr_(2),smooth_val_);
    }


    // ----------------- Linear velocity -> Force -----------------------//

    passive_ds_controller->Update(dx_linear_msr_,dx_linear_des_);
    F_linear_des_ = passive_ds_controller->control_output(); // (3 x 1)
    _damping = (passive_ds_controller->damping_matrix()).cast<float>();

    F_ee_des_(0) = F_linear_des_(0);
    F_ee_des_(1) = F_linear_des_(1);
    F_ee_des_(2) = F_linear_des_(2);

    // ----------------- Debug -----------------------//

    // ROS_WARN_STREAM_THROTTLE(1, "velocity :" << dx_linear_des_ );
    // ROS_WARN_STREAM_THROTTLE(1, "Froces :" << F_linear_des_ );


    if(bDebug){

        std::string robot_name = nd5.getNamespace().substr(0,nd5.getNamespace().find("/",1));
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(p(0),p(1),p(2)) );
            rot_msr_.GetQuaternion(qx,qy,qz,qw);
            q = tf::Quaternion(qx,qy,qz,qw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_name+"/rot_msr_"));
        }
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(p(0),p(1),p(2)) );
            rot_des_.GetQuaternion(qx,qy,qz,qw);
            q = tf::Quaternion(qx,qy,qz,qw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_name+"/rot_target_"));
        }
    }

    // ----------------- Rotation target -> Force -----------------------//

    // damp any rotational motion
    err_orient = rot_des_*rot_msr_.Inverse();
    err_orient.GetQuaternion(qx,qy,qz,qw);
    q = tf::Quaternion(qx,qy,qz,qw);

    err_orient_axis = q.getAxis();
    err_orient_angle = q.getAngle();

    // rotational stiffness. This is correct sign and everything!!!! do not mess with this!
    torque_orient = err_orient_axis * err_orient_angle * (rot_stiffness);

    F_ee_des_(3) = -rot_damping * (dx_angular_msr_(0)-dx_angular_des_(0)) + torque_orient.getX();
    F_ee_des_(4) = -rot_damping * (dx_angular_msr_(1)-dx_angular_des_(1)) + torque_orient.getY();
    F_ee_des_(5) = -rot_damping * (dx_angular_msr_(2)-dx_angular_des_(2)) + torque_orient.getZ();


    if(bDebug){
        // ROS_WARN_STREAM_THROTTLE(2.0,"err_orient_axis: " << err_orient_axis.getX() << " " << err_orient_axis.getY() << " " << err_orient_axis.getZ() );
        // ROS_WARN_STREAM_THROTTLE(2.0,"err_orient_angle: " << err_orient_angle);
        ROS_WARN_STREAM_THROTTLE(1.0, "Forces :" << F_ee_des_ );
        // std::cerr << "Contact force: " << wrench_des_ << std::endl;
    }

    if(std::isnan(err_orient_angle) || std::isinf(err_orient_angle)){
        ROS_WARN_STREAM_THROTTLE(thrott_time,"err_orient_angle: " << err_orient_angle );
        err_orient_angle = 0;
    }


    if(bDebug){
        for(std::size_t i = 0; i < F_msg_.data.size();i++){
            F_msg_.data[i] =F_ee_des_(i);
        }
        pub_F_.publish(F_msg_);
    }

    // computing the torques
    Eigen::MatrixXd J_transpose_pinv;
    pseudo_inverse(J.data.transpose(), J_transpose_pinv);
    // nullspace_torque << (Eigen::MatrixXd::Identity(7, 7) - J.data.transpose()*J_transpose_pinv)*(2.0*(qd - joint_msr_.q.data) - 0.01*joint_msr_.qdot.data);
    nullspace_torque << (Eigen::MatrixXd::Identity(7, 7) - J.data.transpose()*J_transpose_pinv)*(-_jointLimitsGain*joint_msr_.q.data
                                                                                                 -_desiredJointsGain*(joint_msr_.q.data-qd)
                                                                                                 -_jointVelocitiesGain*joint_msr_.qdot.data
                                                                                                 +_wrenchGain*J.data.transpose()*wrench_des_
                                                                                                 + _nullspaceCommandGain*nullspace_command);

    // Eigen::Matrix<double,7,1> temp;
    // temp = -_jointLimitsGain*joint_msr_.q.data-_desiredJointsGain*(joint_msr_.q.data-qd)
    //        -_jointVelocitiesGain*joint_msr_.qdot.data+_wrenchGain*J.data.transpose()*wrench_des_
    //        +_nullspaceCommandGain*nullspace_command;

    nullspace_torque(6) = 0.0f;

    if(_useQP)
    {
        if(!_useKDLInertiaMatrix)
        {
            inertiaMatrix = Eigen::Matrix<double,7,7>::Identity();
        }
        Eigen::Matrix<double,7,6> Jp;
        Jp = inertiaMatrix.inverse()*J.data.transpose()*(J.data*inertiaMatrix.inverse()*J.data.transpose()).inverse();

        Eigen::Matrix<double,7,7> Np;
        Np = Eigen::Matrix<double,7,7>::Identity()-J.data.transpose()*Jp.transpose();

        Eigen::Matrix<double,14,14> H;
        H.setConstant(0.0f);
        // inertiaMatrix.inverse();
        // J.data.transpose()*J;
        // H.block(0,0,7,7) = inertiaMatrix.inverse()*(J.data.transpose()*J.data)*inertiaMatrix.inverse()+1.0f*Eigen::Matrix<double,7,7>::Identity();
        // H.block(0,0,7,7) = inertiaMatrix.inverse()*(J.data.transpose()*J.data)*inertiaMatrix.inverse()+100.0f*(Np*Np.transpose());
        H.block(0,0,7,7) = Eigen::Matrix<double,7,7>::Identity()+_alpha1*(Np*Np.transpose())+_alpha2*Eigen::Matrix<double,7,7>::Identity();
        H.block(7,7,7,7) = Eigen::Matrix<double,7,7>::Identity()+_alpha3*Eigen::Matrix<double,7,7>::Identity();

        Eigen::Matrix<double,14,1> g;
        // g.segment(0,7) = -H.block(0,0,7,7)*J.data.transpose()*F_ee_des_;
        g.segment(0,7) = -J.data.transpose()*F_ee_des_;
        g.segment(7,7) = _desiredJointsGain*(joint_msr_.q.data-qd)+_jointVelocitiesGain*joint_msr_.qdot.data;

        Eigen::Matrix<double,7,14> A;
        A.block(0,0,7,7) = Eigen::Matrix<double,7,7>::Identity();
        A.block(0,7,7,7) = Np;

        Eigen::Matrix<double,14,1> lb, ub;
        ub.segment(0,7) = 2.0f*_torqueLimits.cast <double> ();
        ub.segment(7,7) = 2.0f*_torqueLimits.cast <double> ();
        lb = -ub;

        Eigen::Matrix<double,7,1> Alb, Aub, temp1, temp2, qdotdotmax, qdotdotmin, taumin,taumax;


        float dt = 0.005f;
        temp1 = (_jointVelocityLimits.cast<double>()-joint_msr_.qdot.data)/dt;
        temp2 = (_jointLimits.cast<double>()-joint_msr_.q.data-joint_msr_.qdot.data*dt)/(0.5f*dt*dt);
        qdotdotmax = temp1.cwiseMin(temp2);

        temp1 = (-_jointVelocityLimits.cast<double>()-joint_msr_.qdot.data)/dt;
        temp2 = (-_jointLimits.cast<double>()-joint_msr_.q.data-joint_msr_.qdot.data*dt)/(0.5f*dt*dt);
        qdotdotmin = temp1.cwiseMax(temp2);




        // temp1 = inertiaMatrix*qdotdotmax;    
        // temp2 = inertiaMatrix*qdotdotmin;

        // if(_useCoriolis)
        // {

        //     temp1 += coriolis;
        //     temp2 += coriolis;
        // }

        // Aub = (_torqueLimits.cast<double>()).cwiseMin(temp1.cwiseMax(temp2));
        // Alb = (-_torqueLimits.cast<double>()).cwiseMax(temp1.cwiseMin(temp2));

        for(int k = 0; k < inertiaMatrix.rows(); k++)
        {
            temp1(k) = 0.0f;
            temp2(k) = 0.0f;
            for(int m = 0; m < inertiaMatrix.cols(); m++)
            {

                // temp1(k) += std::max(inertiaMatrix(k,m)*qdotdotmin(m),inertiaMatrix(k,m)*qdotdotmax(m));
                // temp2(k) += std::min(inertiaMatrix(k,m)*qdotdotmin(m),inertiaMatrix(k,m)*qdotdotmax(m));

                temp1(k) += (inertiaMatrix(k,m) > 0.0f)?  inertiaMatrix(k,m)*qdotdotmax(m):  inertiaMatrix(k,m)*qdotdotmin(m);
                temp2(k) += (inertiaMatrix(k,m) > 0.0f)?  inertiaMatrix(k,m)*qdotdotmin(m):  inertiaMatrix(k,m)*qdotdotmax(m);

            }
        }

        // std::cerr << "before max: " <<temp1.transpose() << std::endl;
        // std::cerr << "before min: " <<  temp2.transpose() << std::endl;
        if(_useCoriolis)
        {

            temp1 += coriolis;
            temp2 += coriolis;
        }
        // std::cerr << "after max: " <<temp1.transpose() << std::endl;
        // std::cerr << "after min: " << temp2.transpose() << std::endl;


        Aub = (_torqueLimits.cast<double>()).cwiseMin(temp1);
        Alb = (-_torqueLimits.cast<double>()).cwiseMax(temp2);



        for (int i = 0; i < H.rows(); i++) 
        {
            for (int j = 0; j < H.cols(); j++)
            {
              H_qp[i*H.cols()+j] = H(i,j);
            }
        }

        for (int i = 0; i < A.rows(); i++)
        {
            for (int j = 0; j < A.cols(); j++)
            {
              A_qp[i*A.cols()+j] = A(i,j);
            }
        }

        for (int i = 0; i < g.size(); i++) 
        {
            g_qp[i] = g(i);
            lb_qp[i] = lb(i);
            ub_qp[i] = ub(i);
        }

        for (size_t i = 0; i < 7; i++) 
        {
            lbA_qp[i] = Alb(i);
            ubA_qp[i] = Aub(i);
        }

        qpOASES::SymSparseMat H_mat(H.rows(), H.cols(), H.cols(), H_qp);
        H_mat.createDiagInfo();
        qpOASES::SparseMatrix A_mat(A.rows(), A.cols(), A.cols(), A_qp);
        qpOASES::returnValue ret = qpOASES::TERMINAL_LIST_ELEMENT;

        // std::cerr << "H" << std::endl;
        // std::cerr << H << std::endl;
        // std::cerr << "A" << std::endl;
        // std::cerr << A << std::endl;
        // std::cerr << "inertiaMatrix" << std::endl;
        // std::cerr << inertiaMatrix << std::endl;
        // std::cerr << "eigenvalues" << std::endl;
        // std::cerr << inertiaMatrix.eigenvalues() << std::endl;
        // std::cerr << "qdotdotmax" << std::endl;
        // std::cerr << qdotdotmax << std::endl;
        // std::cerr << "qdotdotmin" << std::endl;
        // std::cerr << qdotdotmin << std::endl;
        // std::cerr << "qdotdotmax" << std::endl;
        // std::cerr << inertiaMatrix*qdotdotmax << std::endl;
        // std::cerr << "qdotdotmin" << std::endl;
        // std::cerr << inertiaMatrix*qdotdotmin << std::endl;
        // std::cerr << "Alb" << std::endl;
        // std::cerr << Alb << std::endl;
        // std::cerr << "Aub" << std::endl;
        // std::cerr << Aub << std::endl;
        // std::cerr << "lb" << std::endl;
        // std::cerr << lb << std::endl;
        // std::cerr << "ub" << std::endl;
        // std::cerr << ub << std::endl;
        // std::cerr << "g" << std::endl;
        // std::cerr << g << std::endl;

        int max_iters = 100;
        if(_initQP)
        {
            ret = _sqp->init(&H_mat, g_qp, &A_mat, lb_qp, ub_qp, lbA_qp, ubA_qp, max_iters);
            _initQP = false;

        }
        else
        {
            ret = _sqp->hotstart(&H_mat, g_qp, &A_mat, lb_qp, ub_qp, lbA_qp, ubA_qp, max_iters);
        }

        qpOASES::real_t xOpt[14];
          
        _sqp->getPrimalSolution(xOpt);

        Eigen::Matrix<double,14,1> result; 
        for(int k = 0; k < 14; k++)
        {
            result(k) = xOpt[k];
        }
    
        if(ret == qpOASES::SUCCESSFUL_RETURN)
        {
            tau_cmd.data = result.segment(0,7)+Np*result.segment(7,7);
        }
        else
        {
            tau_cmd.data.setConstant(0.0f);
        }
    }
    else
    {

        if(_useNullSpace)
        {
            tau_cmd.data = J.data.transpose() * F_ee_des_ + nullspace_torque;
        }
        else
        {
            tau_cmd.data = J.data.transpose() * F_ee_des_;
        }
        
        float alpha = 0.95f;

        for(int k = 0; k < 7; k++)
        {
            if(tau_cmd.data(k)>alpha*_torqueLimits(k))
            {
                tau_cmd.data(k) = alpha*_torqueLimits(k);
            } 
            else if(tau_cmd.data(k)<-alpha*_torqueLimits(k))
            {
                tau_cmd.data(k) = -alpha*_torqueLimits(k);
            }    
        }
    }

    // if(bDebug)
    // {
    //     ROS_WARN_STREAM_THROTTLE(1.0, "Nullspace torques:" << nullspace_torque );
    //     ROS_WARN_STREAM_THROTTLE(1.0, "Force nullspace :" << _wrenchGain*J.data.transpose()*wrench_des_);
    //     ROS_WARN_STREAM_THROTTLE(1.0, "Nullspace command :" << _nullspaceCommandGain*nullspace_command);
    // }

    // if(bDebug){
    //     for(std::size_t i = 0; i < tau_msg_.data.size();i++)
    //     {
    //         tau_msg_.data[i] = tau_cmd.data[i];
    //     }
    //     torque_pub_.publish(tau_msg_);
    // }

    wrench.force(0) = F_ee_des_(0);
    wrench.force(1) = F_ee_des_(1);
    wrench.force(2) = F_ee_des_(2);
    wrench.torque(0) = F_ee_des_(3);
    wrench.torque(1) = F_ee_des_(4);
    wrench.torque(2) = F_ee_des_(5);

    geometry_msgs::Twist msg;
    msg.linear.x = dx_linear_msr_(0);
    msg.linear.y = dx_linear_msr_(1);
    msg.linear.z = dx_linear_msr_(2);
    msg.angular.x = dx_angular_msr_(0);
    msg.angular.y = dx_angular_msr_(1);
    msg.angular.z = dx_angular_msr_(2);
    pub_twist_.publish(msg);

    std_msgs::Float32MultiArray msgDamping;
    msgDamping.data.resize(9);
    msgDamping.data[0] = _damping(0,0);
    msgDamping.data[1] = _damping(0,1);
    msgDamping.data[2] = _damping(0,2);
    msgDamping.data[3] = _damping(1,0);
    msgDamping.data[4] = _damping(1,1);
    msgDamping.data[5] = _damping(1,2);
    msgDamping.data[6] = _damping(2,0);
    msgDamping.data[7] = _damping(2,1);
    msgDamping.data[8] = _damping(2,2);
    pub_damping_matrix_.publish(msgDamping);
    // tau_cmd.data.setZero();
}

void Passive_ds::command_cart_vel(const geometry_msgs::TwistConstPtr &msg){
    x_des_vel_.vel(0) = msg->linear.x;
    x_des_vel_.vel(1) = msg->linear.y;
    x_des_vel_.vel(2) = msg->linear.z;

    x_des_vel_.rot(0) = msg->angular.x;
    x_des_vel_.rot(1) = msg->angular.y;
    x_des_vel_.rot(2) = msg->angular.z;

    if(!bFirst){
        change_ctrl_mode.switch_mode(lwr_controllers::CTRL_MODE::CART_PASSIVE_DS);
        _initQP = true;
    }
    bFirst            = true;
}
void Passive_ds::command_orient(const geometry_msgs::Quaternion &msg){
    rot_des_ = KDL::Rotation::Quaternion(msg.x,msg.y,msg.z,msg.w);
}

void Passive_ds::command_wrench(const geometry_msgs::WrenchConstPtr &msg){
    // rot_des_ = KDL::Rotation::Quaternion(msg.x,msg.y,msg.z,msg.w);
    wrench_des_(0) = msg->force.x;
    wrench_des_(1) = msg->force.y;
    wrench_des_(2) = msg->force.z;
    wrench_des_(3) = msg->torque.x;
    wrench_des_(4) = msg->torque.y;
    wrench_des_(5) = msg->torque.z;
}


void Passive_ds::command_damping_eig(const std_msgs::Float64MultiArray& msg){

    if(msg.data.size() == 2){
        if(passive_ds_controller != NULL){
            passive_ds_controller->set_damping_eigval(msg.data[0],msg.data[1]);
            lwr_controllers::passive_ds_paramConfig config;
            dynamic_server_ds_param->getConfigDefault(config);

            config_cfg.damping_eigval0 = msg.data[0];
            config_cfg.damping_eigval1 = msg.data[1];
            dynamic_server_ds_param->updateConfig(config_cfg);

        }else{
            ROS_ERROR_STREAM_THROTTLE(thrott_time,"[Passive_ds::command_damping_eig]  passive_ds_controller == NULL");
        }
    }else{
        ROS_ERROR_STREAM_THROTTLE(thrott_time,"[Passive_ds::command_damping_eig]   msg.data.size() is not equal to 2, it is : " << msg.data.size());
    }
}

void Passive_ds::command_rot_stiff(const std_msgs::Float64& msg){
    rot_stiffness = msg.data;

    config_cfg.rot_stiffness = rot_stiffness;
    dynamic_server_ds_param->updateConfig(config_cfg);
}

void Passive_ds::command_rot_damp(const std_msgs::Float64& msg){
    rot_damping  = msg.data;

    config_cfg.rot_damping = rot_damping;
    dynamic_server_ds_param->updateConfig(config_cfg);
}


void Passive_ds::command_nullspace(const std_msgs::Float32MultiArray& msg){

    if(msg.data.size() == 7)
    {
        for(int k = 0 ; k < 7; k++)
        {
            nullspace_command(k) = msg.data[k];
        }
    }
}

}





