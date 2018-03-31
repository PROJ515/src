

/*
    old_v_left = v_left;
    newposition = leftWheelDistance;
    newtime = millis();
    v_left = (newposition-oldposition)/(newtime-oldtime);
    oldposition = newposition;
    oldtime = newtime;
    
    
        old_v_right = v_right;
    rnewposition = rightWheelDistance;
    rnewtime = millis();
    v_right = (rnewposition-roldposition)/(rnewtime-roldtime);
    roldposition = rnewposition;
    roldtime = rnewtime;
    
    
    
      if (calculated){
    current_time = micros();
    vx = (v_left + v_right)/2;
    vth = (v_left - v_right)/wheel_spacing;

    double dt = (current_time - last_time);

    double delta_th = vth * dt;

th += delta_th;    //publish theta and v_theta
    float32_msg.data = th;
    theta.publish( &float32_msg);
    float32_msg.data = vth;
    vtheta.publish( &float32_msg);


    last_time = current_time;
    timeout = millis();

    calculated = false;
  }

*/
