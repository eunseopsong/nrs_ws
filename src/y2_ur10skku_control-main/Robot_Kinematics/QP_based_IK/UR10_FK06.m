% Forward kinematics
function T06 = UR10_FK06(th1,th2,th3,th4,th5,th6)
global a1 a2 a3 a4 a5 a6 a7 a8 a9 a10;
global d1 d2 d3 d4 d5 d6 d7 d8 d9 d10;

T06 = [...
  cos(th6)*(sin(th1)*sin(th5) + cos(th2 + th3 + th4)*cos(th1)*cos(th5)) - sin(th2 + th3 + th4)*cos(th1)*sin(th6), - sin(th6)*(sin(th1)*sin(th5) + cos(th2 + th3 + th4)*cos(th1)*cos(th5)) - sin(th2 + th3 + th4)*cos(th1)*cos(th6),   cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5), d6*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + d4*sin(th1) + a3*cos(th2 + th3)*cos(th1) + a2*cos(th1)*cos(th2) + d5*sin(th2 + th3 + th4)*cos(th1);
- cos(th6)*(cos(th1)*sin(th5) - cos(th2 + th3 + th4)*cos(th5)*sin(th1)) - sin(th2 + th3 + th4)*sin(th1)*sin(th6),   sin(th6)*(cos(th1)*sin(th5) - cos(th2 + th3 + th4)*cos(th5)*sin(th1)) - sin(th2 + th3 + th4)*cos(th6)*sin(th1), - cos(th1)*cos(th5) - cos(th2 + th3 + th4)*sin(th1)*sin(th5), a3*cos(th2 + th3)*sin(th1) - d4*cos(th1) - d6*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + a2*cos(th2)*sin(th1) + d5*sin(th2 + th3 + th4)*sin(th1);
                                          cos(th2 + th3 + th4)*sin(th6) + sin(th2 + th3 + th4)*cos(th5)*cos(th6),                                           cos(th2 + th3 + th4)*cos(th6) - sin(th2 + th3 + th4)*cos(th5)*sin(th6),                               -sin(th2 + th3 + th4)*sin(th5),                                                                    d1 + a3*sin(th2 + th3) + a2*sin(th2) - d5*cos(th2 + th3 + th4) - d6*sin(th2 + th3 + th4)*sin(th5);
                                                                                                               0,                                                                                                                0,                                                            0,                                                                                                                                                                    1];
 
end