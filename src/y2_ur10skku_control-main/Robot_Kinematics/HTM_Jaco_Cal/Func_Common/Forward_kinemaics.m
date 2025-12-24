function Fk = Forward_kinemaics(alpha,a,d,theta)

if((abs(alpha) == 0)||(abs(alpha) == pi)||(abs(alpha) == pi/2))
    sim_cosAl = cos(alpha)*(abs(cos(alpha))>=0.0001);
    sim_sinAl = sin(alpha)*(abs(sin(alpha))>=0.0001);
else
    sim_cosAl = simplify(cos(alpha));
    sim_sinAl = simplify(sin(alpha));
end

if((abs(theta) == 0)||(abs(theta) == pi)||(abs(theta) == pi/2))
    sim_cosThe = cos(theta)*(abs(cos(theta))>=0.0001);
    sim_sinThe = sin(theta)*(abs(sin(theta))>=0.0001);
else
    sim_cosThe = simplify(cos(theta));
    sim_sinThe = simplify(sin(theta));
end

Fk = [sim_cosThe,           -sim_sinThe          , 0          ,a;
      sim_sinThe*sim_cosAl,sim_cosThe*sim_cosAl, -sim_sinAl,-d*sim_sinAl;
      sim_sinThe*sim_sinAl,sim_cosThe*sim_sinAl, sim_cosAl ,d*sim_cosAl;
      0                    ,0                    ,0           ,1];