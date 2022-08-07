function f_sym = quadrot_nl(z,phi,theta,psi,vx,vy,vz,v_phi,v_theta,v_psi,s,d,t,u1,u2,u3,u4,K,T)
%QUADROT_NL
%    F_SYM = QUADROT_NL(Z,PHI,THETA,PSI,VX,VY,VZ,V_PHI,V_THETA,V_PSI,S,D,T,U1,U2,U3,U4,K,T)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    29-Jul-2022 16:22:01

t2 = cos(phi);
t3 = cos(psi);
t4 = cos(t);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(t);
t8 = sin(theta);
t9 = K.*d;
t10 = t4.*vx;
t11 = t7.*vy;
t12 = t9-1.0;
t13 = 1.0./t12;
t14 = t10+t11;
f_sym = [z+T.*vz;phi+T.*v_phi;theta+T.*v_theta;psi+T.*v_psi;vx-T.*(vx.*5.0e-1-u1.*(t5.*t6.*2.0+t2.*t3.*t8.*2.0).*1.0).*1.0;vy-T.*(vy.*5.0e-1+u1.*(t3.*t5.*2.0-t2.*t6.*t8.*2.0)).*1.0;vz-T.*(vz.*5.0e-1-t2.*u1.*cos(theta).*2.0+9.809999999997672).*1.0;v_phi+T.*(u2.*1.587301587299444e+2-v_psi.*v_theta.*9.047619047614717e-1);v_theta+T.*(u3.*1.587301587299444e+2+v_phi.*v_psi.*9.047619047614717e-1);v_psi+T.*u4.*8.333333333325572e+1;s-T.*t13.*t14.*1.0;d-T.*(t7.*vx.*1.0-t4.*vy);t-K.*T.*t13.*t14.*1.0];
