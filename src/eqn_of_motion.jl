function eqn_of_motion(t,xx,params)

theta1=xx[1] #[] should be used for array
theta2=xx[2]
u=xx[3]
x=xx[4]
y=xx[5]
dtheta1=xx[6]
dtheta2=xx[7]
du=xx[8]
dx=xx[9]
dy=xx[10]

g=params.g
m1=params.m1 #mass of counter weight
m2=params.m2  #mass of arm
m3=params.m3   #mass of frame
m4=params.m4   #mass of ball
l1=params.l1   #length of pivot to arm CG
l2=params.l2   #length of short arm
l3=params.l3   #length of counter weight
l4=params.l4   #length of long arm
l5=params.l5   #length of string
I=params.I  #inertia of arm
theta0=params.theta0   #start angle
ReleaseAngle=params.ReleaseAngle #parameter name changed
# MaxHeight=params.MaxHeight #parameter name changed
# Distance=params.Distance #parameter name changed

M=[(params.m1+params.m2)*params.l2^2+params.I+params.m2*params.l1^2 -sin(theta1-theta2)*params.m1*params.l2*params.l3 -(params.m1+params.m2)*params.l2*sin(theta1) 0 0
    -sin(theta1-theta2)*params.m1*params.l2*params.l3 params.m1*params.l3^2 params.m1*params.l3*cos(theta2) 0 0
    -(params.m1+params.m2)*params.l2*sin(theta1) params.m1*params.l3*cos(theta2) params.m1+params.m2+params.m3 0 0
    0 0 0 params.m4 0
    0 0 0 0 params.m4]

B=[2*params.l4*u*sin(theta1)-2*params.l4*x*sin(theta1)+2*params.l4*y*cos(theta1) 0 2*u-2*x-2*params.l4*cos(theta1) 2*x-2*u+2*params.l4*cos(theta1) 2*y+2*params.l4*sin(theta1)
    0 0 0 0 1]

f=[-cos(theta1-theta2)*params.m1*params.l2*params.l3*dtheta2^2-params.m1*params.g*params.l2*cos(theta1)-params.m2*params.g*params.l1*cos(theta1)
    cos(theta1-theta2)*params.m1*params.l2*params.l3*dtheta1^2-params.m1*params.g*params.l3*sin(theta2)
    (params.m1+params.m2)*params.l2*cos(theta1)*dtheta1^2+params.m1*params.l3*sin(theta2)*dtheta2^2
    0
    -params.m4*params.g]

NdBdq=[-(2*params.l4*sin(theta1)*du+2*params.l4*u*cos(theta1)*dtheta1-2*params.l4*sin(theta1)*dx-2*params.l4*x*cos(theta1)*dtheta1+2*params.l4*cos(theta1)*dy-2*params.l4*y*sin(theta1)*dtheta1)*dtheta1-(2*du-2*dx+2*params.l4*sin(theta1)*dtheta1)*du-(2*dx-2*du-2*params.l4*sin(theta1)*dtheta1)*dx-(2*dy+2*params.l4*cos(theta1)*dtheta1)*dy
0]

lmd=(B*(M\B'))\((B*(M\f))-NdBdq)
d2q=M\(-B'*lmd+f)

ra=atan2(dy,dx)
if (lmd[2]>0 || params.flag1)
    lmd=(B[1,:]'*(M\B[1,:]))\((B[1,:]'*(M\f))+(-NdBdq[1,:]'))
    d2q=M\((-B[1,:]*lmd)+f)
    if(~params.flag1)
      println("Lift Off Time= ",t)
    end
    params.flag1=true
    if ((ra<=params.ReleaseAngle*(pi/180) && ra>0.00001) || params.flag2)
        if (~params.flag2)
            actual_ReleaseAngle=ra*(180/pi)
            string_angle=asin((y+params.l4*sin(theta1))/params.l5)*(180/pi)
            Release_X_velocity=dx
            Release_Y_velocity=dy
            Release_combined_velocity=sqrt(dx^2+dy^2)
            Release_t=t
            Release_x=x
            Release_y=y
            println("Actual Release Angle= ", actual_ReleaseAngle)
            println("String Angle= ",string_angle)
            println("Release X velocity= ",Release_X_velocity)
            println("Release Y velocity= ",Release_Y_velocity)
            println("Release Combined velocity= ", Release_combined_velocity)
            println("Release time= ",Release_t)
            println("Release x= ",Release_x)
            println("Release y= ",Release_y)
        end
        d2q=M\f
        params.flag2=true
    end
end

xxdot=[dtheta1, dtheta2, du, dx, dy, d2q[1], d2q[2], d2q[3], d2q[4], d2q[5]]

xxdot
end
