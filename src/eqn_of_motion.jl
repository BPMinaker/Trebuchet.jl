function eqn_of_motion(t,xx)

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

M=[(m1+m2)*l2^2+I+m2*l1^2 -sin(theta1-theta2)*m1*l2*l3 -(m1+m2)*l2*sin(theta1) 0 0
    -sin(theta1-theta2)*m1*l2*l3 m1*l3^2 m1*l3*cos(theta2) 0 0
    -(m1+m2)*l2*sin(theta1) m1*l3*cos(theta2) m1+m2+m3 0 0
    0 0 0 m4 0
    0 0 0 0 m4]

B=[2*l4*u*sin(theta1)-2*l4*x*sin(theta1)+2*l4*y*cos(theta1) 0 2*u-2*x-2*l4*cos(theta1) 2*x-2*u+2*l4*cos(theta1) 2*y+2*l4*sin(theta1)
    0 0 0 0 1]

f=[-cos(theta1-theta2)*m1*l2*l3*dtheta2^2-m1*g*l2*cos(theta1)-m2*g*l1*cos(theta1)
    cos(theta1-theta2)*m1*l2*l3*dtheta1^2-m1*g*l3*sin(theta2)
    (m1+m2)*l2*cos(theta1)*dtheta1^2+m1*l3*sin(theta2)*dtheta2^2
    0
    -m4*g]

NdBdq=[-(2*l4*sin(theta1)*du+2*l4*u*cos(theta1)*dtheta1-2*l4*sin(theta1)*dx-2*l4*x*cos(theta1)*dtheta1+2*l4*cos(theta1)*dy-2*l4*y*sin(theta1)*dtheta1)*dtheta1-(2*du-2*dx+2*l4*sin(theta1)*dtheta1)*du-(2*dx-2*du-2*l4*sin(theta1)*dtheta1)*dx-(2*dy+2*l4*cos(theta1)*dtheta1)*dy  0]

lmd=(B*(M\B'))\((B*(M\f))+(-NdBdq))
d2q=M\(-B'*lmd+f)




ra=atan2(dy,dx) #updated
if ((lmd(2)>0) || (flag1==1))
    lmd=(B[1,:]'*(M\B[1,:]))\((B[1,:]'*(M\f))+(-NdBdq[1,:]))
    d2q=M\((-B[1,:]*lmd)+f)
    params.flag1=1
    if ((ra<=Release_angle*(pi/180) && ra>0.00001) || (flag2==1))
        if (flag2==0)
            actual_Release_angle=ra*(180/pi)
            string_angle=asin((y+l4*sin(theta1))/l5)*(180/pi)
            Release_X_velocity=dx
            Release_Y_velocity=dy
            Release_combined_velocity=sqrt(dx^2+dy^2)
            Release_t=t
        end
        d2q=M\f
        params.flag2=1
    end
end




if ((dy<=-0.000001) && (flag3==0))
    params.max_height=y
    params.flag3=1
end


if((y<=(-l4*sin(theta0*(pi/180))-0.000001)) && (flag4==0))
    Landing_time=t
    params.Distance=x
    params.flag4=1
end

# pause()

xxdot[1]=dtheta1
xxdot[2]=dtheta2
xxdot[3]=du
xxdot[4]=dx
xxdot[5]=dy
xxdot[6:10]=d2q

end
