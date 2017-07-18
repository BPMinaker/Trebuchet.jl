using DifferentialEquations
using Plots

function run_sim(;g=9.81,
m1=45.0, #mass of counter weight
m2=5.0,  #mass of arm
m3=20.0,   #mass of frame
m4=0.15,   #mass of ball
l1=-0.75,   #length of pivot to arm CG
l2=0.5,   #length of short arm
l3=0.6,   #length of counter weight
l4=2.0,   #length of long arm
l5=2.0,   #length of string
I=2.6,   #inertia of arm
theta0=48.0,   #start angle
ReleaseAngle=45.0 #parameter name changed
)

params=param_list(g,m1,m2,m3,m4,l1,l2,l3,l4,l5,I,theta0,ReleaseAngle)

println(params)

flag1=0
flag2=0
flag3=0
flag4=0

MaxHeight=0 #parameter name changed
Distance=0

# WE NEED TO UPDATE FROM HERE
dummy(t,xx)=eqn_of_motion(t,xx,params)
tout=(0,10)
x0=[theta0*(pi/180), 0, 0, l5-cos(theta0*(pi/180))*l4, -l4*sin(theta0*(pi/180)), 0, 0, 0, 0, 0]
prob=ODEProblem(dummy,x0, tout) #eqn_of_motion needs to read parameters as well
sol=solve(prob)
# UPTO HERE

max_height=params.max_height #ignored for now
Distance=params.Distance #ignored for now

end
