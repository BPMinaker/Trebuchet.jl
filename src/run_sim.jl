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
ReleaseAngle=45.0, #parameter name changed
MaxHeight=0.0, #parameter name changed
Distance=0.0, #parameter name changed
flag1=false, #Test1
flag2=false,
flag3=false,
flag4=false #upto here
)

params=param_list(g,m1,m2,m3,m4,l1,l2,l3,l4,l5,I,theta0,ReleaseAngle,MaxHeight,Distance,flag1,flag2,flag3,flag4)

println(params)

dummy(t,xx)=eqn_of_motion(t,xx,params)
tout=(0.0,10.0)
x0=[theta0*(pi/180), 0.0, 0.0, l5-cos(theta0*(pi/180))*l4, -l4*sin(theta0*(pi/180)), 0.0, 0.0, 0.0, 0.0, 0.0]
prob=ODEProblem(dummy,x0, tout)
sol=solve(prob)

MaxHeight=params.MaxHeight #name changed
Distance=params.Distance #working now

end
