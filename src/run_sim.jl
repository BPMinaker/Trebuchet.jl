using DifferentialEquations
using Plots; plotlyjs()

function run_sim(;g=9.81,
m1=1.0, #mass of counter weight
m2=0.3,  #mass of arm
m3=0.8,   #mass of frame
m4=0.023,   #mass of ball
l1=0.2,   #length of pivot to arm CG #'-' sign is deleted
l2=0.5,   #length of short arm
l3=0.2,   #length of counter weight
l4=1.0,   #length of long arm
l5=0.6,   #length of string
I=1/12*m2*(l2+l4)^2,   #inertia of arm
theta0=48.0,   #start angle
ReleaseAngle=45.0, #parameter name changed
flag1=false,
flag2=false
)

params=param_list(g,m1,m2,m3,m4,l1,l2,l3,l4,l5,I,theta0,ReleaseAngle,flag1,flag2)

println(params)

dummy(t,xx)=eqn_of_motion(t,xx,params)

tout=(0.0,4.0)
x0=[params.theta0*(pi/180), 0.0, 0.0, params.l5-cos(params.theta0*(pi/180))*params.l4, -params.l4*sin(params.theta0*(pi/180)), 0.0, 0.0, 0.0, 0.0, 0.0]
prob=ODEProblem(dummy,x0,tout)
sol=solve(prob,DP5(),saveat=0.0005,abstol=1e-8,reltol=1e-8)
plot(sol, vars=(4,5),show=true)
end
