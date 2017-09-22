using DifferentialEquations
using Plots; plotlyjs()

function run_sim(;g=9.81,
m1=1.0, #mass of counter weight
m2=0.3,  #mass of arm
m3=0.8,   #mass of frame
m4=0.023,   #mass of ball
l1=0.2,   #length of pivot to arm CG, +ve toward counter weight
l2=0.5,   #length of short arm
l3=0.2,   #length of counter weight
l4=1.0,   #length of long arm
l5=0.6,   #length of string
I=1/12*m2*(l2+l4)^2,   #inertia of arm
h=0.8,   # height of pivot from ground
ReleaseAngle=45.0, #parameter name changed
flag1=false,
flag2=false
)

params=param_list(g,m1,m2,m3,m4,l1,l2,l3,l4,l5,I,h,ReleaseAngle,flag1,flag2)

println("Counterweight mass=",m1)
println("Throwing arm mass=",m2)
println("Frame mass=",m3)
println("Projectile mass=",m4)
println("Throwing arm CG to pivot distance=",l1)
println("Short arm length=",l2)
println("Counterweight arm length=",l3)
println("Long arm length=",l4)
println("String length=",l5)
println("Arm moment of inertia=",I)
println("Pivot height=",h)
println("Inital arm angle=",asin(h/l4)*180/pi)

dummy(t,xx)=eqn_of_motion(t,xx,params)

tout=(0.0,4.0)
x0=[asin(h/l4), 0.0, 0.0,l5-sqrt(l4^2-h^2), -h, 0.0, 0.0, 0.0, 0.0, 0.0]
prob=ODEProblem(dummy,x0,tout)
sol=solve(prob,DP5(),saveat=0.005,abstol=1e-8,reltol=1e-8)
plot(sol, vars=(4,5),show=true)
end
