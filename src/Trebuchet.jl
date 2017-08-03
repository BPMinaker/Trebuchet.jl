module Trebuchet

export run_sim
export param_list

include("eqn_of_motion.jl")
include("run_sim.jl")

type param_list
  g::Float64
  m1::Float64
  m2::Float64
  m3::Float64
  m4::Float64
  l1::Float64
  l2::Float64
  l3::Float64
  l4::Float64
  l5::Float64
  I::Float64
  theta0::Float64
  ReleaseAngle::Float64
  MaxHeight::Float64
  Distance::Float64
  flag1::Bool #Test1
  flag2::Bool
#  flag3::Bool
#  flag4::Bool #upto here

function param_list(# ; distinguishes params based on the names
  g=0.0,
  m1=0.0, #mass of counter weight
  m2=0.0,  #mass of arm
  m3=0.0,   #mass of frame
  m4=0.0,   #mass of ball
  l1=0.0,   #length of pivot to arm CG
  l2=0.0,   #length of short arm
  l3=0.0,   #length of counter weight
  l4=0.0,   #length of long arm
  l5=0.0,   #length of string
  I=0.0,  #inertia of arm
  theta0=0.0,   #start angle
  ReleaseAngle=0.0, #parameter name changed
  MaxHeight=0.0, #parameter name changed
  Distance=0.0, #parameter name changed
  flag1=false,
  flag2=false
#  flag3=false,
#  flag4=false
)
  new(g,m1,m2,m3,m4,l1,l2,l3,l4,l5,I,theta0,ReleaseAngle,MaxHeight,Distance,flag1,flag2) #flag3,4 deleted
end #for function

end #for type
end #for module
