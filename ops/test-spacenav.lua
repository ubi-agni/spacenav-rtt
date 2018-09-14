require "rttlib"
require "rttros"

tc=rtt.getTC()
tcName=tc:getName()
print (tcName)
if tcName=="lua" then
  d=tc:getPeer("Deployer")
elseif tcName=="Deployer" then
  d=tc
end

d:import("rtt_rosnode")
d:import("rtt_roscomm")
d:import("rtt_geometry_msgs")
d:import("eigen_typekit")

d:import("spacenav")
d:loadComponent("sn", "cosima::hw::SpaceNavOrocos")

-- MANDATORY! Assign a FileDescriptorActivity to the component.
ret = d:setFileDescriptorActivity("sn", 0, 1, rtt.globals.ORO_SCHED_OTHER)
if ret then
  print ("success ")
end
-- Configure the component,
-- which initializes the device
-- and flashes LEDs when it was successful.
sn = d:getPeer("sn")
devicepath = sn:getProperty("devicePath")
devicepath:set("/dev/input/spacemouse")
sn:configure()

ros=rtt.provides("ros")
d:stream("sn.in_current_pose_port",ros:topic("/spacenav/input_pose"))
d:stream("sn.out_pose_port",ros:topic("/spacenav/output_pose"))


-- Start the component to get updates,
-- triggered by the FileDescriptorActivity.
sn:start()

