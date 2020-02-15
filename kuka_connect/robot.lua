------------
--- Represent the connection to a real robot.
-- @module robot

local socket = require('socket')
local util = require('kuka_connect/util')
local template = require('kuka_connect/template')
local server = require('kuka_connect.core')

local RSI_PORT = 6008 -- RSI will accept updates on this port
local NET_TIMEOUT = 1

local Robot = util.class()

--- Construct a new Robot.
function Robot:_init(handle, ip, ghost)
  self.handle = handle
  self.ip = ip
  self.port = RSI_PORT
  self.lastKnownJointState = {
    a1 = 0,
    a2 = 0,
    a3 = 0,
    a4 = 0,
    a5 = 0,
    a6 = 0,
  }
  self.debugMode = true
  self.connected = false

  -- Render ghost robot by default
  if ghost == nil or ghost == true then
    self.renderGhost = true
  else
    self.renderGhost = false
  end
end

function Robot:run_script(script)
  if script == nil then
    error('Script is empty')
  end

  local client = socket.tcp()
  client:settimeout(NET_TIMEOUT)
  local success = client:connect(self.ip, self.port)

  if not success then
    error('Failed to open connection')
    return
  end

  -- Robot scripts won't be evaluated unless they end in a carriage return
  client:send(script .. '\r\n')
  client:close()
end

function ip_same_subnet(ip, ips)
  local targetParts = util.string_split(ip, '.')
  local sameSubnetIps = {}

  for i, otherIp in ipairs(ips) do
    local inSameSubnet = true
    local otherParts = util.string_split(otherIp, '.')

    for j = 1, 3 do
      if otherParts[j] ~= targetParts[j] then
        inSameSubnet = false
      end
    end

    if inSameSubnet then
      table.insert(sameSubnetIps, otherIp)
    end
  end

  return sameSubnetIps
end

function get_my_ip()
  return server.get_assigned_ips()
end

function Robot:connect()
  server.connect('172.31.1.150', 6008)

  self.connected = true

  if self.renderGhost then
    local existingGhostHandle = sim.getObjectHandle(sim.getObjectName(self.handle) .. '_ghost@silentError')

    -- Make ghost robot if it doesn't exist
    -- Handles case where robot is disconnected and then reconnected and this
    -- code is called twice
    if existingGhostHandle == -1 then
      self.ghostHandle = util.make_ghost_model(self.handle, true)
    else
      self.ghostHandle = existingGhostHandle
    end
  end
end

function Robot:get_joint_angles()
  local joints = sim.getObjectsInTree(self.handle, sim.object_joint_type)

  return {
    a1 = sim.getJointPosition(joints[1]),
    a2 = sim.getJointPosition(joints[2]),
    a3 = sim.getJointPosition(joints[3]),
    a4 = sim.getJointPosition(joints[4]),
    a5 = sim.getJointPosition(joints[5]),
    a6 = sim.getJointPosition(joints[6]),
  }
end

function Robot:update_ghost()
  if self.renderGhost == false then
    return
  end

  if self.ghostHandle == nil or self.lastKnownJointState == nil then
    return
  end

  local ghostJoints = sim.getObjectsInTree(self.ghostHandle, sim.object_joint_type)

  sim.setJointPosition(ghostJoints[1], self.lastKnownJointState.a1)
  sim.setJointPosition(ghostJoints[2], self.lastKnownJointState.a2)
  sim.setJointPosition(ghostJoints[3], self.lastKnownJointState.a3)
  sim.setJointPosition(ghostJoints[4], self.lastKnownJointState.a4)
  sim.setJointPosition(ghostJoints[5], self.lastKnownJointState.a5)
  sim.setJointPosition(ghostJoints[6], self.lastKnownJointState.a6)
end

-- Takes a hash table with angles in radians for keys
-- base, shoulder, elbow, wrist1, wrist2, and wrist3
-- and tells the robot to servo its joints to match the specified angles.
-- Defaults to the joint angles of the robot in the V-REP scene if none specified.
function Robot:servo_to(jointAngles, doMove)
  if not self.connected then
    return
  end

  if jointAngles == nil then
    jointAngles = self:get_joint_angles()
  end

  local pose = {
    jointAngles.a1,
    jointAngles.a2,
    jointAngles.a3,
    jointAngles.a4,
    jointAngles.a5,
    jointAngles.a6,
  }

  server.update_pose(pose)

  local a1, a2, a3, a4, a5, a6 = server.get_pose()

  if a1 ~= nil then
    self.lastKnownJointState.a1 = a1
    self.lastKnownJointState.a2 = a2
    self.lastKnownJointState.a3 = a3
    self.lastKnownJointState.a4 = a4
    self.lastKnownJointState.a5 = a5
    self.lastKnownJointState.a6 = a6

    self:update_ghost()
  end
end

function Robot:disconnect()
  if not self.connected then
    return
  end

  server.disconnect()

  self.connected = false
end

return Robot
