local kuka_connect = require('kuka_connect.core')

--kuka_connect.update_pose({ 0, -90, 90, 0, 90, 0 })
kuka_connect.update_pose({ 0, -90, 90, 0, 0, 0 })

kuka_connect.connect('127.0.0.1', 6008)
print('Connected to robot')

for i = 1, 10 do
  local pose = kuka_connect.get_pose()

  local msg = ''

  if pose ~= nil then
    for j = 1, 6 do
      msg = msg .. pose[j] .. ', '
    end
  end

  print(msg)

  os.execute('sleep 1')
end

print('Killing connection')
kuka_connect.stop_server()
print('Done')

