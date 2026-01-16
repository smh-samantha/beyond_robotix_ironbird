-- This script is an example of reading from the CAN bus
---@diagnostic disable: need-check-nil

-- Load CAN driver1. The first will attach to a protocol of 10, the 2nd to a protocol of 12
-- this allows the script to distinguish packets on two CAN interfaces
local driver1 = CAN:get_device(5)
local driver2 = CAN:get_device2(5)

if not driver1 and not driver2 then
   gcs:send_text(0,"No scripting CAN interfaces found")
   return
end

-- Only accept DroneCAN node status msg on second driver
-- node status is message ID 1043 (MagneticFieldStrengthHiRes)
-- Message ID is 16 bits left shifted by 8 in the CAN frame ID.
-- driver2:add_filter(uint32_t(0xFFFF) << 8, uint32_t(1043) << 8)

function show_frame(dnum, frame)
    local ID = frame:data(0) -- 8 bit ID
    local pressure = frame:data(1) | frame:data(2) | frame:data(3) | frame:data(4) -- 32 bit float
    local temperature = frame:data(5) | frame:data(6) | frame:data(7) | frame:data(8)  -- 32 bit float
    local pressure_var = frame:data(9) | frame:data(10) | frame:data(11) | frame:data(12)  -- 32 bit float
    gcs:send_text(0,string.format("CAN[%u] msg from pressure sensor " .. tostring(ID) .. ": %f, %f, %f", dnum, pressure, temperature, pressure_var))
end

function update()

   -- see if we got any frames
   if driver1 then
      frame = driver1:read_frame()
      if frame then
         show_frame(1, frame)
      end
   end
   if driver2 then
      frame = driver2:read_frame()
      if frame then
         show_frame(2, frame)
      end
   end

  return update, 10

end

return update()
