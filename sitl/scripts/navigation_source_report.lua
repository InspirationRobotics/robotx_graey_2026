-- Simulator copy of the repo's read-only selected EKF source reporter.
local function update()
    gcs:send_named_float("NAV_SRC", ahrs:get_posvelyaw_source_set() + 1)
    return update, 200
end
return update, 200
