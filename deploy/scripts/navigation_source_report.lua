-- Optional Cube telemetry only. Requires verified scripting support/enabled Lua.
-- Reports the selected source; this does not prove GPS is actually being fused.
-- No source selection, arming, origin, mode or parameter changes.
local function update()
    gcs:send_named_float("NAV_SRC", ahrs:get_posvelyaw_source_set() + 1)
    return update, 200
end
return update, 200
