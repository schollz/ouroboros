-- local pattern_time = require("pattern")
local GGrid = {}

function GGrid:new(args)
  local m = setmetatable({}, {__index=GGrid})
  local args = args == nil and {} or args

  m.grid_on = args.grid_on == nil and true or args.grid_on

  -- initiate the grid
  m.g = grid.connect()
  m.g.key = function(x, y, z)
    if m.grid_on then m:grid_key(x, y, z) end
  end
  print("grid columns: " .. m.g.cols)

  -- setup visual
  m.visual = {}
  m.grid_width = 16
  for i = 1, 8 do
    m.visual[i] = {}
    for j = 1, m.grid_width do m.visual[i][j] = 0 end
  end

  -- keep track of pressed buttons
  m.pressed_buttons = {}

  -- grid refreshing
  m.grid_refresh = metro.init()
  m.grid_refresh.time = 0.05
  m.grid_refresh.event = function()
    if m.grid_on then m:grid_redraw() end
  end
  m.grid_refresh:start()

  m.blinks = {{v=0, max=15}}

  return m
end

function GGrid:grid_key(x, y, z)
  self:key_press(y, x, z == 1)
  self:grid_redraw()
end

function GGrid:key_held_action(row, col)
  if col == 8 or col == 9 then
    -- enqueue recording
    local l = col == 8 and 1 or 2
    loopers[l]:rec_queue_up(row)
  end
end

function GGrid:key_press(row, col, on)
  local k = row .. "," .. col
  local time_on = 0
  if on then
    self.pressed_buttons[k] = 0
  else
    time_on = self.pressed_buttons[k]
    self.pressed_buttons[k] = nil
  end

  if (row >= 3 and row <= 8 and col >= 1 and col <= 6) or (row >= 3 and row <= 8 and col >= 11 and col <= 16) then
    local l = col < 9 and 1 or 2
    local r = row - 2
    local c = col - (l == 1 and 0 or 10)
    if params:get(l .. "note_pressing") == 1 then
      -- hold notes to play them
      if on then
        print(string.format("[grid] key_press %d on (hold)", l))
        loopers[l]:note_grid_on(r, c)
        loopers[l]:button_down(r, c)
      else
        print(string.format("[grid] key_press %d off (hold)", l))
        loopers[l]:note_grid_off(r, c)
        loopers[l]:button_up(r, c)
      end
    else
      -- toggle notes on off
      if on then
        print(string.format("[grid] key_press %d on (toggle)", l))
        if loopers[l]:is_note_on(r, c) then
          loopers[l]:note_grid_off(r, c)
        else
          loopers[l]:note_grid_on(r, c)
        end
        loopers[l]:button_down(r, c)
      else
        loopers[l]:button_up(r, c)
      end
    end
  elseif (col == 8 or col == 9) then
    local l = col < 9 and 1 or 2
    params:set(l .. "loop", row)
  elseif (col == 7 or col == 10) then
    local l = col < 9 and 1 or 2
    loopers[l]:pset("db", 9 - row)
  elseif row == 2 then
    if on then
      local l = col < 9 and 1 or 2
      local v = col
      if l == 2 then v = 17 - col end
      params:set(l .. "arp_option", v)
    end
  elseif row == 1 and (col == 1 or col == 16) then
    if on then
      local l = col < 9 and 1 or 2
      params:set(l .. "note_pressing", 3 - params:get(l .. "note_pressing"))
    end
  elseif row == 1 and (col == 2 or col == 15) then
    if on then
      local l = col < 9 and 1 or 2
      params:set(l .. "hold_change", 3 - params:get(l .. "hold_change"))
    end
  elseif row == 1 and ((col >= 4 and col <= 6) or (col >= 11 and col <= 13)) then
    if on then
      local l = col < 9 and 1 or 2
      if (col == 4 or col == 13) then
        params:delta(l .. "note_adjust", -1)
      elseif (col == 5 or col == 12) then
        params:set(l .. "note_adjust", 0)
      else
        params:delta(l .. "note_adjust", 1)
      end
    end
  end
end

function GGrid:get_visual()
  -- do blinking
  for i, v in ipairs(self.blinks) do
    self.blinks[i].v = self.blinks[i].v + 1
    if self.blinks[i].v > self.blinks[i].max then self.blinks[i].v = 0 end
  end

  -- clear visual
  for row = 1, 8 do
    for col = 1, self.grid_width do
      self.visual[row][col] = self.visual[row][col] - 1
      if self.visual[row][col] < 0 then self.visual[row][col] = 0 end
    end
  end

  -- illuminate rec queue / is recorded / current loop
  for l = 1, 2 do
    for i = 1, 8 do
      local j = l == 1 and 8 or 9
      self.visual[i][j] = 0
      if loopers[l]:is_in_rec_queue(i) then
        self.visual[i][j] = 7
      elseif loopers[l]:is_recorded(i) then
        self.visual[i][j] = 3
      end
      if loopers[l]:is_recording(i) then self.visual[i][j] = (self.blinks[1].v > 7 and 3 or 7) end
      if params:get(l .. "loop") == i then self.visual[i][j] = self.visual[i][j] + 8 end
    end
  end

  -- illuminate level
  for looper = 1, 2 do
    local v = 9 - loopers[looper]:pget("db")
    local col = looper == 1 and 7 or 10
    for row = 1, 8 do self.visual[row][col] = v <= row and 4 or 2 end
  end

  -- illuminate the arp speeds
  for l = 1, 2 do
    for i = 1, 6 do self.visual[2][l == 1 and i or 17 - i] = params:get(l .. "arp_option") == i and 4 or 2 end
  end

  -- illuminate toggle
  for l = 1, 2 do
    self.visual[1][l == 1 and 1 or 16] = params:get(l .. "note_pressing") == 1 and 3 or 10
    self.visual[1][l == 1 and 2 or 15] = params:get(l .. "hold_change") == 1 and 3 or 10
  end

  -- illuminate note adjustments
  for l = 1, 2 do
    local row = 1
    if params:get(l .. "note_adjust") == 0 then
      local col = l == 1 and 5 or 12
      self.visual[row][col] = 10
    elseif params:get(l .. "note_adjust") < 0 then
      local col = l == 1 and 4 or 13
      self.visual[row][col] = math.abs(params:get(l .. "note_adjust"))
    else
      local col = l == 1 and 6 or 11
      self.visual[row][col] = params:get(l .. "note_adjust")
    end
  end

  -- illuminate currently pressed button
  for k, _ in pairs(self.pressed_buttons) do
    self.pressed_buttons[k] = self.pressed_buttons[k] + 1
    local row, col = k:match("(%d+),(%d+)")
    row = tonumber(row)
    col = tonumber(col)
    if self.pressed_buttons[k] == 20 then -- 1 second
      print("[ggrid] holding ", row, col, "for >1 second")
      self:key_held_action(row, col)
    end
  end

  -- illuminate the notes
  for looper = 1, 2 do
    for i = 1, 6 do
      for j = 1, 6 do
        local row = i + 2
        local col = j + (looper == 1 and 0 or 10)
        self.visual[row][col] = 1
        if loopers[looper]:is_note_playing(i, j) then self.visual[row][col] = self.visual[row][col] + 7 end
        if loopers[looper]:is_note_on(i, j) then self.visual[row][col] = self.visual[row][col] + 6 end
      end
    end
  end

  if clock_chord > 0 and clock_chord < 16 then
    self.visual[7][clock_chord] = self.visual[7][clock_chord] + 2
    if self.visual[7][clock_chord] > 15 then self.visual[7][clock_chord] = 15 end
    self.visual[7][17 - clock_chord] = self.visual[7][17 - clock_chord] + 2
    if self.visual[7][17 - clock_chord] > 15 then self.visual[7][17 - clock_chord] = 15 end
  end

  if clock_beat > 0 and clock_beat < 16 then
    self.visual[8][clock_beat] = self.visual[8][clock_beat] + 2
    if self.visual[8][clock_beat] > 15 then self.visual[8][clock_beat] = 15 end
    self.visual[8][17 - clock_beat] = self.visual[8][17 - clock_beat] + 2
    if self.visual[8][17 - clock_beat] > 15 then self.visual[8][17 - clock_beat] = 15 end
  end
  return self.visual
end

function GGrid:grid_redraw()
  self.g:all(0)
  local gd = self:get_visual()
  local s = 1
  local e = self.grid_width
  local adj = 0
  for row = 1, 8 do for col = s, e do if gd[row][col] ~= 0 then self.g:led(col + adj, row, gd[row][col]) end end end
  self.g:refresh()
end

return GGrid
