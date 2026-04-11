/**
 * In-browser mission simulation — mirrors Python resilient-navigator behavior
 * (Environment, D* Lite Fig.5, A*, FSM, faults, mission loop). Deterministic.
 */
(function (global) {
  'use strict';

  var FREE = 0, OBSTACLE = 1, HAZARD = 2;
  var INF = Infinity;
  var SQRT2 = Math.SQRT2;

  /** FNV-1a hash of JSON config — seed for deterministic PRNG (Easy Open). */
  function hashConfig(config) {
    var s = JSON.stringify(config);
    var h = 2166136261 >>> 0;
    for (var i = 0; i < s.length; i++) {
      h ^= s.charCodeAt(i);
      h = Math.imul(h, 16777619);
    }
    return h >>> 0;
  }

  function mulberry32(a) {
    return function () {
      a |= 0;
      a = (a + 0x6d2b79f5) | 0;
      var t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  /** Standard normal via Box–Muller; u1 must be in (0,1]. */
  function randn01(rng) {
    var u1 = 0;
    while (u1 <= 1e-12) u1 = rng();
    var u2 = rng();
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  }

  function hypot(dx, dy) { return Math.hypot(dx, dy); }
  function hFun(a, b) { return hypot(a[0] - b[0], a[1] - b[1]); }
  function isClose(a, b) { return Math.abs(a - b) < 1e-9; }
  function deepCopy(o) { return JSON.parse(JSON.stringify(o)); }

  function heapSiftUp(heap, i, cmp) {
    while (i > 0) {
      var p = (i - 1) >> 1;
      if (cmp(heap[i], heap[p]) >= 0) break;
      var t = heap[i]; heap[i] = heap[p]; heap[p] = t;
      i = p;
    }
  }
  function heapSiftDown(heap, i, cmp) {
    var n = heap.length;
    while (true) {
      var l = i * 2 + 1, r = l + 1, m = i;
      if (l < n && cmp(heap[l], heap[m]) < 0) m = l;
      if (r < n && cmp(heap[r], heap[m]) < 0) m = r;
      if (m === i) break;
      var t = heap[i]; heap[i] = heap[m]; heap[m] = t;
      i = m;
    }
  }
  function heapPush(heap, item, cmp) {
    heap.push(item);
    heapSiftUp(heap, heap.length - 1, cmp);
  }
  function heapPop(heap, cmp) {
    if (heap.length === 0) return null;
    var top = heap[0];
    var last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      heapSiftDown(heap, 0, cmp);
    }
    return top;
  }
  /** Lexicographic (a0,a1,seq) compare for D* heap entries */
  function cmpKey(a, b) {
    if (a[0] !== b[0]) return a[0] < b[0] ? -1 : 1;
    if (a[1] !== b[1]) return a[1] < b[1] ? -1 : 1;
    return a[2] - b[2];
  }

  function Environment(config) {
    var envCfg = config.environment;
    var self = this;
    this._w = envCfg.grid_size[0] | 0;
    this._h = envCfg.grid_size[1] | 0;
    this._grid = new Int8Array(this._w * this._h);
    this._currentTimestep = -1;
    this._dynamicHazards = envCfg.dynamic_hazards || [];
    this._triggeredIds = {};
    this._changeEvents = [];

    var staticCells = [];
    (envCfg.static_obstacles || []).forEach(function (ob) {
      var x0 = ob.x | 0, y0 = ob.y | 0, ww = ob.width | 0, hh = ob.height | 0;
      for (var x = x0; x < x0 + ww; x++) {
        for (var y = y0; y < y0 + hh; y++) {
          if (x >= 0 && x < self._w && y >= 0 && y < self._h) {
            staticCells.push([x, y]);
            self._grid[y * self._w + x] = OBSTACLE;
          }
        }
      }
    });
    if (staticCells.length) this._changeEvents.push([0, staticCells]);
  }
  Environment.prototype._idx = function (x, y) { return y * this._w + x; };
  Environment.prototype.update = function (t) { this._currentTimestep = t; };
  Environment.prototype.getGrid = function () { return this._grid; };
  Object.defineProperty(Environment.prototype, 'current_timestep', { get: function () { return this._currentTimestep; } });
  Object.defineProperty(Environment.prototype, 'width', { get: function () { return this._w; } });
  Object.defineProperty(Environment.prototype, 'height', { get: function () { return this._h; } });

  Environment.prototype.is_blocked = function (x, y, exempt, ignoreHazardCells) {
    if (exempt != null && (x | 0) === exempt[0] && (y | 0) === exempt[1]) return false;
    if (x < 0 || y < 0 || x >= this._w || y >= this._h) return true;
    var v = this._grid[this._idx(x, y)];
    if (ignoreHazardCells) return v === OBSTACLE;
    return v !== FREE;
  };

  Environment.prototype.get_neighbors = function (x, y, ignoreHazardCells) {
    var ign = !!ignoreHazardCells;
    var out = [];
    for (var dx = -1; dx <= 1; dx++) {
      for (var dy = -1; dy <= 1; dy++) {
        if (dx === 0 && dy === 0) continue;
        var nx = x + dx, ny = y + dy;
        if (!this.is_blocked(nx, ny, null, ign)) out.push([nx, ny]);
      }
    }
    return out;
  };

  Environment.prototype._diskCells = function (cx, cy, radius) {
    var r = radius, ri = Math.ceil(r), out = [];
    for (var x = cx - ri; x <= cx + ri; x++) {
      for (var y = cy - ri; y <= cy + ri; y++) {
        if (x >= 0 && x < this._w && y >= 0 && y < this._h && hypot(x - cx, y - cy) <= r + 1e-9)
          out.push([x, y]);
      }
    }
    return out;
  };

  Environment.prototype._fillRectCells = function (x0, y0, ww, hh) {
    var out = [];
    for (var x = x0; x < x0 + ww; x++) {
      for (var y = y0; y < y0 + hh; y++) {
        if (x >= 0 && x < this._w && y >= 0 && y < this._h) out.push([x, y]);
      }
    }
    return out;
  };

  Environment.prototype._applyDynamicHazard = function (hz) {
    var pos = hz.position;
    var px = pos.x | 0, py = pos.y | 0;
    var cells = [];
    if (hz.type === 'no_fly_zone') {
      cells = this._diskCells(px, py, +hz.radius);
      for (var i = 0; i < cells.length; i++) {
        var c = cells[i];
        this._grid[this._idx(c[0], c[1])] = HAZARD;
      }
    } else if (hz.type === 'obstacle_spawn') {
      cells = this._fillRectCells(px, py, hz.width | 0, hz.height | 0);
      for (var j = 0; j < cells.length; j++) {
        var d = cells[j];
        this._grid[this._idx(d[0], d[1])] = OBSTACLE;
      }
    } else throw new Error('Unknown hazard ' + hz.type);
    return cells;
  };

  Environment.prototype.trigger_dynamic_hazard_by_id = function (hid, eventTimestep) {
    for (var i = 0; i < this._dynamicHazards.length; i++) {
      var hz = this._dynamicHazards[i];
      if (String(hz.id) !== String(hid)) continue;
      if (this._triggeredIds[hid]) return false;
      this._triggeredIds[hid] = true;
      var cells = this._applyDynamicHazard(hz);
      var t = eventTimestep != null ? eventTimestep | 0 : (this._currentTimestep >= 0 ? this._currentTimestep : 0);
      if (cells.length) this._changeEvents.push([t, cells]);
      return true;
    }
    return false;
  };

  Environment.prototype.get_changes_since = function (lastT) {
    var set = {};
    for (var i = 0; i < this._changeEvents.length; i++) {
      var ev = this._changeEvents[i];
      if (ev[0] > lastT) {
        ev[1].forEach(function (c) { set[c[0] + ',' + c[1]] = true; });
      }
    }
    var keys = Object.keys(set).map(function (s) {
      var p = s.split(',');
      return [+p[0], +p[1]];
    });
    keys.sort(function (a, b) { return a[0] !== b[0] ? a[0] - b[0] : a[1] - b[1]; });
    return keys;
  };

  function Drone(config) {
    var d = config.drone;
    this.position = [d.start_position[0] | 0, d.start_position[1] | 0];
    this.heading = 0;
    this.battery = +d.battery_capacity;
    this.sensor_noise_level = +d.sensor_noise_baseline;
    this._sensorRange = d.sensor_range | 0;
    this._batCap = +d.battery_capacity;
    this._drain = +d.battery_drain_rate;
    this._maxNoise = +d.max_sensor_noise;
    this._baseNoise = +d.sensor_noise_baseline;
    this._gw = config.environment.grid_size[0] | 0;
    this._gh = config.environment.grid_size[1] | 0;
    this._speedScale = 1;
    /** Set in runSimulation — mulberry32 instance for reproducible noise / move skips. */
    this._rng = null;
  }
  Drone.prototype.get_effective_sensor_range = function () {
    var ratio = this.sensor_noise_level / Math.max(this._maxNoise, 1e-9);
    if (ratio > 1) ratio = 1;
    return this._sensorRange * (1.0 - 0.5 * ratio);
  };
  Drone.prototype.get_position_estimate = function (env) {
    var rng = this._rng;
    if (!rng) throw new Error('drone._rng not set');
    var std = this.sensor_noise_level * 5.0;
    var nx = Math.round(randn01(rng) * std);
    var ny = Math.round(randn01(rng) * std);
    var px = this.position[0] | 0, py = this.position[1] | 0;
    var gw = env._w, gh = env._h;
    var estX = Math.max(0, Math.min(gw - 1, px + nx));
    var estY = Math.max(0, Math.min(gh - 1, py + ny));
    if (!env.is_blocked(estX, estY)) return [estX, estY];
    var maxR = Math.max(gw, gh);
    for (var rad = 1; rad < maxR; rad++) {
      for (var dx = -rad; dx <= rad; dx++) {
        for (var dy = -rad; dy <= rad; dy++) {
          if (Math.max(Math.abs(dx), Math.abs(dy)) !== rad) continue;
          var ex = estX + dx, ey = estY + dy;
          if (ex >= 0 && ex < gw && ey >= 0 && ey < gh && !env.is_blocked(ex, ey)) return [ex, ey];
        }
      }
    }
    return [px, py];
  };
  Drone.prototype.get_telemetry = function () {
    return {
      battery: this.battery,
      battery_capacity: this._batCap,
      speed_scale: this._speedScale,
      sensor_noise_level: this.sensor_noise_level
    };
  };
  Drone.prototype.set_speed_scale = function (s) {
    this._speedScale = Math.max(0.1, Math.min(1, s));
  };
  Drone.prototype.sense = function (env) {
    var rng = this._rng;
    var x = this.position[0], y = this.position[1];
    var obs = [];
    var r = this.get_effective_sensor_range();
    var ri = Math.ceil(r);
    for (var cx = x - ri; cx <= x + ri; cx++) {
      for (var cy = y - ri; cy <= y + ri; cy++) {
        if (cx < 0 || cy < 0 || cx >= env._w || cy >= env._h) continue;
        if (hypot(cx - x, cy - y) > r + 1e-9) continue;
        if (env._grid[env._idx(cx, cy)] !== FREE) obs.push([cx, cy]);
      }
    }
    var sigma = Math.max(this.sensor_noise_level, 0) * 0.5;
    var pex = x, pey = y;
    if (sigma > 0 && rng) {
      pex = x + randn01(rng) * sigma;
      pey = y + randn01(rng) * sigma;
    }
    return {
      observed_obstacles: obs,
      position_estimate: [pex, pey],
      effective_sensor_range: r
    };
  };
  Drone.prototype.apply_sensor_degradation = function (sev) {
    this.sensor_noise_level = Math.min(this._maxNoise, this.sensor_noise_level + sev);
  };
  Drone.prototype.restore_sensor = function (amt) {
    this.sensor_noise_level = Math.max(this._baseNoise, this.sensor_noise_level - amt);
  };
  Drone.prototype.move = function (tx, ty) {
    var rng = this._rng;
    var x = this.position[0], y = this.position[1];
    tx |= 0; ty |= 0;
    if (x === tx && y === ty) return;
    if (this._speedScale < 1.0 && rng) {
      if (rng() > this._speedScale) {
        this.battery = Math.max(0, this.battery - this._drain * this._speedScale * 0.5);
        return;
      }
    }
    var best = null, bestD = INF;
    for (var nx = x - 1; nx <= x + 1; nx++) {
      for (var ny = y - 1; ny <= y + 1; ny++) {
        if (nx === x && ny === y) continue;
        if (nx < 0 || ny < 0 || nx >= this._gw || ny >= this._gh) continue;
        var dd = hypot(tx - nx, ty - ny);
        if (dd < bestD || (isClose(dd, bestD) && (!best || nx < best[0] || (nx === best[0] && ny < best[1])))) {
          bestD = dd; best = [nx, ny];
        }
      }
    }
    if (!best) return;
    this.position = best;
    this.heading = Math.atan2(best[1] - y, best[0] - x);
    this.battery = Math.max(0, this.battery - this._drain * this._speedScale);
  };

  function MissionManager(config) {
    var m = config.mission;
    this._wps = m.waypoints.map(function (w) { return [w[0] | 0, w[1] | 0]; });
    this._home = [m.home_position[0] | 0, m.home_position[1] | 0];
    this._tol = m.waypoint_tolerance != null ? m.waypoint_tolerance | 0 : 1;
    this.current_waypoint_index = 0;
    this.waypoints_completed = 0;
    this.mission_status = 'in_progress';
    this._abortedTarget = null;
  }
  MissionManager.prototype._cheb = function (a, b) {
    return Math.max(Math.abs(a[0] - b[0]), Math.abs(a[1] - b[1]));
  };
  MissionManager.prototype.get_current_target = function () {
    if (this.mission_status === 'aborted' && this._abortedTarget) return this._abortedTarget;
    if (this.current_waypoint_index >= this._wps.length)
      return this._wps.length ? this._wps[this._wps.length - 1] : this._home;
    return this._wps[this.current_waypoint_index];
  };
  MissionManager.prototype.update = function (pos) {
    if (this.mission_status !== 'in_progress') return;
    if (this.current_waypoint_index >= this._wps.length) {
      this.mission_status = 'completed';
      return;
    }
    var t = this._wps[this.current_waypoint_index];
    if (this._cheb(pos, t) <= this._tol) {
      this.waypoints_completed++;
      this.current_waypoint_index++;
      if (this.current_waypoint_index >= this._wps.length) this.mission_status = 'completed';
    }
  };
  MissionManager.prototype.abort_mission = function () {
    this.mission_status = 'aborted';
    this._abortedTarget = this._home;
  };
  MissionManager.prototype.skip_waypoint = function () {
    if (this.mission_status !== 'in_progress') return;
    if (this.current_waypoint_index >= this._wps.length) return;
    this.current_waypoint_index++;
    if (this.current_waypoint_index >= this._wps.length) this.mission_status = 'completed';
  };
  MissionManager.prototype.get_progress = function () {
    var total = this._wps.length;
    var idx = Math.min(this.current_waypoint_index, total);
    return {
      mission_status: this.mission_status,
      current_waypoint_index: this.current_waypoint_index,
      waypoints_total: total,
      waypoints_completed: this.waypoints_completed,
      current_target: this.get_current_target(),
      home_position: this._home
    };
  };

  function FaultInjector(faultConfig) {
    this._schedule = (faultConfig && faultConfig.schedule) ? faultConfig.schedule.slice() : [];
    this._fired = {};
    this._episode = null;
    this._lastStepEvents = [];
    this._activeFaults = [];
  }
  FaultInjector.prototype.get_last_step_events = function () { return this._lastStepEvents.slice(); };
  FaultInjector.prototype.update = function (timestep, drone, env) {
    this._lastStepEvents = [];
    var ep = this._episode;
    if (ep && timestep >= ep.expire_at) {
      drone.restore_sensor(ep.severity);
      this._lastStepEvents.push({ type: 'fault_expired', timestep: timestep, fault_type: 'sensor_degradation' });
      this._episode = null;
    }
    for (var i = 0; i < this._schedule.length; i++) {
      if (this._fired[i]) continue;
      var entry = this._schedule[i];
      if ((entry.timestep | 0) !== timestep) continue;
      this._fired[i] = true;
      if (entry.type === 'sensor_degradation') {
        var sev = +entry.severity;
        var dur = entry.duration | 0;
        drone.apply_sensor_degradation(sev);
        this._episode = { severity: sev, expire_at: timestep + dur };
        this._lastStepEvents.push({
          type: 'fault_injected', timestep: timestep, fault_type: 'sensor_degradation', severity: sev
        });
      } else if (entry.type === 'environmental_hazard') {
        var hid = String(entry.hazard_id);
        if (env.trigger_dynamic_hazard_by_id(hid, timestep)) {
          this._lastStepEvents.push({
            type: 'hazard_spawned', timestep: timestep, id: hid,
            description: String(entry.description || '')
          });
        }
      }
    }
    this._refreshActive(timestep);
  };
  FaultInjector.prototype._refreshActive = function (timestep) {
    var out = [];
    var ep = this._episode;
    if (ep) {
      out.push({
        type: 'sensor_degradation',
        severity: ep.severity,
        remaining_duration: Math.max(0, ep.expire_at - timestep),
        description: 'active',
        timestep_activated: timestep
      });
    }
    for (var i = 0; i < this._schedule.length; i++) {
      if (!this._fired[i]) continue;
      var entry = this._schedule[i];
      if (entry.type !== 'environmental_hazard') continue;
      out.push({
        type: 'environmental_hazard',
        hazard_id: String(entry.hazard_id),
        description: String(entry.description || ''),
        timestep_activated: entry.timestep | 0
      });
    }
    this._activeFaults = out;
  };
  FaultInjector.prototype.get_active_faults = function () {
    return this._activeFaults ? this._activeFaults.slice() : [];
  };

  function FaultDetector(algo) {
    this._warn = +algo.sensor_degradation_threshold;
    this._crit = +algo.critical_fault_threshold;
  }
  FaultDetector.prototype.evaluate = function (drone, env, planner) {
    var px = drone.position[0] | 0, py = drone.position[1] | 0;
    if (env.is_blocked(px, py)) {
      var nbsEg = env.get_neighbors(px, py, true);
      if (!nbsEg.length) {
        return { level: 'CRITICAL', type: 'position_trapped', details: 'surrounded' };
      }
      var path0 = planner.get_full_path();
      var escapePlanned = false;
      if (path0.length >= 2) {
        var n0 = [path0[0][0] | 0, path0[0][1] | 0];
        var n1 = [path0[1][0] | 0, path0[1][1] | 0];
        var j, nb;
        if (n0[0] === px && n0[1] === py) {
          for (j = 0; j < nbsEg.length; j++) {
            nb = nbsEg[j];
            if (nb[0] === n1[0] && nb[1] === n1[1] && !env.is_blocked(n1[0], n1[1], null, true)) {
              escapePlanned = true;
              break;
            }
          }
        }
      }
      if (!escapePlanned) {
        return { level: 'CRITICAL', type: 'position_compromised', details: 'in hazard' };
      }
    }
    var noise = drone.sensor_noise_level;
    var path = planner.get_full_path();
    var pathBlocked = !path.length;
    if (!pathBlocked) {
      for (var i = 0; i < path.length; i++) {
        var ix = path[i][0] | 0, iy = path[i][1] | 0;
        if (ix === px && iy === py) continue;
        if (env.is_blocked(ix, iy)) { pathBlocked = true; break; }
      }
    }
    var sensorLevel = noise >= this._crit ? 'CRITICAL' : (noise >= this._warn ? 'WARNING' : 'NOMINAL');
    if (sensorLevel !== 'NOMINAL' && pathBlocked) {
      return { level: sensorLevel === 'CRITICAL' ? 'CRITICAL' : 'WARNING', type: 'combined', details: 'combined' };
    }
    if (pathBlocked) return { level: 'WARNING', type: 'path_blocked', details: 'path' };
    if (sensorLevel === 'CRITICAL') return { level: 'CRITICAL', type: 'sensor_degradation', details: 'crit' };
    if (sensorLevel === 'WARNING') return { level: 'WARNING', type: 'sensor_degradation', details: 'warn' };
    return { level: 'NOMINAL', type: 'none', details: 'ok' };
  };

  function FSM(missionCfg, algoCfg) {
    this._warn = +algoCfg.sensor_degradation_threshold;
    this._sensorCrit = +algoCfg.critical_fault_threshold;
    this._batFrac = 0.05;
    this._state = 'NOMINAL';
    this.requires_replan = false;
    this.emergency_escape = false;
    this._log = [];
    this._prevNoise = null;
    this._lastTs = 0;
  }
  FSM.prototype.get_state = function () { return this._state; };
  FSM.prototype.get_transition_log = function () { return this._log.slice(); };
  FSM.prototype.record_waypoint_skipped = function (timestep, reason) {
    this._log.push({ timestep: timestep, from_state: this._state, to_state: this._state, trigger: 'waypoint_skipped_' + reason });
  };
  FSM.prototype.check_depleted_battery_abort = function (timestep, drone, mm) {
    if (this._state === 'ABORT') return;
    if (drone.get_telemetry().battery > 0) return;
    this._transition(timestep, this._state, 'ABORT', 'battery_depleted', drone, mm);
  };
  FSM.prototype._battery_low = function (drone) {
    var bat = drone.get_telemetry().battery;
    var cap = drone.get_telemetry().battery_capacity;
    if (bat <= 0) return true;
    return bat <= this._batFrac * cap;
  };
  FSM.prototype._transition = function (timestep, oldS, newS, trigger, drone, mm) {
    if (oldS === newS) return;
    this._log.push({ timestep: timestep, from_state: oldS, to_state: newS, trigger: trigger });
    this._state = newS;
    if (newS === 'DEGRADED' && drone) drone.set_speed_scale(0.75);
    if (newS === 'NOMINAL' && drone) drone.set_speed_scale(1.0);
    if (newS === 'SAFE_MODE' && mm) mm.abort_mission();
  };
  FSM.prototype.update = function (timestep, fs, drone, mm) {
    this._lastTs = timestep;
    if (this._state === 'ABORT') {
      this._prevNoise = drone.sensor_noise_level;
      return;
    }
    if (fs.type === 'position_trapped') {
      this._transition(timestep, this._state, 'ABORT', 'position_trapped', drone, mm);
      this.requires_replan = false;
      this.emergency_escape = false;
      this._prevNoise = drone.sensor_noise_level;
      return;
    }
    if (this._battery_low(drone)) {
      this._transition(timestep, this._state, 'ABORT', 'battery_below_5pct', drone, mm);
      this._prevNoise = drone.sensor_noise_level;
      return;
    }
    if (fs.type === 'position_compromised') {
      if (this._state !== 'REPLANNING') {
        this._transition(timestep, this._state, 'REPLANNING', 'emergency_escape', drone, mm);
      }
      this.requires_replan = true;
      this.emergency_escape = true;
      this._prevNoise = drone.sensor_noise_level;
      return;
    }
    if (this._state === 'NOMINAL') this._fromNominal(timestep, fs, drone, mm);
    else if (this._state === 'DEGRADED') this._fromDegraded(timestep, fs, drone, mm);
    else if (this._state === 'REPLANNING') this._fromReplanning(timestep, fs, drone, mm);
    else if (this._state === 'SAFE_MODE') {}
    this._prevNoise = drone.sensor_noise_level;
  };
  FSM.prototype._fromNominal = function (timestep, fs, drone, mm) {
    if (fs.type === 'path_blocked') {
      this._transition(timestep, 'NOMINAL', 'REPLANNING', 'path_blocked', drone, mm);
      this.requires_replan = true;
      return;
    }
    if (fs.level === 'CRITICAL') {
      this._transition(timestep, 'NOMINAL', 'SAFE_MODE', 'sensor_critical', drone, mm);
      return;
    }
    if (fs.level === 'WARNING') this._transition(timestep, 'NOMINAL', 'DEGRADED', 'sensor_warning', drone, mm);
  };
  FSM.prototype._fromDegraded = function (timestep, fs, drone, mm) {
    if (fs.type === 'path_blocked' || fs.type === 'combined') {
      this._transition(timestep, 'DEGRADED', 'REPLANNING', 'path_blocked_or_combined', drone, mm);
      this.requires_replan = true;
      return;
    }
    if (fs.level === 'CRITICAL') {
      this._transition(timestep, 'DEGRADED', 'SAFE_MODE', 'sensor_critical', drone, mm);
      return;
    }
    var noise = drone.sensor_noise_level;
    var pn = this._prevNoise;
    if (pn != null && noise > pn + 1e-12) {
      this._transition(timestep, 'DEGRADED', 'REPLANNING', 'sensor_noise_worsened', drone, mm);
      this.requires_replan = true;
      return;
    }
    if (fs.level === 'NOMINAL' && noise < this._warn) {
      this._transition(timestep, 'DEGRADED', 'NOMINAL', 'sensor_recovered', drone, mm);
    }
  };
  FSM.prototype._fromReplanning = function (timestep, fs, drone, mm) {
    if (fs.level === 'CRITICAL' && fs.type !== 'position_compromised') {
      this._transition(timestep, 'REPLANNING', 'SAFE_MODE', 'critical_while_replanning', drone, mm);
    }
  };
  FSM.prototype.replan_succeeded = function (fs, drone, mm) {
    if (this._state !== 'REPLANNING') return;
    var ts = this._lastTs;
    if (fs.level === 'CRITICAL' && fs.type !== 'position_compromised') {
      this._transition(ts, 'REPLANNING', 'SAFE_MODE', 'replan_done_critical_fault', drone, mm);
    }
    else if (fs.level === 'WARNING') this._transition(ts, 'REPLANNING', 'DEGRADED', 'replan_succeeded_warning', drone, mm);
    else this._transition(ts, 'REPLANNING', 'NOMINAL', 'replan_succeeded_nominal', drone, mm);
    this.requires_replan = false;
  };
  FSM.prototype.replan_failed = function (drone, mm) {
    if (this._state !== 'REPLANNING') return;
    var ts = this._lastTs;
    this._transition(ts, 'REPLANNING', 'SAFE_MODE', 'replan_failed', drone, mm);
    this.requires_replan = false;
  };
  FSM.prototype.check_safe_mode_abort = function (timestep, drone, mm, planner) {
    if (this._state !== 'SAFE_MODE') return;
    if (mm.mission_status !== 'aborted') return;
    if (planner.get_full_path().length) return;
    this._transition(timestep, 'SAFE_MODE', 'ABORT', 'no_path_home', drone, mm);
  };

  function leqKey(k1, k2) {
    return k1[0] < k2[0] || (k1[0] === k2[0] && k1[1] <= k2[1]);
  }
  function ltKey(k1, k2) {
    return k1[0] < k2[0] || (k1[0] === k2[0] && k1[1] < k2[1]);
  }

  /** Heap ordering for D* Lite open set (matches Python tuple order). */
  var _heapCmp = function (a, b) {
    if (a[0] !== b[0]) return a[0] < b[0] ? -1 : 1;
    if (a[1] !== b[1]) return a[1] < b[1] ? -1 : 1;
    return a[2] - b[2];
  };

  function DStarLitePlanner() {
    this._env = null;
    this._w = 0; this._hgt = 0;
    this._g = null; this._rhs = null;
    this._sStart = [0, 0]; this._sGoal = [0, 0]; this._sLast = [0, 0];
    this._km = 0;
    this._heap = [];
    this._heapSeq = 0;
    this._heapBest = {};
    this._lastProcessedTimestep = -1;
    this._path = [];
    this._timingWrap = null;
    this._exemptCell = null;
    this._ignoreHazardCells = false;
  }
  DStarLitePlanner.prototype._ig = function (x, y) { return this._g[y * this._w + x]; };
  DStarLitePlanner.prototype._irhs = function (x, y) { return this._rhs[y * this._w + x]; };
  DStarLitePlanner.prototype._setG = function (x, y, v) { this._g[y * this._w + x] = v; };
  DStarLitePlanner.prototype._setRhs = function (x, y, v) { this._rhs[y * this._w + x] = v; };

  DStarLitePlanner.prototype.calculate_key = function (s) {
    var gx = this._ig(s[0], s[1]);
    var rhsx = this._irhs(s[0], s[1]);
    var m = Math.min(gx, rhsx);
    return [m + hFun(this._sStart, s) + this._km, m];
  };

  DStarLitePlanner.prototype._edgeCost = function (u, v) {
    var ex = this._exemptCell;
    var ign = this._ignoreHazardCells;
    var ux = u[0], uy = u[1], vx = v[0], vy = v[1];
    if (this._env.is_blocked(vx, vy, ex, ign) || this._env.is_blocked(ux, uy, ex, ign)) return INF;
    if (Math.abs(ux - vx) + Math.abs(uy - vy) === 1) return 1;
    if (Math.abs(ux - vx) === 1 && Math.abs(uy - vy) === 1) return SQRT2;
    return INF;
  };

  DStarLitePlanner.prototype._succ = function (u) {
    return this._env.get_neighbors(u[0], u[1], this._ignoreHazardCells);
  };

  DStarLitePlanner.prototype._pred = function (u) {
    var ux = u[0], uy = u[1], out = [];
    for (var dx = -1; dx <= 1; dx++) {
      for (var dy = -1; dy <= 1; dy++) {
        if (dx === 0 && dy === 0) continue;
        var nx = ux + dx, ny = uy + dy;
        if (nx >= 0 && nx < this._w && ny >= 0 && ny < this._hgt) out.push([nx, ny]);
      }
    }
    return out;
  };

  DStarLitePlanner.prototype.update_vertex = function (u) {
    var gx = this._sGoal[0], gy = this._sGoal[1];
    if (u[0] !== gx || u[1] !== gy) {
      var best = INF;
      var succ = this._succ(u);
      for (var i = 0; i < succ.length; i++) {
        var sp = succ[i];
        var c = this._edgeCost(u, sp);
        var gsp = this._ig(sp[0], sp[1]);
        best = Math.min(best, c + gsp);
      }
      this._setRhs(u[0], u[1], best);
    }
    var key = u[0] + ',' + u[1];
    if (this._heapBest[key]) delete this._heapBest[key];
    var gxy = this._ig(u[0], u[1]), rhsxy = this._irhs(u[0], u[1]);
    if (gxy !== rhsxy) {
      var k = this.calculate_key(u);
      this._heapBest[key] = k;
      this._heapSeq++;
      heapPush(this._heap, [k[0], k[1], this._heapSeq, u[0], u[1]], _heapCmp);
    }
  };

  DStarLitePlanner.prototype._topKey = function () {
    var heap = this._heap;
    while (heap.length) {
      var top = heap[0];
      var k0 = top[0], k1 = top[1], u = [top[3], top[4]];
      var key = u[0] + ',' + u[1];
      if (!this._heapBest[key]) {
        heapPop(heap, _heapCmp);
        continue;
      }
      var kb = this._heapBest[key];
      if (k0 !== kb[0] || k1 !== kb[1]) {
        heapPop(heap, _heapCmp);
        continue;
      }
      return [k0, k1];
    }
    return [INF, INF];
  };

  DStarLitePlanner.prototype._pop = function () {
    var heap = this._heap;
    while (heap.length) {
      var top = heapPop(heap, _heapCmp);
      if (!top) return null;
      var u = [top[3], top[4]];
      var key = u[0] + ',' + u[1];
      if (!this._heapBest[key]) continue;
      var kb = this._heapBest[key];
      if (top[0] !== kb[0] || top[1] !== kb[1]) continue;
      delete this._heapBest[key];
      return [[top[0], top[1]], u];
    }
    return null;
  };

  DStarLitePlanner.prototype._consistentStart = function () {
    return this._ig(this._sStart[0], this._sStart[1]) === this._irhs(this._sStart[0], this._sStart[1]);
  };

  DStarLitePlanner.prototype.compute_shortest_path = function () {
    var sx = this._sStart[0], sy = this._sStart[1];
    var ck = this.calculate_key([sx, sy]);
    while (leqKey(this._topKey(), ck) || !this._consistentStart()) {
      var popped = this._pop();
      if (!popped) break;
      var kOld = popped[0], u = popped[1];
      var kNew = this.calculate_key(u);
      if (ltKey(kOld, kNew)) {
        var key = u[0] + ',' + u[1];
        this._heapBest[key] = kNew;
        this._heapSeq++;
        heapPush(this._heap, [kNew[0], kNew[1], this._heapSeq, u[0], u[1]], _heapCmp);
      } else {
        var gu = this._ig(u[0], u[1]), rhsu = this._irhs(u[0], u[1]);
        if (gu > rhsu) {
          this._setG(u[0], u[1], rhsu);
          var pred = this._pred(u);
          for (var i = 0; i < pred.length; i++) this.update_vertex(pred[i]);
        } else {
          this._setG(u[0], u[1], INF);
          var pred2 = this._pred(u);
          var set = {};
          for (var j = 0; j < pred2.length; j++) set[pred2[j][0] + ',' + pred2[j][1]] = pred2[j];
          set[u[0] + ',' + u[1]] = u;
          for (var k in set) this.update_vertex(set[k]);
        }
      }
      ck = this.calculate_key([sx, sy]);
    }
  };

  DStarLitePlanner.prototype._initArrays = function () {
    var n = this._w * this._hgt;
    this._g = new Float64Array(n);
    this._rhs = new Float64Array(n);
    for (var i = 0; i < n; i++) { this._g[i] = INF; this._rhs[i] = INF; }
  };

  DStarLitePlanner.prototype._initialize = function () {
    this._heap = [];
    this._heapBest = {};
    this._initArrays();
    var gx = this._sGoal[0], gy = this._sGoal[1];
    this._setRhs(gx, gy, 0);
    this.update_vertex([gx, gy]);
  };

  DStarLitePlanner.prototype.plan = function (start, goal, env, exemptStart) {
    exemptStart = !!exemptStart;
    this._exemptCell = null;
    this._ignoreHazardCells = false;
    var sx = start[0] | 0, sy = start[1] | 0, gx = goal[0] | 0, gy = goal[1] | 0;
    if (!exemptStart) {
      if (env.is_blocked(sx, sy) || env.is_blocked(gx, gy)) throw new Error('blocked');
    } else {
      if (env.is_blocked(gx, gy)) throw new Error('blocked');
      this._exemptCell = [sx, sy];
      this._ignoreHazardCells = true;
    }
    var t0 = performance.now();
    this._env = env;
    this._w = env._w; this._hgt = env._h;
    this._sStart = [sx, sy];
    this._sGoal = [gx, gy];
    this._sLast = this._sStart.slice();
    this._km = 0;
    this._initialize();
    this.compute_shortest_path();
    this._lastProcessedTimestep = Math.max(env.current_timestep, 0);
    this._path = this._extractPath();
    var dt = performance.now() - t0;
    if (this._timingWrap) this._timingWrap.push({ kind: 'plan', ms: dt, timestep: env.current_timestep });
    return this._path.slice();
  };

  DStarLitePlanner.prototype.replan = function (cur, goal, env, exemptStart) {
    exemptStart = !!exemptStart;
    if ((goal[0] | 0) !== this._sGoal[0] || (goal[1] | 0) !== this._sGoal[1]) {
      this.plan(cur, goal, env, exemptStart);
      return;
    }
    var t0 = performance.now();
    this._exemptCell = null;
    this._ignoreHazardCells = false;
    var gx = goal[0] | 0, gy = goal[1] | 0;
    var c = [cur[0] | 0, cur[1] | 0];
    if (!exemptStart) {
      if (env.is_blocked(c[0], c[1]) || env.is_blocked(gx, gy)) throw new Error('blocked');
    } else {
      if (env.is_blocked(gx, gy)) throw new Error('blocked');
      this._exemptCell = c.slice();
      this._ignoreHazardCells = true;
    }
    this._env = env;
    if (c[0] !== this._sLast[0] || c[1] !== this._sLast[1]) {
      this._km += hFun(this._sLast, c);
      this._sLast = c.slice();
    }
    this._sStart = c.slice();
    var changed = env.get_changes_since(this._lastProcessedTimestep);
    this._lastProcessedTimestep = env.current_timestep;
    var affected = {};
    for (var i = 0; i < changed.length; i++) {
      var cx = changed[i][0], cy = changed[i][1];
      affected[cx + ',' + cy] = [cx, cy];
      var pred = this._pred([cx, cy]);
      for (var j = 0; j < pred.length; j++) {
        var p = pred[j];
        affected[p[0] + ',' + p[1]] = p;
      }
    }
    for (var k in affected) this.update_vertex(affected[k]);
    this.compute_shortest_path();
    this._path = this._extractPath();
    var dt = performance.now() - t0;
    if (this._timingWrap) this._timingWrap.push({ kind: 'replan', ms: dt, timestep: env.current_timestep });
  };

  DStarLitePlanner.prototype._extractPath = function () {
    if (!this._g || !this._env) return [];
    var sx = this._sStart[0], sy = this._sStart[1];
    var gx = this._sGoal[0], gy = this._sGoal[1];
    if (!isFinite(this._ig(sx, sy))) return [];
    var path = [[sx, sy]];
    var cur = [sx, sy];
    var guard = this._w * this._hgt + 5;
    while ((cur[0] !== gx || cur[1] !== gy) && guard-- > 0) {
      var best = null, bestC = INF;
      var succ = this._succ(cur);
      for (var i = 0; i < succ.length; i++) {
        var sp = succ[i];
        var c = this._edgeCost(cur, sp) + this._ig(sp[0], sp[1]);
        if (c < bestC || (isClose(c, bestC) && (!best || sp[0] < best[0] || (sp[0] === best[0] && sp[1] < best[1])))) {
          bestC = c; best = sp;
        }
      }
      if (!best || !isFinite(bestC)) break;
      path.push(best);
      cur = best;
    }
    return path;
  };

  DStarLitePlanner.prototype.get_next_step = function (pos) {
    var cx = pos[0] | 0, cy = pos[1] | 0;
    if (!this._g || !this._env) throw new Error('plan first');
    if (!isFinite(this._ig(cx, cy))) throw new Error('no path');
    if (cx === this._sGoal[0] && cy === this._sGoal[1]) return [cx, cy];
    var best = null, bestC = INF;
    var succ = this._succ([cx, cy]);
    for (var i = 0; i < succ.length; i++) {
      var sp = succ[i];
      var c = this._edgeCost([cx, cy], sp) + this._ig(sp[0], sp[1]);
      if (c < bestC || (isClose(c, bestC) && (!best || sp[0] < best[0] || (sp[0] === best[0] && sp[1] < best[1])))) {
        bestC = c; best = sp;
      }
    }
    if (!best || !isFinite(bestC)) throw new Error('no step');
    return best;
  };

  DStarLitePlanner.prototype.get_full_path = function () { return this._path.slice(); };
  DStarLitePlanner.prototype.get_name = function () { return 'd_star_lite'; };

  function AStarPlanner() {
    this._env = null;
    this._goal = [0, 0];
    this._path = [];
    this._timingWrap = null;
    this._exemptCell = null;
    this._ignoreHazardCells = false;
  }
  AStarPlanner.prototype._edgeCost = DStarLitePlanner.prototype._edgeCost;
  AStarPlanner.prototype.plan = function (start, goal, env, exemptStart) {
    exemptStart = !!exemptStart;
    this._exemptCell = null;
    this._ignoreHazardCells = false;
    var t0 = performance.now();
    var sx = start[0] | 0, sy = start[1] | 0, gx = goal[0] | 0, gy = goal[1] | 0;
    if (!exemptStart) {
      if (env.is_blocked(sx, sy) || env.is_blocked(gx, gy)) throw new Error('blocked');
    } else {
      if (env.is_blocked(gx, gy)) throw new Error('blocked');
      this._exemptCell = [sx, sy];
      this._ignoreHazardCells = true;
    }
    this._env = env;
    this._goal = [gx, gy];
    if (sx === gx && sy === gy) {
      this._path = [[sx, sy]];
      if (this._timingWrap) this._timingWrap.push({ kind: 'plan', ms: performance.now() - t0, timestep: env.current_timestep });
      return this._path.slice();
    }
    var gScore = {};
    var came = {};
    var tie = 0;
    var heap = [];
    var startKey = sx + ',' + sy;
    gScore[startKey] = 0;
    heapPush(heap, [0 + hFun([sx, sy], [gx, gy]), tie++, 0, sx, sy], function (a, b) {
      if (a[0] !== b[0]) return a[0] < b[0] ? -1 : 1;
      return a[2] - b[2];
    });
    while (heap.length) {
      var item = heapPop(heap, function (a, b) {
        if (a[0] !== b[0]) return a[0] < b[0] ? -1 : 1;
        return a[2] - b[2];
      });
      var gU = item[2], ux = item[3], uy = item[4];
      var ukey = ux + ',' + uy;
      if (gU > (gScore[ukey] || INF) + 1e-9) continue;
      if (ux === gx && uy === gy) {
        this._path = this._reconstruct(came, [gx, gy], [sx, sy]);
        if (this._timingWrap) this._timingWrap.push({ kind: 'plan', ms: performance.now() - t0, timestep: env.current_timestep });
        return this._path.slice();
      }
      var neigh = env.get_neighbors(ux, uy, this._ignoreHazardCells);
      for (var i = 0; i < neigh.length; i++) {
        var v = neigh[i];
        var c = this._edgeCost([ux, uy], v);
        if (!isFinite(c)) continue;
        var tentative = gU + c;
        var vk = v[0] + ',' + v[1];
        var gv = gScore[vk];
        if (gv === undefined || tentative < gv - 1e-12) {
          gScore[vk] = tentative;
          came[vk] = [ux, uy];
          heapPush(heap, [tentative + hFun(v, [gx, gy]), tie++, tentative, v[0], v[1]], function (a, b) {
            if (a[0] !== b[0]) return a[0] < b[0] ? -1 : 1;
            return a[2] - b[2];
          });
        }
      }
    }
    this._path = [];
    if (this._timingWrap) this._timingWrap.push({ kind: 'plan', ms: performance.now() - t0, timestep: env.current_timestep });
    return [];
  };

  AStarPlanner.prototype._reconstruct = function (came, goalCell, startCell) {
    var pathRev = [];
    var cur = goalCell;
    var guard = 50000;
    while (cur && guard-- > 0) {
      pathRev.push(cur.slice());
      if (cur[0] === startCell[0] && cur[1] === startCell[1]) break;
      var k = cur[0] + ',' + cur[1];
      var p = came[k];
      if (!p) return [];
      cur = p;
    }
    if (!pathRev.length || pathRev[pathRev.length - 1][0] !== startCell[0] || pathRev[pathRev.length - 1][1] !== startCell[1]) return [];
    pathRev.reverse();
    return pathRev;
  };

  AStarPlanner.prototype.replan = function (cur, goal, env, exemptStart) {
    var t0 = performance.now();
    this.plan(cur, goal, env, !!exemptStart);
    if (this._timingWrap) this._timingWrap.push({ kind: 'replan', ms: performance.now() - t0, timestep: env.current_timestep });
  };

  AStarPlanner.prototype.get_next_step = function (pos) {
    if (!this._path.length || !this._env) throw new Error('plan first');
    var cx = pos[0] | 0, cy = pos[1] | 0;
    var ix = -1;
    for (var i = 0; i < this._path.length; i++) {
      if (this._path[i][0] === cx && this._path[i][1] === cy) { ix = i; break; }
    }
    if (ix < 0) throw new Error('off path');
    if (cx === this._goal[0] && cy === this._goal[1]) return [cx, cy];
    if (ix + 1 < this._path.length) return this._path[ix + 1].slice();
    throw new Error('no step');
  };

  AStarPlanner.prototype.get_full_path = function () { return this._path.slice(); };
  AStarPlanner.prototype.get_name = function () { return 'a_star'; };

  function makePlanner(kind) {
    if (kind === 'd_star_lite') return new DStarLitePlanner();
    return new AStarPlanner();
  }

  function plannerDisplayName(p) {
    var n = p.get_name();
    if (n === 'd_star_lite') return 'D* Lite';
    if (n === 'a_star') return 'A*';
    return n;
  }

  function buildEnvironmentExport(config) {
    var envCfg = config.environment;
    var gw = envCfg.grid_size[0] | 0, gh = envCfg.grid_size[1] | 0;
    var staticObs = [];
    (envCfg.static_obstacles || []).forEach(function (ob) {
      for (var x = ob.x; x < ob.x + ob.width; x++) {
        for (var y = ob.y; y < ob.y + ob.height; y++) {
          if (x >= 0 && x < gw && y >= 0 && y < gh) staticObs.push([x, y]);
        }
      }
    });
    function hazardTrig(id) {
      var sch = (config.fault_injection && config.fault_injection.schedule) || [];
      for (var i = 0; i < sch.length; i++) {
        if (sch[i].type === 'environmental_hazard' && String(sch[i].hazard_id) === String(id))
          return sch[i].timestep | 0;
      }
      return null;
    }
    function hzCells(hz) {
      var px = hz.position.x | 0, py = hz.position.y | 0;
      if (hz.type === 'no_fly_zone') {
        var r = +hz.radius, ri = Math.ceil(r), out = [];
        for (var x = px - ri; x <= px + ri; x++) {
          for (var y = py - ri; y <= py + ri; y++) {
            if (x >= 0 && x < gw && y >= 0 && y < gh && hypot(x - px, y - py) <= r + 1e-9) out.push([x, y]);
          }
        }
        return out;
      }
      var out2 = [];
      for (var xx = px; xx < px + (hz.width | 0); xx++) {
        for (var yy = py; yy < py + (hz.height | 0); yy++) {
          if (xx >= 0 && xx < gw && yy >= 0 && yy < gh) out2.push([xx, yy]);
        }
      }
      return out2;
    }
    var dyn = (envCfg.dynamic_hazards || []).map(function (hz) {
      return {
        id: String(hz.id),
        type: hz.type,
        trigger_timestep: hazardTrig(hz.id),
        cells: hzCells(hz)
      };
    });
    return { grid_size: [gw, gh], static_obstacles: staticObs, dynamic_hazards: dyn };
  }

  function mergeExportEvents(fsmLog, injEvents) {
    var events = [];
    fsmLog.forEach(function (e) {
      events.push({
        timestep: e.timestep | 0,
        type: 'fsm_transition',
        from: e.from_state,
        to: e.to_state,
        trigger: e.trigger
      });
    });
    injEvents.forEach(function (e) {
      if (e.type === 'hazard_spawned') {
        events.push({ timestep: e.timestep | 0, type: 'hazard_spawned', id: e.id, description: e.description || '' });
      } else if (e.type === 'fault_injected') {
        events.push({ timestep: e.timestep | 0, type: 'fault_injected', fault_type: e.fault_type, severity: e.severity });
      } else if (e.type === 'fault_expired') {
        events.push({ timestep: e.timestep | 0, type: 'fault_expired', fault_type: e.fault_type });
      }
    });
    var order = { hazard_spawned: 0, fault_injected: 1, fault_expired: 2, fsm_transition: 3 };
    events.sort(function (a, b) {
      if (a.timestep !== b.timestep) return a.timestep - b.timestep;
      return (order[a.type] || 9) - (order[b.type] || 9);
    });
    return events;
  }

  function missionResultString(mstatus, fsmState, battery) {
    if (mstatus === 'completed') return 'completed';
    if (battery <= 0 && mstatus !== 'completed') return 'abort_emergency';
    if (fsmState === 'ABORT') return 'aborted';
    return 'aborted';
  }

  function faultsPayload(active, stepInj) {
    var injected = null;
    for (var i = 0; i < stepInj.length; i++) {
      var ev = stepInj[i];
      if (ev.type === 'fault_injected') {
        injected = { fault_type: ev.fault_type, severity: ev.severity };
        break;
      }
      if (ev.type === 'hazard_spawned') {
        injected = { fault_type: 'environmental_hazard', hazard_id: ev.id, description: ev.description || '' };
        break;
      }
    }
    return { active: active, injected_this_step: injected };
  }

  function fsmTransitionForTimeline(newEntries) {
    if (!newEntries.length) return null;
    var e = newEntries[newEntries.length - 1];
    return { from: e.from_state, to: e.to_state, trigger: e.trigger };
  }

  function skipBlockedWaypoints(env, mm, fsm, waypointCount, timestep) {
    while (mm.mission_status === 'in_progress' && mm.current_waypoint_index < waypointCount) {
      var t = mm.get_current_target();
      if (!env.is_blocked(t[0], t[1])) break;
      fsm.record_waypoint_skipped(timestep, 'target_cell_blocked');
      mm.skip_waypoint();
    }
  }

  function planningPosition(drone, env, fsm) {
    var st = fsm.get_state();
    if (st === 'DEGRADED' || st === 'REPLANNING' || st === 'SAFE_MODE') {
      return drone.get_position_estimate(env);
    }
    return [drone.position[0] | 0, drone.position[1] | 0];
  }

  /**
   * plan() throws if start or goal cell is blocked — e.g. drone still on a cell
   * that just became a hazard while skipBlockedWaypoints advanced the goal.
   * Retry with a fresh planner once; on second failure return a new planner with no path.
   */
  function safePlan(planner, start, goal, env, plannerKind) {
    var ex = env.is_blocked(start[0], start[1]);
    try {
      planner.plan(start, goal, env, ex);
      return planner;
    } catch (e1) {
      var p2 = makePlanner(plannerKind);
      p2._timingWrap = planner._timingWrap;
      try {
        p2.plan(start, goal, env, ex);
        return p2;
      } catch (e2) {
        var p3 = makePlanner(plannerKind);
        p3._timingWrap = planner._timingWrap;
        return p3;
      }
    }
  }

  /**
   * Run full simulation; returns export-shaped object.
   */
  function runSimulation(config, plannerKind, maxSteps) {
    maxSteps = maxSteps || 2000;
    var timings = [];
    var rng = mulberry32(hashConfig(config));
    var env = new Environment(deepCopy(config));
    var drone = new Drone(deepCopy(config));
    drone._rng = rng;
    if (env.is_blocked(drone.position[0], drone.position[1])) throw new Error('start blocked');
    var mm = new MissionManager(deepCopy(config));
    var faultInj = new FaultInjector(config.fault_injection || {});
    var fd = new FaultDetector(config.algorithm);
    var fsm = new FSM(config.mission, config.algorithm);
    var planner = makePlanner(plannerKind);
    planner._timingWrap = timings;
    var waypointCount = config.mission.waypoints.length;
    skipBlockedWaypoints(env, mm, fsm, waypointCount, 0);
    var lastGoal = mm.get_current_target();
    planner = safePlan(planner, drone.position, lastGoal, env, plannerKind);

    var replanEvents = [];
    var timeline = [];
    var injectorAccum = [];
    var fsmLogCursor = 0;
    var batCap = +config.drone.battery_capacity;

    for (var timestep = 0; timestep < maxSteps; timestep++) {
      var logBefore = fsm.get_transition_log().length;
      faultInj.update(timestep, drone, env);
      var stepInj = faultInj.get_last_step_events();
      injectorAccum = injectorAccum.concat(stepInj);
      env.update(timestep);
      skipBlockedWaypoints(env, mm, fsm, waypointCount, timestep);
      var senseData = drone.sense(env);

      var gt = mm.get_current_target();
      if (gt[0] !== lastGoal[0] || gt[1] !== lastGoal[1]) {
        lastGoal = gt.slice();
        planner = safePlan(planner, planningPosition(drone, env, fsm), lastGoal, env, plannerKind);
      }

      var faultStatus = fd.evaluate(drone, env, planner);
      fsm.update(timestep, faultStatus, drone, mm);
      var planningPos = planningPosition(drone, env, fsm);

      var replannedThisStep = false;
      var pathBeforeReplan = null;
      if (fsm.requires_replan) {
        var pathBefore = planner.get_full_path().map(function (p) { return [p[0], p[1]]; });
        pathBeforeReplan = pathBefore;
        replannedThisStep = true;
        var exemptStart = fsm.emergency_escape;
        try {
          planner.replan(planningPos, mm.get_current_target(), env, exemptStart);
          var pathOk = planner.get_full_path().length > 0;
          var pathAfter = planner.get_full_path().map(function (p) { return [p[0], p[1]]; });
          replanEvents.push({ timestep: timestep, path_before: pathBefore, path_after: pathAfter });
          var fsAfter = fd.evaluate(drone, env, planner);
          if (!pathOk) fsm.replan_failed(drone, mm);
          else fsm.replan_succeeded(fsAfter, drone, mm);
        } catch (err) {
          replanEvents.push({ timestep: timestep, path_before: pathBefore, path_after: [], error: true });
          fsm.replan_failed(drone, mm);
        } finally {
          fsm.emergency_escape = false;
        }
      } else if (fsm.get_state() === 'DEGRADED' || fsm.get_state() === 'REPLANNING' || fsm.get_state() === 'SAFE_MODE') {
        try {
          var ppx = planningPos[0] | 0, ppy = planningPos[1] | 0;
          planner.replan(planningPos, mm.get_current_target(), env, env.is_blocked(ppx, ppy));
        } catch (e) {}
      }

      if (fsm.get_state() === 'SAFE_MODE' && mm.mission_status === 'aborted') {
        fsm.check_safe_mode_abort(timestep, drone, mm, planner);
      }

      var nextMove;
      try {
        nextMove = planner.get_next_step(planningPos);
      } catch (e) {
        var px = drone.position[0] | 0, py = drone.position[1] | 0;
        if (env.is_blocked(px, py)) {
          var neigh = env.get_neighbors(px, py);
          if (!neigh.length) neigh = env.get_neighbors(px, py, true);
          nextMove = neigh.length ? neigh[0].slice() : drone.position.slice();
        } else {
          if (fsm.get_state() === 'REPLANNING') fsm.replan_failed(drone, mm);
          nextMove = drone.position.slice();
        }
      }
      drone.move(nextMove[0], nextMove[1]);
      mm.update(drone.position);
      fsm.check_depleted_battery_abort(timestep, drone, mm);

      var logFull = fsm.get_transition_log();
      var newFsm = logFull.slice(fsmLogCursor);
      fsmLogCursor = logFull.length;
      var wp = mm.get_progress();
      var stNow = fsm.get_state();
      var pe;
      if (stNow === 'DEGRADED' || stNow === 'REPLANNING' || stNow === 'SAFE_MODE') {
        pe = [planningPos[0], planningPos[1]];
      } else {
        pe = senseData.position_estimate;
      }
      var planned = planner.get_full_path().map(function (p) { return [p[0] | 0, p[1] | 0]; });

      timeline.push({
        timestep: timestep,
        drone: {
          position: [drone.position[0], drone.position[1]],
          position_estimate: [pe[0], pe[1]],
          planning_position: [planningPos[0] | 0, planningPos[1] | 0],
          heading: drone.heading,
          battery: drone.battery,
          battery_pct: batCap > 0 ? Math.round(10000 * drone.battery / batCap) / 100 : 0,
          sensor_noise_level: drone.sensor_noise_level,
          speed_scale: drone.get_telemetry().speed_scale
        },
        fsm: { state: fsm.get_state(), transition: fsmTransitionForTimeline(newFsm) },
        mission: {
          current_waypoint_index: wp.current_waypoint_index,
          current_target: [wp.current_target[0], wp.current_target[1]],
          waypoints_completed: wp.waypoints_completed,
          total_waypoints: wp.waypoints_total,
          status: wp.mission_status
        },
        planner: {
          name: plannerDisplayName(planner),
          planned_path: planned,
          replanned_this_step: replannedThisStep,
          path_before_replan: pathBeforeReplan ? pathBeforeReplan.map(function (p) { return [p[0], p[1]]; }) : null
        },
        faults: faultsPayload(faultInj.get_active_faults(), stepInj)
      });

      if (config.mission && mm.mission_status === 'completed') break;
      if (fsm.get_state() === 'ABORT') break;
      if (drone.battery <= 0) break;
    }

    var transitionLog = fsm.get_transition_log();
    var merged = mergeExportEvents(transitionLog, injectorAccum);
    var mresult = missionResultString(mm.mission_status, fsm.get_state(), drone.battery);

    var planTimes = timings.filter(function (t) { return t.kind === 'plan'; }).map(function (t) { return t.ms; });
    var replanTimes = timings.filter(function (t) { return t.kind === 'replan'; }).map(function (t) { return t.ms; });
    var totalPathCells = 0;
    for (var ti = 0; ti < timeline.length; ti++) {
      totalPathCells += (timeline[ti].planner.planned_path || []).length;
    }
    var trajLen = 0;
    for (var tj = 1; tj < timeline.length; tj++) {
      var a = timeline[tj - 1].drone.position, b = timeline[tj].drone.position;
      trajLen += hypot(a[0] - b[0], a[1] - b[1]);
    }

    return {
      metadata: {
        config: deepCopy(config),
        total_timesteps: timeline.length,
        mission_result: mresult,
        planner_used: plannerDisplayName(planner),
        total_replans: replanEvents.length,
        export_timestamp: new Date().toISOString().replace(/\.\d{3}Z$/, 'Z'),
        replan_compute_timings_ms: timings,
        stats: {
          trajectory_length: trajLen,
          sum_planned_path_lengths: totalPathCells,
          battery_used: batCap - drone.battery,
          fsm_transition_count: transitionLog.length,
          plan_call_ms_samples: planTimes,
          replan_call_ms_samples: replanTimes
        }
      },
      environment: buildEnvironmentExport(config),
      timeline: timeline,
      events: merged,
      fsm_transition_log: transitionLog,
      _replan_events: replanEvents
    };
  }

  global.RNSim = {
    runSimulation: runSimulation,
    buildEnvironmentExport: buildEnvironmentExport,
    mergeExportEvents: mergeExportEvents,
    plannerDisplayName: plannerDisplayName,
    missionResultString: missionResultString,
    hashConfig: hashConfig
  };
})(typeof window !== 'undefined' ? window : this);
