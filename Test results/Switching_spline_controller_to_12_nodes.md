How to change the Spline controller from 5 nodes to 12 nodes

Background
----------
The node count is **compile-time**, not CSV-driven: it's baked into fixed
parameter indices (`ExoCode/src/ControllerData.h`), a fixed `num_parameter`,
and fixed-size stack arrays in `ExoCode/src/Controller.cpp` (`const int n = 5;`).
Going from 5 to 12 nodes touches every place that number appears. This is the
same kind of change documented for a 5→7 upgrade in
`Useful guides by us/ADDING_SPLINE_NODES.md` — this file is the fully worked
5→12 version, with two extra limits that upgrade crosses and this one does
too (see "Limits" below), plus exact copy-paste code for every file.

The `Spline` class and the `controller_defs::spline` parameter layout are
**shared by hip, ankle, arm_1, and arm_2** — this change affects all four
joints and all four `spline.csv` files, even if you only use one joint today.

Parameter math for 12 nodes: `2*N + 6` where `N` = node count →
`2*12 + 6 = 30` parameters.

Limits this upgrade crosses (read before editing)
--------------------------------------------------
- **`max_parameters`** (`ExoCode/src/ControllerData.h:244`) is currently `22`
  (`= spv2::num_parameter`, the largest controller today). 30 > 22, so
  **`max_parameters` must be raised** — this is a real RAM cost, since
  `ControllerData::parameters[controller_defs::max_parameters]` (line 284)
  is allocated per joint/side. The 5→7 upgrade in the other guide didn't
  need this (20 ≤ 22); 12 nodes does.
- **`MAX_COLUMNS`** (`ExoCode/src/ListCtrlParams.h:38`) is currently `30`,
  and the GUI-listing code reserves `PREFIX_COLS = 4` of those for a
  joint-name/joint-id/controller-name/controller-id prefix (line 74),
  leaving 26 usable data columns. 30 data columns (24 node values + 6
  flags) don't fit in 26 — **`MAX_COLUMNS` must be raised too**, or the
  spline row silently gets truncated when streamed to the GUI. The 5→7
  upgrade didn't need this either (20 ≤ 26); 12 nodes does.

Everything else below is the same shape as the other guide's 5→7 example,
just carried out to 12 nodes and including these two extra edits.

---

Edit 1 — Parameter indices (`ExoCode/src/ControllerData.h`)
-------------------------------------------------------------
Find `namespace spline` (currently lines 67-86):

```cpp
    namespace spline
    {
        const uint8_t node1_x_idx = 0;                          //Percent gait for node 1
        const uint8_t node1_y_idx = 1;                          //Torque for node 1 in Nm
        const uint8_t node2_x_idx = 2;                          //Percent gait for node 2
        const uint8_t node2_y_idx = 3;                          //Torque for node 2 in Nm
        const uint8_t node3_x_idx = 4;                          //Percent gait for node 3
        const uint8_t node3_y_idx = 5;                          //Torque for node 3 in Nm
        const uint8_t node4_x_idx = 6;                          //Percent gait for node 4
        const uint8_t node4_y_idx = 7;                          //Torque for node 4 in Nm
        const uint8_t node5_x_idx = 8;                          //Percent gait for node 5
        const uint8_t node5_y_idx = 9;                          //Torque for node 5 in Nm
        const uint8_t sim_gait_idx = 10;                        //Flag to simulate percent gait
        const uint8_t use_percent_gait_idx = 11;                //0 = use percent stance (legacy), 1 = use percent gait
        const uint8_t use_pid_idx = 12;                         //Flag to use PID control
        const uint8_t p_gain_idx = 13;                          //Value of P Gain for PID control
        const uint8_t i_gain_idx = 14;                          //Value of I Gain for PID control
        const uint8_t d_gain_idx = 15;                          //Value of D Gain for PID control
        const uint8_t num_parameter = 16;
    }
```

Replace the whole block with:

```cpp
    namespace spline
    {
        const uint8_t node1_x_idx = 0;                          //Percent gait for node 1
        const uint8_t node1_y_idx = 1;                          //Torque for node 1 in Nm
        const uint8_t node2_x_idx = 2;                          //Percent gait for node 2
        const uint8_t node2_y_idx = 3;                          //Torque for node 2 in Nm
        const uint8_t node3_x_idx = 4;                          //Percent gait for node 3
        const uint8_t node3_y_idx = 5;                          //Torque for node 3 in Nm
        const uint8_t node4_x_idx = 6;                          //Percent gait for node 4
        const uint8_t node4_y_idx = 7;                          //Torque for node 4 in Nm
        const uint8_t node5_x_idx = 8;                          //Percent gait for node 5
        const uint8_t node5_y_idx = 9;                          //Torque for node 5 in Nm
        const uint8_t node6_x_idx = 10;                         //Percent gait for node 6
        const uint8_t node6_y_idx = 11;                         //Torque for node 6 in Nm
        const uint8_t node7_x_idx = 12;                         //Percent gait for node 7
        const uint8_t node7_y_idx = 13;                         //Torque for node 7 in Nm
        const uint8_t node8_x_idx = 14;                         //Percent gait for node 8
        const uint8_t node8_y_idx = 15;                         //Torque for node 8 in Nm
        const uint8_t node9_x_idx = 16;                         //Percent gait for node 9
        const uint8_t node9_y_idx = 17;                         //Torque for node 9 in Nm
        const uint8_t node10_x_idx = 18;                        //Percent gait for node 10
        const uint8_t node10_y_idx = 19;                        //Torque for node 10 in Nm
        const uint8_t node11_x_idx = 20;                        //Percent gait for node 11
        const uint8_t node11_y_idx = 21;                        //Torque for node 11 in Nm
        const uint8_t node12_x_idx = 22;                        //Percent gait for node 12
        const uint8_t node12_y_idx = 23;                        //Torque for node 12 in Nm
        const uint8_t sim_gait_idx = 24;                        //Flag to simulate percent gait
        const uint8_t use_percent_gait_idx = 25;                //0 = use percent stance (legacy), 1 = use percent gait
        const uint8_t use_pid_idx = 26;                         //Flag to use PID control
        const uint8_t p_gain_idx = 27;                          //Value of P Gain for PID control
        const uint8_t i_gain_idx = 28;                          //Value of I Gain for PID control
        const uint8_t d_gain_idx = 29;                          //Value of D Gain for PID control
        const uint8_t num_parameter = 30;
    }
```

The flag/gain reads in `calc_motor_cmd` use these named constants, so once
you renumber them here those reads follow automatically — you don't need to
touch anything else in `Controller.cpp` for the flags/gains, only the node
arrays and `n` (Edits 3 and 4).

Edit 2 — `max_parameters` (`ExoCode/src/ControllerData.h:244`)
-----------------------------------------------------------------
Find:

```cpp
    const uint8_t max_parameters = spv2::num_parameter;         //This should be the largest of all the num_parameters
```

Replace with:

```cpp
    const uint8_t max_parameters = spline::num_parameter;       //This should be the largest of all the num_parameters
```

(`spline::num_parameter` is now `30`, larger than `spv2::num_parameter` = 22
and `pjmc_plus::num_parameter` = 17, so it becomes the new largest. This
line's own comment says it should track whichever controller has the most
parameters — after this edit that's `spline`.)

This array is shared across every joint/side's `ControllerData::parameters`
member (`ControllerData.h:284`), so this raises RAM use by `(30-22) = 8
floats × 4 bytes = 32 bytes` per joint/side instance — trivial on a Teensy,
but worth knowing it's not free.

Edit 3 — Parameter bounds table (`ExoCode/src/ControllerData.cpp`)
-----------------------------------------------------------------------
Find `spline_bounds` (currently around line 104-122):

```cpp
    const ParameterBoundConfig spline_bounds[controller_defs::spline::num_parameter] =
    {
        param_bound(false, 0.0f, 100.0f, false),    // 0 node1_x
        param_bound(false, -100.0f, 100.0f, false), // 1 node1_y
        param_bound(false, 0.0f, 100.0f, false),    // 2 node2_x
        param_bound(false, -100.0f, 100.0f, false), // 3 node2_y
        param_bound(false, 0.0f, 100.0f, false),    // 4 node3_x
        param_bound(false, -100.0f, 100.0f, false), // 5 node3_y
        param_bound(false, 0.0f, 100.0f, false),    // 6 node4_x
        param_bound(false, -100.0f, 100.0f, false), // 7 node4_y
        param_bound(false, 0.0f, 100.0f, false),    // 8 node5_x
        param_bound(false, -100.0f, 100.0f, false), // 9 node5_y
        param_bound(false, 0.0f, 1.0f, true),       // 10 sim_gait
        param_bound(false, 0.0f, 1.0f, true),       // 11 use_percent_gait
        param_bound(false, 0.0f, 1.0f, true),       // 12 use_pid
        param_bound(false, 0.0f, 10000.0f, false),  // 13 p_gain
        param_bound(false, 0.0f, 10000.0f, false),  // 14 i_gain
        param_bound(false, 0.0f, 10000.0f, false),  // 15 d_gain
    };
```

Replace with:

```cpp
    const ParameterBoundConfig spline_bounds[controller_defs::spline::num_parameter] =
    {
        param_bound(false, 0.0f, 100.0f, false),    // 0 node1_x
        param_bound(false, -100.0f, 100.0f, false), // 1 node1_y
        param_bound(false, 0.0f, 100.0f, false),    // 2 node2_x
        param_bound(false, -100.0f, 100.0f, false), // 3 node2_y
        param_bound(false, 0.0f, 100.0f, false),    // 4 node3_x
        param_bound(false, -100.0f, 100.0f, false), // 5 node3_y
        param_bound(false, 0.0f, 100.0f, false),    // 6 node4_x
        param_bound(false, -100.0f, 100.0f, false), // 7 node4_y
        param_bound(false, 0.0f, 100.0f, false),    // 8 node5_x
        param_bound(false, -100.0f, 100.0f, false), // 9 node5_y
        param_bound(false, 0.0f, 100.0f, false),    // 10 node6_x
        param_bound(false, -100.0f, 100.0f, false), // 11 node6_y
        param_bound(false, 0.0f, 100.0f, false),    // 12 node7_x
        param_bound(false, -100.0f, 100.0f, false), // 13 node7_y
        param_bound(false, 0.0f, 100.0f, false),    // 14 node8_x
        param_bound(false, -100.0f, 100.0f, false), // 15 node8_y
        param_bound(false, 0.0f, 100.0f, false),    // 16 node9_x
        param_bound(false, -100.0f, 100.0f, false), // 17 node9_y
        param_bound(false, 0.0f, 100.0f, false),    // 18 node10_x
        param_bound(false, -100.0f, 100.0f, false), // 19 node10_y
        param_bound(false, 0.0f, 100.0f, false),    // 20 node11_x
        param_bound(false, -100.0f, 100.0f, false), // 21 node11_y
        param_bound(false, 0.0f, 100.0f, false),    // 22 node12_x
        param_bound(false, -100.0f, 100.0f, false), // 23 node12_y
        param_bound(false, 0.0f, 1.0f, true),       // 24 sim_gait
        param_bound(false, 0.0f, 1.0f, true),       // 25 use_percent_gait
        param_bound(false, 0.0f, 1.0f, true),       // 26 use_pid
        param_bound(false, 0.0f, 10000.0f, false),  // 27 p_gain
        param_bound(false, 0.0f, 10000.0f, false),  // 28 i_gain
        param_bound(false, 0.0f, 10000.0f, false),  // 29 d_gain
    };
```

This table is what the GUI uses to clamp/validate live parameter edits per
index — if you skip this edit, the array size mismatch (`[30]` expected vs.
`[16]` initializers) **will fail to compile**, so the compiler will catch it
if you forget.

Edit 4 — Build the node arrays (`ExoCode/src/Controller.cpp`, `Spline::calc_motor_cmd`)
--------------------------------------------------------------------------------------------
Find (currently lines 846-862):

```cpp
    float x[5] =
    {
        _controller_data->parameters[controller_defs::spline::node1_x_idx],
        _controller_data->parameters[controller_defs::spline::node2_x_idx],
        _controller_data->parameters[controller_defs::spline::node3_x_idx],
        _controller_data->parameters[controller_defs::spline::node4_x_idx],
        _controller_data->parameters[controller_defs::spline::node5_x_idx],
    };

    float y[5] =
    {
        _controller_data->parameters[controller_defs::spline::node1_y_idx],
        _controller_data->parameters[controller_defs::spline::node2_y_idx],
        _controller_data->parameters[controller_defs::spline::node3_y_idx],
        _controller_data->parameters[controller_defs::spline::node4_y_idx],
        _controller_data->parameters[controller_defs::spline::node5_y_idx],
    };
```

Replace with:

```cpp
    float x[12] =
    {
        _controller_data->parameters[controller_defs::spline::node1_x_idx],
        _controller_data->parameters[controller_defs::spline::node2_x_idx],
        _controller_data->parameters[controller_defs::spline::node3_x_idx],
        _controller_data->parameters[controller_defs::spline::node4_x_idx],
        _controller_data->parameters[controller_defs::spline::node5_x_idx],
        _controller_data->parameters[controller_defs::spline::node6_x_idx],
        _controller_data->parameters[controller_defs::spline::node7_x_idx],
        _controller_data->parameters[controller_defs::spline::node8_x_idx],
        _controller_data->parameters[controller_defs::spline::node9_x_idx],
        _controller_data->parameters[controller_defs::spline::node10_x_idx],
        _controller_data->parameters[controller_defs::spline::node11_x_idx],
        _controller_data->parameters[controller_defs::spline::node12_x_idx],
    };

    float y[12] =
    {
        _controller_data->parameters[controller_defs::spline::node1_y_idx],
        _controller_data->parameters[controller_defs::spline::node2_y_idx],
        _controller_data->parameters[controller_defs::spline::node3_y_idx],
        _controller_data->parameters[controller_defs::spline::node4_y_idx],
        _controller_data->parameters[controller_defs::spline::node5_y_idx],
        _controller_data->parameters[controller_defs::spline::node6_y_idx],
        _controller_data->parameters[controller_defs::spline::node7_y_idx],
        _controller_data->parameters[controller_defs::spline::node8_y_idx],
        _controller_data->parameters[controller_defs::spline::node9_y_idx],
        _controller_data->parameters[controller_defs::spline::node10_y_idx],
        _controller_data->parameters[controller_defs::spline::node11_y_idx],
        _controller_data->parameters[controller_defs::spline::node12_y_idx],
    };
```

Note the line right after this in `calc_motor_cmd`
(`float torque_cmd = _spline_interpolate(x, y, percent_gait);`) doesn't need
to change — it just passes the arrays through.

Edit 5 — Node count in the interpolator (`ExoCode/src/Controller.cpp`, `Spline::_spline_interpolate`)
--------------------------------------------------------------------------------------------------------
Find, near the top of the function (currently line 899):

```cpp
    const int n = 5;
```

Replace with:

```cpp
    const int n = 12;
```

This is the only change inside `_spline_interpolate` — the tridiagonal
solve and evaluation are already written generically in terms of `n`
(`y2[n]`, `u[n - 1]`, etc.), so nothing else in that function needs to
change. (If you also applied the PCHIP swap from
`Test results/Switching_spline_controller_to_pchip.md`, make this same
`n = 5` → `n = 12` edit in `_pchip_interpolate` instead/as well — that
function was written with the identical `const int n = 5;` pattern.)

Edit 6 — GUI column budget (`ExoCode/src/ListCtrlParams.h`)
----------------------------------------------------------------
Find (currently line 38):

```cpp
	const int MAX_COLUMNS = 30;
```

Replace with:

```cpp
	const int MAX_COLUMNS = 34;
```

Why 34: the names row (`readAndParseFifthRow`, `ListCtrlParams.cpp:478-623`)
writes a 4-column prefix (`PREFIX_COLS`, joint name / joint id / controller
name / controller id — `ListCtrlParams.h:74`) before the CSV's own data
columns. 12 nodes need 30 data columns (24 node values + 6 flags/gains), so
the names row needs `4 + 30 = 34` total columns. The values row
(`readAndParseValuesRow`) only reserves 3 prefix columns, so it needs `3 +
30 = 33` — less than 34, so 34 covers both rows.
Raising `MAX_COLUMNS` also raises `MAX_MESSAGE_SIZE` (`ListCtrlParams.h:48-49`,
computed from `MAX_COLUMNS`), which just grows the pre-sized transmission
buffer accordingly — no other edit needed there.

Edit 7 — Parameter CSV files (all four `spline.csv` files)
------------------------------------------------------------------
Update all four, since the `Spline` class/layout is shared:
- `SDCard/ankleControllers/spline.csv`
- `SDCard/hipControllers/spline.csv`
- `SDCard/arm1Controllers/spline.csv`
- `SDCard/arm2Controllers/spline.csv`

Every row in these files currently has exactly 16 comma-separated fields
(one per parameter) — confirmed by parsing each file with Python's `csv`
module, all 6 rows come back with `len(row) == 16`. For 12 nodes every row
needs exactly **30** fields.

Current `ankleControllers/spline.csv`:
```
5,"header Size, the first N rows will be ignored, except for this first cells in the first two rows",,,,,,,,,,,,,,
16,"parameter number, the number of parameters to read per line",,,,,,,,,,,,,,
,Parameter list for the  spline controller,,,,,,,,,,,,,,
,Parameter order:,,,,,,,,,,,,,,
Node1_x,Node1_y,Node2_x,Node2_y,Node3_x,Node3_y,Node4_x,Node4_y,Node5_x,Node5_y,1=sim %gait,1=%gait 0=%stance,PID Flag,P Gain,I Gain,D Gain
0,0,25,0,48,-12,63,0,100,0,0,1,0,0,0,0
```

Replace with (30 fields per row — the header rows are padded with trailing
commas purely for visual column alignment in a spreadsheet; only the first
1-2 cells of rows 1-4 are actually read):
```
5,"header Size, the first N rows will be ignored, except for this first cells in the first two rows",,,,,,,,,,,,,,,,,,,,,,,,,,,,
30,"parameter number, the number of parameters to read per line",,,,,,,,,,,,,,,,,,,,,,,,,,,,
,Parameter list for the  spline controller,,,,,,,,,,,,,,,,,,,,,,,,,,,,
,Parameter order:,,,,,,,,,,,,,,,,,,,,,,,,,,,,
Node1_x,Node1_y,Node2_x,Node2_y,Node3_x,Node3_y,Node4_x,Node4_y,Node5_x,Node5_y,Node6_x,Node6_y,Node7_x,Node7_y,Node8_x,Node8_y,Node9_x,Node9_y,Node10_x,Node10_y,Node11_x,Node11_y,Node12_x,Node12_y,1=sim %gait,1=%gait 0=%stance,PID Flag,P Gain,I Gain,D Gain
0,0,15,0,25,0,35,-3,45,-8,50,-12,55,-8,60,-3,65,0,75,0,85,0,100,0,0,1,0,0,0,0
```

The example data row above is **illustrative only** — it reshapes the
original 5-node ankle push-off pulse `(0,0)(25,0)(48,-12)(63,0)(100,0)` onto
12 evenly-ish spaced nodes with the same overall shape (flat, dips to -12
around mid-stance, back to flat through swing) and keeps the original
flags (`sim_gait=0, use_percent_gait=1, PID=0, gains=0`). Replace the
numbers with whatever profile you actually want to tune — the important
constraints are: **exactly 30 values**, **x strictly increasing**
(`_spline_interpolate` returns 0 as a safety fallback if not — see
`Controller.cpp:903-909`), and node1 at `x=0` / node12 at `x=100` if you
want the existing "flat before first node, flat after last node" behavior
to still bookend the gait cycle.

For `hipControllers/spline.csv`, `arm1Controllers/spline.csv`, and
`arm2Controllers/spline.csv` (all three currently share this data row:
`0,0,25,1,50,3,75,-3,100,0,1,0,0,0,0,0`), apply the same header/name-row
change as above, and for the value row a 12-node reshaping of that curve,
e.g.:
```
0,0,9,0.3,18,0.7,27,1,36,1.8,45,2.5,55,3,64,0.5,73,-3,82,-1.5,91,-0.3,100,0,1,0,0,0,0,0
```
(again illustrative — same original shape: rises to a +3 peak near 50%,
falls through a -3 trough near 75%, back to 0 by 100%, `sim_gait=1,
use_percent_gait=0, PID=0, gains=0` preserved.)

---

After the edits
----------------
1. Recompile and flash the Teensy (`ExoCode/ExoCode.ino`) — the node count
   is compiled in, so this always requires a re-flash, same as the 5→7 case.
2. Confirm each `spline.csv`'s parameter-count cell (row 2, first field) is
   `30`, matching `controller_defs::spline::num_parameter`.
3. Once flashed, node positions/torques can be tuned freely via the CSV or
   live from the GUI without another recompile — only the **count** (this
   whole change) requires a firmware rebuild.

Quick checklist (5 → 12 nodes)
--------------------------------
- [ ] `ControllerData.h` — add `node6_*`…`node12_*` indices, shift
      `sim_gait_idx`…`d_gain_idx` to 24-29, `num_parameter = 30`
- [ ] `ControllerData.h` — `max_parameters = spline::num_parameter` (was
      `spv2::num_parameter`); 22 → 30
- [ ] `ControllerData.cpp` — `spline_bounds[]` expanded to 30 entries
- [ ] `Controller.cpp` `calc_motor_cmd` — `x[12]` / `y[12]` with
      node6…node12 reads
- [ ] `Controller.cpp` `_spline_interpolate` (and `_pchip_interpolate` if
      present) — `const int n = 12;`
- [ ] `ListCtrlParams.h` — `MAX_COLUMNS = 34` (was 30)
- [ ] All four `spline.csv` files — count cell `30`, 30 columns in the
      name row and every value row
- [ ] Recompile + flash
- [ ] Bench-test each joint you actually use before trusting it on a
      person, same precautions as `Useful guides by us/Using_spline_controller.md`
      (sim_gait sweep first, verify ±15 Nm clamp still applies
      (`Controller.cpp:865-872`, unchanged by this edit), direction
      calibration before real torque)
