# Navigation system

`navigation_system` is a **waypoint and route manager with a hand-entered obstacle list**. It reads GNSS position, keeps up to five routes of waypoints, advances to the next waypoint when the rover gets close, reports range and bearing to it, and edits routes to avoid circular keep-out zones. It does **not** perceive the world and it does **not** command the drive system; see [known-issues.md](known-issues.md).

Nodes:

| Node | File | Role |
|---|---|---|
| `NAVnode` | `src/NAVnode.cpp` | State, services, 200 ms main timer, route editing |
| `NAVDisplay` | `src/NAVDisplay.cpp` | raylib map window; talks to `NAVnode` through services and the `NAVDisplayInfo` topic |
| `gps_bridge_node.py` | `src/gps_bridge_node.py` | NMEA → `sensor_msgs/NavSatFix` on `/gps/fix` (used by `rover_ekf` and the dashboard) |

Classes: `Waypoint` (position in three coordinate systems), `Obstacle` (a waypoint with an integer keep-out radius in metres), `ListOfWaypoints` (doubly linked list of `ListNode`), `Sprite` (display).

## Data model

- Five preplanned routes (`Route1`–`Route5`); each is a doubly linked list of waypoints. `routeToFollow` and `routeToEdit` are chosen independently.
- Obstacles live in one dynamic array, initially 10 entries, grown by 10 whenever it fills.
- `circumnavigationStyle` selects what happens when a route crosses an obstacle (below).
- `preference` (`local`, `geodetic`, `earthCentred`) chooses which representation is shown to the operator.
- Arrival: the node advances to the next (or previous, if `travellingForwards` is false) waypoint when the rover is within **2 m** of the current one.
- Track log: every 100 timer cycles (20 s) the position is appended to a log, capped at 250 points, written to `captainsLog.txt` on shutdown.

## Coordinate systems and the maths

A waypoint can be created from local `(x, y)`, geodetic `(longitude, latitude, altitude)`, or Earth-centred `(X, Y, Z)`, and always holds all three. **The geodetic constructor takes longitude first, then latitude.**

### WGS-84 constants

$$ a = 6378137.0\ \text{m},\qquad f = \frac{1}{298.257223565},\qquad e^2 = f(2-f) $$

### Geodetic → Earth-centred (ECEF)

With latitude $\varphi$, longitude $\lambda$ (radians) and altitude $h$:

$$ N(\varphi) = \frac{a}{\sqrt{1 - e^2 \sin^2\varphi}} $$

$$ X = (N+h)\cos\varphi\cos\lambda,\quad Y = (N+h)\cos\varphi\sin\lambda,\quad Z = \big(N(1-e^2)+h\big)\sin\varphi $$

### Earth-centred → local East-North

The local frame is anchored at the **home** waypoint $(\varphi_0, \lambda_0)$ with ECEF position $(X_0,Y_0,Z_0)$. Let $\Delta = (X-X_0,\ Y-Y_0,\ Z-Z_0)$. The east and north components are

$$ x = -\sin\lambda_0\,\Delta_X + \cos\lambda_0\,\Delta_Y $$

$$ y = -\sin\varphi_0\cos\lambda_0\,\Delta_X - \sin\varphi_0\sin\lambda_0\,\Delta_Y + \cos\varphi_0\,\Delta_Z $$

The "up" component is dropped; the local frame is 2-D. The inverse, local → ECEF, adds the transposed rotation to the home position:

$$ \Delta_X = -\sin\lambda_0\,x - \sin\varphi_0\cos\lambda_0\,y,\quad \Delta_Y = \cos\lambda_0\,x - \sin\varphi_0\sin\lambda_0\,y,\quad \Delta_Z = \cos\varphi_0\,y $$

The local frame is a flat tangent plane at home with up = 0, so it is only meaningful near home (a few kilometres at most).

### Earth-centred → geodetic

An iterative method (5 passes) starting from $\varphi = \operatorname{atan2}\big(Z,\ p(1-e^2)\big)$, with $p=\sqrt{X^2+Y^2}$ and $h=0$:

$$ h = \frac{p}{\cos\varphi} - N(\varphi),\qquad \varphi \leftarrow \operatorname{atan2}\!\left(Z,\ p\left(1 - e^2\frac{N}{N+h}\right)\right),\qquad \lambda = \operatorname{atan2}(Y, X) $$

### Fix applied in this repository (unit bug)

The home latitude/longitude are stored in **degrees**, but the rotation formulas above were being evaluated with them as if they were radians. Every local ↔ ECEF conversion was therefore wrong for real GPS coordinates (the error was zero only when home was at 0°, 0°). The conversions now convert to radians first. Checked numerically with home at 51.46° N, 112.70° W: 100 m east and 200 m north maps to 51.461797° N, 112.698561° W (hand estimate 51.461799° N, 112.698558° W), and a +0.001° latitude/longitude offset maps to 69.5 m east, 111.3 m north (hand estimate 69.4 m, 111.2 m). The static helpers `local_To_EarthCentred` and `local_To_GeodeticApprox` previously took `int` arguments (truncating metres) and returned radians while everything else uses degrees; they now take `double` and return degrees.

### Bearing and range to the next waypoint

With $\Delta x, \Delta y$ the local offset from the rover to the waypoint:

$$ \text{range} = \sqrt{\Delta x^2 + \Delta y^2},\qquad \text{bearing} = \operatorname{atan2}(\Delta x,\ \Delta y)\ \ (\text{mod } 2\pi) $$

The bearing is radians clockwise from north (+y), in $[0, 2\pi)$. The earlier code used $\arcsin(\Delta y/\Delta x)$ on already-normalised components, which is not defined whenever $|\Delta y| > |\Delta x|$ and returned NaN; it has been replaced by the `atan2` form.

### NMEA position parsing in `NAVnode`

NMEA gives latitude as `ddmm.mmmm` and longitude as `dddmm.mmmm`. To convert to decimal degrees:

$$ \text{deg} = \lfloor v/100 \rfloor + \frac{v - 100\lfloor v/100 \rfloor}{60} $$

`NAVnode` previously parsed these with `stoi`, which discards everything after the decimal point and never converts minutes to degrees. It now uses `stod` and the conversion above. Longitude is assumed west (negative). It also required `longitude > 0`, which could never be true for a western longitude, so the position never updated; that test is now "non-zero". The Python `gps_bridge_node` parses NMEA correctly, validates the checksum, and requires a non-zero GGA fix quality.

## Obstacle avoidance geometry

Obstacles are circles of integer radius $R$. Three strategies:

| Style | Behaviour |
|---|---|
| `reroute` | Delete any waypoint inside an obstacle. Cheapest and the default. |
| `trackCrawling` | Delete the colliding waypoint but first insert waypoints on the original track just before and after the obstacle so the route "crawls" around its edge. |
| `automatic_Circumnavigation_Off` | Only print a collision message. |

### Closest point on a segment to an obstacle ("point of interest")

For a segment from $A$ to $B$ and obstacle centre $C$:

$$ \hat{u} = \frac{B-A}{|B-A|},\qquad \ell = \operatorname{clamp}\big((C-A)\cdot\hat{u},\ 0,\ |B-A|\big),\qquad P = A + \ell\,\hat{u} $$

The segment collides if $|C-P| < R$. A zero-length segment returns $A$.

### Detour waypoint placement (`generateDetour`)

Let $\hat{n}$ be the unit vector from $P$ to $C$. Candidates are tried in order and the first one not inside any obstacle is inserted before the next waypoint:

1. $P - (R+1)\hat{n}$ — 1 m past the near edge,
2. $P - R\hat{n}$ — on the near edge,
3. $P + (R+1)\hat{n}$ — 1 m past the far edge,
4. $P + R\hat{n}$ — on the far edge.

New segments are re-checked recursively. If all four candidates are blocked, the obstacles form a chain; the code walks along the chain to find both ends, builds a waypoint beyond each end, keeps the one that costs the smaller detour, and discards the other.

### Known weaknesses of this geometry

It is a reactive geometric patch, not a planner: no costs, no kinodynamics, no guarantee of finding a path, and chained or overlapping obstacles are handled with special cases. Memory is managed with raw `new`/`delete`; `navnode_fix` corrected several mismatches (for example `delete` on an array allocated with `new[]`).

## Services

`NAVnode` serves (all names exact):

- Route selection: `SelectRouteToFollow`, `SelectRouteToEdit`, `ToggleRouteToFollow`, `ToggleRouteToEdit`, `TargetNextWaypoint`, `TargetPreviousWaypoint`, `ToggleDirection`, `StopNavigating`, `ClearRoute`.
- Add (local / geodetic / earth-centred): `AddLocalWaypoint`, `AddLocalWaypointAtIndex`, `AddLocalObstacle`, `AddGeodeticWaypoint`, `AddGeodeticWaypointAtIndex`, `AddGeodeticObstacle`, `AddEarthCentredWaypoint`, `AddEarthCentredWaypointAtIndex`, `AddEarthCentredObstacle`.
- Remove: `RemoveLocalWaypoint`, `RemoveGeodeticWaypoint`, `RemoveEarthCentredWaypoint`, `RemoveLocalObstacle`, `RemoveGeodeticObstacle`, `RemoveEarthCentredObstacle`, `RemoveLastWaypoint`, `RemoveLastObstacle`.
- Settings: `ResetHome` (requires the password in `Waypoint::password`, currently hard-coded), `ToggleDebug`, `TogglePreference`, `ToggleCircumnavigationStyle`.
- Raw GNSS pass-through: `SendToGNSS`, `ReadFromGNSS`.

Action: `FollowRoute` accepts and cancels any goal but does not execute anything (see known issues).

Topic: `NAVDisplayInfo` (`navigation_interfaces/DisplayInfo`) carries rover and waypoint positions in all three systems, `heading`, `bearing2waypoint`, `range2waypoint`, speed and the current preference/route selection.

## Testing without hardware

[`scripts/tests/waypoint_conversion_test.cpp`](../scripts/tests/waypoint_conversion_test.cpp) checks the coordinate maths with no ROS. [`scripts/tests/navnode_nmea_smoke_test.sh`](../scripts/tests/navnode_nmea_smoke_test.sh) runs the real `NAVnode` against a virtual serial port that feeds it an NMEA sentence, then checks the services, the display topic and the home/local-coordinate round trip. Both pass in the WSL2 environment. Note that the local coordinates are only meaningful after `ResetHome` has set the home waypoint; before that, home is the Earth's centre and `local_x/local_y` are millions of metres.

## Two GPS paths

`NAVnode` opens the GNSS receiver itself (parameters `gnss_port` default `/dev/ttyUSB0`, `gnss_baudrate` default 38400) and parses NMEA internally. `gps_bridge_node` independently opens the same default port and publishes `/gps/fix`. Run only one at a time unless they are given different ports.
