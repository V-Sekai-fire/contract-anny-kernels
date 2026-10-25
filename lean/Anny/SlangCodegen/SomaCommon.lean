import Drape.SlangCodegen.Dsl
import Anny.SlangCodegen.Common

/-!
# `Anny.SlangCodegen.SomaCommon` — shared pieces of the SOMA-X pose kernels

Not a kernel. Statement generators the `Soma*` kernels inline. Matrices are
nine scalar locals as in `Common` (row-major, `<pfx>(3i+j)`); a stored 3×3
is 9 floats row-major, a stored affine is 12 floats `[R | t]` (row r at
`4r … 4r+3`).

Rotations stay 3×3 matrices everywhere: no quaternion is formed in any
kernel. The twist angles SOMA-X takes through quaternions are read off the
matrix instead (`twistAngle`).
-/

namespace Anny.SlangCodegen.SomaCommon

open LeanSlang
open Drape.SlangCodegen.Dsl
open Anny.SlangCodegen.Common

/-- The 3×3 at `buf[base … base+8]` (row-major) into `pfx`. -/
def load9 (pfx buf : String) (base : E) : List St :=
  mat pfx fun i j => at_ buf (base + u (3 * i + j))

/-- Write the 3×3 `r` to `buf[base … base+8]`. -/
def store9 (buf : String) (base : E) (r : String) : List St :=
  r9.map fun k => setAt buf (base + u k) (s r k)

/-- The numerator of the twist about axis `ax` (a runtime uint, 0 = x):
    `R21 − R12`, `R02 − R20` or `R10 − R01`, which is `4 w q_ax` for the
    rotation's quaternion `(q, w)`. -/
def twistNum (r : String) (ax : E) : E :=
  sel (eq ax (u 0)) (m r 2 1 - m r 1 2)
    (sel (eq ax (u 1)) (m r 0 2 - m r 2 0) (m r 1 0 - m r 0 1))

/-- `out = ` SOMA-X's swing-twist angle of the rotation `r` about `ax`,
    `quaternion_twist_angle_xyzw(matrix_to_quaternion_xyzw(r), ax)`, from the
    matrix alone.

    SOMA-X takes the unit quaternion `(q, w)` with `w ≥ 0`, its half-angle
    root `[q, w + 1]`, and returns `4·atan2(q_ax, w + 1)`. With
    `w = ½√(1 + tr)` and `q_ax = num / 4w` (`twistNum`), multiplying both
    arguments of the atan2 by `4w > 0` gives

        4 · atan2(num, (1 + tr) + 2√(1 + tr)),

    the same angle with no quaternion. `1 + tr` is clamped at 0 as SOMA-X
    clamps it. This is not `2·atan2(q_ax, w)`: the two agree only when the
    rotation is a pure twist (the parity test measures how far apart they
    are on the clips). -/
def twistAngle (out r : String) (ax : E) : List St :=
  [ let_ fT (out ++ "_t") (fmax (fl 1.0 + m r 0 0 + m r 1 1 + m r 2 2) (fl 0.0))
  , let_ fT out (fl 4.0 * call "atan2"
      [ twistNum r ax, v (out ++ "_t") + fl 2.0 * call "sqrt" [v (out ++ "_t")] ]) ]

/-- SOMA-X's local Euler channel `ax` of the rotation `r`
    (`_local_euler_xyz_from_matrix`): channel 0 is its local-X override
    `atan2(R21, R11)`; channels 1 and 2 are `matrix_to_euler_xyz`'s
    `asin(−clamp(R20, −1, 1))` and `atan2(R10, R00)`. SOMA-X has no
    separate gimbal-lock branch, so neither has this. -/
def eulerChannel (r : String) (ax : E) : E :=
  sel (eq ax (u 0)) (call "atan2" [m r 2 1, m r 1 1])
    (sel (eq ax (u 1)) (call "asin" [- call "clamp" [m r 2 0, fl (-1.0), fl 1.0]])
      (call "atan2" [m r 1 0, m r 0 0]))

/-- Component `i` of the unit axis `ax` (a runtime uint). -/
def axisUnit (ax : E) (i : Nat) : E := sel (eq ax (u i)) (fl 1.0) (fl 0.0)

/-- `out = ` the twist angle of `r` about `ax` by the plain swing-twist split, built from the IK
    solver's rank-1 Kabsch completion (swing-twist-kusudama `KusKabsch.lean`, the same construction
    as entities-godot `ik_kabsch_6d.cpp`), every step a 3-vector or 3×3:

      v0 = a (the twist axis), u0 = r·a / |r·a|
      (v0, v1, v2) = `frameOf v0`         ref = y if |v0.y| < 0.9 else x,
                                          v1 = norm(v0 × ref), v2 = norm(v0 × v1)
      (c1, c2) = `canonCompletion`        R0 v1, R0 v2 with R0 the shortest arc v0 → u0:
                                          k = (v0 × u0)/|v0 × u0|, `rod k (v0·u0) |v0 × u0|`
      swing  R0 = U Vᵀ,  U = [u0, c1, c2],  V = [v0, v1, v2]   (`applyR`)
      twist  T  = R0ᵀ · r                 a rotation about a alone
      angle  atan2 of T's two off-diagonal and two diagonal entries in the plane normal to a

    `canonCompletion` leaves |v0 × u0| < 1e-9 to the C++ fallback, ported here: aligned keeps
    (v1, v2); antipodal takes c1 = Godot's `u0.get_any_perpendicular()`, c2 = norm(u0 × c1).
    `norm` keeps KusKabsch's 1e-12 floor.

    This is the standard twist, `2·atan2(q_ax, w)`. SOMA-X's half-angle form (`twistAngle`) is twice
    the standard twist of `√r`; the two differ once `r` has swing, so the kernels select it with
    `twistForm`; the Kabsch split is the default. -/
def swingTwistAngle (out r : String) (ax : E) : List St :=
  let n := fun x => out ++ x
  let normed := fun (dst src : String) =>
    [ let_ fT (dst ++ "_l") (call "sqrt" [dot3 src src]) ] ++
    vec dst (fun i => sel (gt (v (dst ++ "_l")) (.litFloatExact 1e-12)) (s src i / v (dst ++ "_l")) (s src i))
  -- rod k cs sn w = cs w + sn (k × w) + (k·w)(1 − cs) k
  let rod := fun (dst k w : String) =>
    cross (dst ++ "_kx") k w ++
    [ let_ fT (dst ++ "_kw") (dot3 k w) ] ++
    vec dst (fun i => v (n "_cs") * s w i + v (n "_sn") * s (dst ++ "_kx") i +
                      v (dst ++ "_kw") * (fl 1.0 - v (n "_cs")) * s k i)
  vec (n "_v0") (fun i => axisUnit ax i) ++
  vec (n "_ru") (fun i => m r i 0 * s (n "_v0") 0 + m r i 1 * s (n "_v0") 1 + m r i 2 * s (n "_v0") 2) ++
  normed (n "_u0") (n "_ru") ++
  [ let_ bT (n "_fy") (lt (fabs (s (n "_v0") 1)) (fl 0.9)) ] ++
  vec (n "_ref") (fun i => if i == 0 then sel (v (n "_fy")) (fl 0.0) (fl 1.0)
                           else if i == 1 then sel (v (n "_fy")) (fl 1.0) (fl 0.0) else fl 0.0) ++
  cross (n "_v1r") (n "_v0") (n "_ref") ++ normed (n "_v1") (n "_v1r") ++
  cross (n "_v2r") (n "_v0") (n "_v1") ++ normed (n "_v2") (n "_v2r") ++
  cross (n "_ax") (n "_v0") (n "_u0") ++
  [ let_ fT (n "_sn") (call "sqrt" [dot3 (n "_ax") (n "_ax")])
  , let_ fT (n "_cs") (dot3 (n "_v0") (n "_u0")) ] ++
  vec (n "_k") (fun i => s (n "_ax") i / fmax (v (n "_sn")) (.litFloatExact 1e-9)) ++
  rod (n "_c1") (n "_k") (n "_v1") ++ rod (n "_c2") (n "_k") (n "_v2") ++
  [ if_ (lt (v (n "_sn")) (.litFloatExact 1e-9))
      [ if_ (gt (v (n "_cs")) (fl 0.0))
          (r3.map (fun i => setv (n "_c1" ++ toString i) (s (n "_v1") i)) ++
           r3.map (fun i => setv (n "_c2" ++ toString i) (s (n "_v2") i)))
          ([ let_ bT (n "_px") (and_ (le (fabs (s (n "_u0") 0)) (fabs (s (n "_u0") 1)))
                                     (le (fabs (s (n "_u0") 0)) (fabs (s (n "_u0") 2)))) ] ++
           vec (n "_pr") (fun i => if i == 0 then sel (v (n "_px")) (fl 1.0) (fl 0.0)
                                   else if i == 1 then sel (v (n "_px")) (fl 0.0) (fl 1.0) else fl 0.0) ++
           cross (n "_p1") (n "_u0") (n "_pr") ++ normed (n "_p1n") (n "_p1") ++
           cross (n "_p2") (n "_u0") (n "_p1n") ++ normed (n "_p2n") (n "_p2") ++
           r3.map (fun i => setv (n "_c1" ++ toString i) (s (n "_p1n") i)) ++
           r3.map (fun i => setv (n "_c2" ++ toString i) (s (n "_p2n") i))) ] ] ++
  -- R0 = U Vᵀ: entry (i, j) = u0_i v0_j + c1_i v1_j + c2_i v2_j
  mat (n "_R0") (fun i j => s (n "_u0") i * s (n "_v0") j + s (n "_c1") i * s (n "_v1") j +
                            s (n "_c2") i * s (n "_v2") j) ++
  matTMul (n "_T") (n "_R0") r ++
  [ let_ fT out (call "atan2"
      [ sel (eq ax (u 0)) (m (n "_T") 2 1 - m (n "_T") 1 2)
          (sel (eq ax (u 1)) (m (n "_T") 0 2 - m (n "_T") 2 0) (m (n "_T") 1 0 - m (n "_T") 0 1))
      , sel (eq ax (u 0)) (m (n "_T") 1 1 + m (n "_T") 2 2)
          (sel (eq ax (u 1)) (m (n "_T") 2 2 + m (n "_T") 0 0) (m (n "_T") 0 0 + m (n "_T") 1 1)) ]) ]

end Anny.SlangCodegen.SomaCommon
