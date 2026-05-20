from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_vent_tower_safety")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.62, 0.60, 1.0))
    dark_void = model.material("dark_vent_void", rgba=(0.04, 0.045, 0.045, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.05, 1.0))
    hinge_black = model.material("blackened_hinge", rgba=(0.05, 0.05, 0.045, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.8, 0.05, 0.025, 1.0))
    rubber = model.material("rubber_black", rgba=(0.01, 0.01, 0.01, 1.0))

    tower = model.part("tower")

    def tbox(name: str, size, xyz, material: Material = galvanized, rpy=(0.0, 0.0, 0.0)) -> None:
        tower.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def tcyl(name: str, radius: float, length: float, xyz, material: Material = galvanized, rpy=(0.0, 0.0, 0.0)) -> None:
        tower.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Roof curb and heavy-gauge tower shell.  The main side openings are built as
    # real frames rather than a painted-on rectangle, leaving visible airflow gaps.
    tbox("roof_flashing", (1.34, 1.04, 0.035), (0.0, 0.0, 0.0175))
    tbox("curb_plinth", (1.12, 0.82, 0.150), (0.0, 0.0, 0.110))
    tbox("duct_shadow", (0.58, 0.38, 0.035), (0.0, 0.0, 0.198), dark_void)

    # Four load-bearing corner posts and side webs.
    for x in (-0.36, 0.36):
        for y in (-0.26, 0.26):
            tbox(f"corner_post_{x:+.0f}_{y:+.0f}", (0.075, 0.075, 1.03), (x, y, 0.695))
    tbox("side_wall_0", (0.72, 0.045, 1.00), (0.0, -0.260, 0.685))
    tbox("side_wall_1", (0.72, 0.045, 1.00), (0.0, 0.260, 0.685))

    # Front and rear framed vent openings.
    for face, x, sign in (("front", 0.385, 1.0), ("rear", -0.385, -1.0)):
        tbox(f"{face}_lower_rail", (0.050, 0.520, 0.160), (x, 0.0, 0.320))
        tbox(f"{face}_upper_rail", (0.050, 0.520, 0.125), (x, 0.0, 1.010))
        tbox(f"{face}_side_rail_0", (0.050, 0.080, 0.610), (x, -0.220, 0.695))
        tbox(f"{face}_side_rail_1", (0.050, 0.080, 0.610), (x, 0.220, 0.695))
        tbox(f"{face}_center_mullion", (0.052, 0.030, 0.600), (x + 0.002 * sign, 0.0, 0.695))
        tbox(f"{face}_screen_backer", (0.018, 0.390, 0.525), (x - 0.010 * sign, 0.0, 0.700), dark_void)
        for i, z in enumerate((0.460, 0.555, 0.650, 0.745, 0.840, 0.935)):
            # Broad, slightly downturned louver plates bridge into the side rails.
            tbox(
                f"{face}_louver_{i}",
                (0.085, 0.430, 0.020),
                (x + 0.020 * sign, 0.0, z),
                galvanized,
                rpy=(0.0, -0.26 * sign, 0.0),
            )

    # Boxed top throat frame under the flap; the top remains visibly open.
    tbox("top_rear_rail", (0.090, 0.620, 0.060), (-0.365, 0.0, 1.180))
    tbox("top_front_rail", (0.090, 0.620, 0.060), (0.365, 0.0, 1.180))
    tbox("top_side_rail_0", (0.730, 0.075, 0.060), (0.0, -0.292, 1.180))
    tbox("top_side_rail_1", (0.730, 0.075, 0.060), (0.0, 0.292, 1.180))
    tbox("top_opening_shadow", (0.690, 0.485, 0.018), (0.040, 0.0, 1.160), dark_void)

    # Roof flange fasteners and vent-frame fasteners.
    for i, x in enumerate((-0.52, -0.25, 0.25, 0.52)):
        for j, y in enumerate((-0.40, 0.40)):
            tcyl(f"base_bolt_{i}_{j}", 0.017, 0.012, (x, y, 0.191), hinge_black)
    for face, x, sign in (("front", 0.414, 1.0), ("rear", -0.414, -1.0)):
        for i, y in enumerate((-0.245, 0.245)):
            for j, z in enumerate((0.375, 1.035)):
                tcyl(
                    f"{face}_frame_bolt_{i}_{j}",
                    0.012,
                    0.010,
                    (x, y, z),
                    hinge_black,
                    rpy=(0.0, math.pi / 2.0 * sign, 0.0),
                )

    # Hinge line: fixed knuckles, vertical bearing plates, and load-spreading
    # leaf plates are separate visible features connected into the top frame.
    hinge_x = -0.430
    hinge_z = 1.255
    fixed_knuckles = [(-0.285, 0.130), (0.0, 0.140), (0.285, 0.130)]
    for i, (y, length) in enumerate(fixed_knuckles):
        tcyl(
            f"fixed_barrel_{i}",
            0.025,
            length,
            (hinge_x, y, hinge_z),
            hinge_black,
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        )
        tbox(f"fixed_bearing_plate_{i}", (0.020, length + 0.018, 0.105), (hinge_x + 0.004, y, hinge_z - 0.055), galvanized)
        tbox(f"fixed_leaf_{i}", (0.095, length + 0.018, 0.010), (hinge_x + 0.042, y, 1.208), galvanized)
        for end_y in (y - length * 0.32, y + length * 0.32):
            tcyl(
                f"fixed_leaf_bolt_{i}_{end_y:+.0f}",
                0.009,
                0.007,
                (hinge_x + 0.064, end_y, 1.216),
                hinge_black,
            )

    # Triangular-looking welded gusset load paths represented as diagonal strap
    # plates from hinge brackets down into the side/corner structure.
    tbox("hinge_gusset_0", (0.030, 0.085, 0.360), (-0.500, -0.315, 1.075), galvanized, rpy=(0.0, 0.48, 0.0))
    tbox("hinge_gusset_1", (0.030, 0.085, 0.360), (-0.500, 0.315, 1.075), galvanized, rpy=(0.0, 0.48, 0.0))

    # Safety guard rails and over-travel stop structure.  Every rail is tied
    # back to the tower by feet or diagonal braces.
    for y, suffix in ((-0.490, "0"), (0.490, "1")):
        tbox(f"guard_mount_bridge_{suffix}", (0.180, 0.260, 0.028), (0.070, y * 0.765, 1.125), safety_yellow)
        tbox(f"guard_saddle_{suffix}", (0.050, 0.050, 0.190), (0.070, y, 1.170), safety_yellow)
        tbox(f"guard_foot_{suffix}", (0.180, 0.060, 0.025), (0.070, y, 1.218), safety_yellow)
        tbox(f"guard_post_{suffix}", (0.045, 0.045, 0.730), (0.070, y, 1.570), safety_yellow)
        tbox(f"guard_back_post_{suffix}", (0.045, 0.045, 0.500), (-0.345, y, 1.455), safety_yellow)
        tbox(f"guard_top_rail_{suffix}", (0.470, 0.038, 0.038), (-0.135, y, 1.895), safety_yellow)
        tbox(f"guard_diag_{suffix}", (0.038, 0.038, 0.570), (-0.165, y, 1.565), safety_yellow, rpy=(0.0, -0.82, 0.0))
        tbox(f"overtravel_stop_{suffix}", (0.062, 0.075, 0.055), (0.075, y, 1.925), rubber)
        tbox(f"stop_backer_{suffix}", (0.085, 0.085, 0.075), (0.050, y, 1.900), safety_yellow)

    # Lockout bracket on the fixed structure.
    tbox("lockout_anchor", (0.018, 0.090, 0.130), (0.440, -0.535, 1.270), lockout_red)
    tcyl("lockout_anchor_hole", 0.018, 0.004, (0.451, -0.535, 1.300), dark_void, rpy=(0.0, math.pi / 2.0, 0.0))
    tbox("lockout_anchor_bridge", (0.105, 0.035, 0.030), (0.388, -0.505, 1.225), lockout_red)
    tbox("lockout_guard_strut", (0.305, 0.035, 0.030), (0.245, -0.492, 1.225), lockout_red)

    flap = model.part("flap")

    def fbox(name: str, size, xyz, material: Material = galvanized, rpy=(0.0, 0.0, 0.0)) -> None:
        flap.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def fcyl(name: str, radius: float, length: float, xyz, material: Material = galvanized, rpy=(0.0, 0.0, 0.0)) -> None:
        flap.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Child frame origin is the hinge pin line.  The weather flap starts just
    # forward of the barrels so the closed panel clears the fixed knuckles.
    fbox("flap_plate", (0.920, 0.740, 0.040), (0.500, 0.0, -0.015))
    fbox("front_drip_lip", (0.055, 0.740, 0.115), (0.965, 0.0, -0.071))
    fbox("side_skirt_0", (0.885, 0.045, 0.095), (0.510, -0.392, -0.060))
    fbox("side_skirt_1", (0.885, 0.045, 0.095), (0.510, 0.392, -0.060))
    fbox("front_reinforcing_rib", (0.060, 0.650, 0.035), (0.845, 0.0, 0.022))
    fbox("hinge_reinforcing_rib", (0.070, 0.650, 0.035), (0.130, 0.0, 0.022))
    fbox("center_stiffener", (0.650, 0.045, 0.032), (0.495, 0.0, 0.023))
    fbox("diagonal_stiffener_0", (0.640, 0.032, 0.034), (0.500, -0.160, 0.020), rpy=(0.0, 0.0, 0.38))
    fbox("diagonal_stiffener_1", (0.640, 0.032, 0.034), (0.500, 0.160, 0.020), rpy=(0.0, 0.0, -0.38))

    moving_knuckles = [(-0.145, 0.130), (0.145, 0.130)]
    for i, (y, length) in enumerate(moving_knuckles):
        fcyl(
            f"moving_barrel_{i}",
            0.024,
            length,
            (0.0, y, 0.0),
            hinge_black,
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        )
        fbox(f"moving_leaf_{i}", (0.130, length + 0.018, 0.014), (0.074, y, -0.003))
        for bolt_x in (0.080, 0.128):
            fcyl(f"moving_leaf_bolt_{i}_{bolt_x:.2f}", 0.009, 0.007, (bolt_x, y, 0.006), hinge_black)

    # Moving lockout ear and two stop lugs; both are tied directly into the flap
    # skin and stiffeners.
    fbox("lockout_ear", (0.024, 0.095, 0.135), (0.875, -0.438, 0.020), lockout_red)
    fcyl("lockout_ear_hole", 0.017, 0.005, (0.887, -0.438, 0.050), dark_void, rpy=(0.0, math.pi / 2.0, 0.0))
    fbox("stop_lug_0", (0.070, 0.060, 0.070), (0.830, -0.382, 0.036), safety_yellow)
    fbox("stop_lug_1", (0.070, 0.060, 0.070), (0.830, 0.382, 0.036), safety_yellow)
    for i, y in enumerate((-0.245, 0.0, 0.245)):
        fcyl(f"flap_skin_bolt_{i}", 0.012, 0.008, (0.740, y, 0.008), hinge_black)

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.1, lower=0.0, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("tower_to_flap")

    limits = hinge.motion_limits
    ctx.check(
        "heavy duty bounded flap hinge",
        limits is not None
        and abs(limits.lower - 0.0) < 1e-6
        and 0.85 <= limits.upper <= 0.95
        and limits.effort >= 200.0
        and tuple(hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}, limits={limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            positive_elem="flap_plate",
            negative_elem="top_front_rail",
            min_gap=0.006,
            max_gap=0.020,
            name="closed flap clears top throat frame",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="xy",
            elem_a="flap_plate",
            elem_b="top_opening_shadow",
            min_overlap=0.35,
            name="weather flap covers framed vent throat",
        )
        ctx.expect_gap(
            flap,
            tower,
            axis="y",
            positive_elem="moving_barrel_1",
            negative_elem="fixed_barrel_1",
            min_gap=0.006,
            max_gap=0.020,
            name="hinge knuckles have assembly clearance",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="xz",
            elem_a="moving_barrel_1",
            elem_b="fixed_barrel_1",
            min_overlap=0.040,
            name="moving and fixed barrels share hinge line",
        )
        ctx.expect_gap(
            flap,
            tower,
            axis="y",
            positive_elem="lockout_ear",
            negative_elem="lockout_anchor",
            min_gap=0.002,
            max_gap=0.015,
            name="lockout plates sit adjacent without collision",
        )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({hinge: limits.upper}):
        open_aabb = ctx.part_world_aabb(flap)
        lug_center = aabb_center(ctx.part_element_world_aabb(flap, elem="stop_lug_1"))
        stop_center = aabb_center(ctx.part_element_world_aabb(tower, elem="overtravel_stop_1"))

    ctx.check(
        "flap opens upward toward overtravel stops",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.55,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )
    ctx.check(
        "stop lug reaches guarded stop sector",
        lug_center is not None
        and stop_center is not None
        and abs(lug_center[0] - stop_center[0]) < 0.080
        and abs(lug_center[1] - stop_center[1]) < 0.130
        and abs(lug_center[2] - stop_center[2]) < 0.080,
        details=f"lug_center={lug_center}, stop_center={stop_center}",
    )

    return ctx.report()


object_model = build_object_model()
