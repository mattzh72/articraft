from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _x_cylinder(part, name: str, radius: float, length: float, xyz, material) -> None:
    """Cylinder whose long axis is world/local X."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_cylinder(part, name: str, radius: float, length: float, xyz, material) -> None:
    """Cylinder whose long axis is world/local Y."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_watch_winder_box")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    black_plate = model.material("black_plate", rgba=(0.015, 0.017, 0.018, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.55, 0.57, 0.55, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.68, 0.05, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.85, 0.05, 0.03, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    polycarbonate = model.material("clear_polycarbonate", rgba=(0.55, 0.72, 0.82, 0.34))
    amber_window = model.material("amber_window", rgba=(1.0, 0.72, 0.22, 0.30))

    base = model.part("base")

    # Hollow presentation-box tray: separate floor and welded side plates keep the
    # top visibly open while giving the heavy-duty variant a bolted steel shell.
    base.visual(Box((0.50, 0.36, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.0125)), material=dark_steel, name="floor_plate")
    base.visual(Box((0.42, 0.28, 0.006)), origin=Origin(xyz=(0.0, 0.0, 0.031)), material=black_plate, name="floor_liner")
    base.visual(Box((0.025, 0.36, 0.160)), origin=Origin(xyz=(-0.2375, 0.0, 0.105)), material=dark_steel, name="side_wall_0")
    base.visual(Box((0.025, 0.36, 0.160)), origin=Origin(xyz=(0.2375, 0.0, 0.105)), material=dark_steel, name="side_wall_1")
    base.visual(Box((0.50, 0.025, 0.160)), origin=Origin(xyz=(0.0, -0.1675, 0.105)), material=dark_steel, name="front_wall")
    base.visual(Box((0.50, 0.025, 0.160)), origin=Origin(xyz=(0.0, 0.1675, 0.105)), material=dark_steel, name="rear_wall")

    for i, (x, y) in enumerate(((-0.226, -0.156), (0.226, -0.156), (-0.226, 0.156), (0.226, 0.156))):
        base.visual(Box((0.040, 0.040, 0.172)), origin=Origin(xyz=(x, y, 0.111)), material=satin_steel, name=f"corner_post_{i}")
        base.visual(Cylinder(radius=0.007, length=0.004), origin=Origin(xyz=(x, y, 0.199)), material=satin_steel, name=f"corner_bolt_{i}")

    # Exterior rub rails and bolted front armor plate.
    base.visual(Box((0.54, 0.020, 0.040)), origin=Origin(xyz=(0.0, -0.190, 0.085)), material=black_plate, name="front_armor")
    base.visual(Box((0.54, 0.018, 0.022)), origin=Origin(xyz=(0.0, 0.189, 0.085)), material=black_plate, name="rear_armor")
    base.visual(Box((0.022, 0.39, 0.024)), origin=Origin(xyz=(-0.261, 0.0, 0.084)), material=black_plate, name="side_rail_0")
    base.visual(Box((0.022, 0.39, 0.024)), origin=Origin(xyz=(0.261, 0.0, 0.084)), material=black_plate, name="side_rail_1")

    for i, x in enumerate((-0.205, -0.105, 0.105, 0.205)):
        base.visual(Box((0.014, 0.006, 0.014)), origin=Origin(xyz=(x, -0.190, 0.112)), material=satin_steel, name=f"front_armor_bolt_{i}")
        base.visual(Box((0.014, 0.006, 0.014)), origin=Origin(xyz=(x, 0.189, 0.096)), material=satin_steel, name=f"rear_armor_bolt_{i}")

    # One-piece trunnion yoke: floor-mounted bearing support with two explicit
    # cheek points for the rotating watch cradle shaft.
    base.visual(Box((0.455, 0.130, 0.012)), origin=Origin(xyz=(0.0, -0.006, 0.031)), material=satin_steel, name="cradle_yoke")
    base.visual(Box((0.030, 0.112, 0.136)), origin=Origin(xyz=(-0.215, -0.006, 0.093)), material=safety_yellow, name="yoke_cheek_0")
    base.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(-0.201, -0.006, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="bearing_boss_0",
    )
    base.visual(Box((0.030, 0.112, 0.136)), origin=Origin(xyz=(0.215, -0.006, 0.093)), material=safety_yellow, name="yoke_cheek_1")
    base.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.201, -0.006, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="bearing_boss_1",
    )
    for i, x in enumerate((-0.175, -0.095, 0.095, 0.175)):
        base.visual(Cylinder(radius=0.0065, length=0.005), origin=Origin(xyz=(x, -0.052, 0.0395)), material=satin_steel, name=f"yoke_bolt_front_{i}")
        base.visual(Cylinder(radius=0.0065, length=0.005), origin=Origin(xyz=(x, 0.040, 0.0395)), material=satin_steel, name=f"yoke_bolt_rear_{i}")

    # Fixed guarding tied back into the tray, not a floating cage.
    for i, x in enumerate((-0.205, 0.205)):
        base.visual(Box((0.020, 0.020, 0.145)), origin=Origin(xyz=(x, -0.126, 0.098)), material=safety_yellow, name=f"front_guard_post_{i}")
        base.visual(Box((0.020, 0.020, 0.145)), origin=Origin(xyz=(x, 0.090, 0.098)), material=safety_yellow, name=f"rear_guard_post_{i}")
    base.visual(Box((0.430, 0.018, 0.018)), origin=Origin(xyz=(0.0, -0.126, 0.177)), material=safety_yellow, name="front_guard_rail")
    base.visual(Box((0.430, 0.018, 0.018)), origin=Origin(xyz=(0.0, 0.090, 0.177)), material=safety_yellow, name="rear_guard_rail")
    base.visual(Box((0.018, 0.234, 0.018)), origin=Origin(xyz=(-0.205, -0.018, 0.177)), material=safety_yellow, name="guard_side_rail_0")
    base.visual(Box((0.018, 0.234, 0.018)), origin=Origin(xyz=(0.205, -0.018, 0.177)), material=safety_yellow, name="guard_side_rail_1")
    base.visual(Box((0.410, 0.006, 0.112)), origin=Origin(xyz=(0.0, -0.130, 0.105)), material=polycarbonate, name="front_guard_shield")

    # Lid hinge base knuckles are mounted on segmented leaves so they do not
    # occupy the same spans as the moving lid knuckles.
    base.visual(Box((0.520, 0.026, 0.065)), origin=Origin(xyz=(0.0, 0.191, 0.152)), material=dark_steel, name="hinge_back_plate")
    for i, (x, length) in enumerate(((-0.194, 0.084), (0.0, 0.102), (0.194, 0.084))):
        base.visual(Box((length + 0.020, 0.014, 0.055)), origin=Origin(xyz=(x, 0.204, 0.180)), material=satin_steel, name=f"hinge_leaf_{i}")
        _x_cylinder(base, f"hinge_knuckle_{i}", 0.011, length, (x, 0.198, 0.207), satin_steel)
        base.visual(Box((length + 0.010, 0.026, 0.018)), origin=Origin(xyz=(x, 0.184, 0.183)), material=dark_steel, name=f"hinge_rib_{i}")

    # High-stress diagonal load paths from rear hinge plate down into side walls.
    for i, x in enumerate((-0.205, 0.205)):
        base.visual(
            Box((0.018, 0.116, 0.018)),
            origin=Origin(xyz=(x, 0.132, 0.141), rpy=(math.radians(-38.0), 0.0, 0.0)),
            material=safety_yellow,
            name=f"hinge_gusset_{i}",
        )
        base.visual(Box((0.044, 0.016, 0.036)), origin=Origin(xyz=(x, 0.156, 0.152)), material=safety_yellow, name=f"lid_stop_block_{i}")

    # Lockout pivot support cheeks on the front armor plate.
    base.visual(Box((0.140, 0.020, 0.110)), origin=Origin(xyz=(0.0, -0.190, 0.145)), material=satin_steel, name="lockout_backplate")
    for i, x in enumerate((-0.038, 0.038)):
        base.visual(Box((0.018, 0.030, 0.052)), origin=Origin(xyz=(x, -0.194, 0.158)), material=satin_steel, name=f"lockout_lug_{i}")
        _x_cylinder(base, f"lockout_lug_boss_{i}", 0.010, 0.018, (x, -0.194, 0.160), satin_steel)

    # Lid: an armored frame with a transparent inspection window and reinforced
    # hinge edge. Its part frame is exactly on the hinge line.
    lid = model.part("lid")
    lid.visual(Box((0.500, 0.026, 0.026)), origin=Origin(xyz=(0.0, -0.020, 0.000)), material=dark_steel, name="rear_bar")
    lid.visual(Box((0.500, 0.026, 0.026)), origin=Origin(xyz=(0.0, -0.327, 0.000)), material=dark_steel, name="front_bar")
    lid.visual(Box((0.028, 0.300, 0.026)), origin=Origin(xyz=(-0.236, -0.177, 0.000)), material=dark_steel, name="side_bar_0")
    lid.visual(Box((0.028, 0.300, 0.026)), origin=Origin(xyz=(0.236, -0.177, 0.000)), material=dark_steel, name="side_bar_1")
    lid.visual(Box((0.444, 0.286, 0.008)), origin=Origin(xyz=(0.0, -0.170, -0.004)), material=amber_window, name="inspection_window")
    lid.visual(Box((0.510, 0.018, 0.018)), origin=Origin(xyz=(0.0, -0.170, 0.010), rpy=(0.0, 0.0, math.radians(33.0))), material=safety_yellow, name="cross_brace_0")
    lid.visual(Box((0.510, 0.018, 0.018)), origin=Origin(xyz=(0.0, -0.170, 0.010), rpy=(0.0, 0.0, math.radians(-33.0))), material=safety_yellow, name="cross_brace_1")
    for i, x in enumerate((-0.102, 0.102)):
        _x_cylinder(lid, f"hinge_knuckle_{i}", 0.011, 0.082, (x, 0.000, 0.000), satin_steel)
    lid.visual(Box((0.104, 0.020, 0.055)), origin=Origin(xyz=(0.0, -0.340, -0.016)), material=satin_steel, name="latch_striker")
    lid.visual(Box((0.036, 0.024, 0.060)), origin=Origin(xyz=(-0.205, -0.036, 0.035)), material=safety_yellow, name="stop_tab_0")
    lid.visual(Box((0.036, 0.024, 0.060)), origin=Origin(xyz=(0.205, -0.036, 0.035)), material=safety_yellow, name="stop_tab_1")
    for i, (x, y) in enumerate(((-0.190, -0.044), (0.190, -0.044), (-0.190, -0.295), (0.190, -0.295))):
        lid.visual(Cylinder(radius=0.006, length=0.004), origin=Origin(xyz=(x, y, 0.015)), material=satin_steel, name=f"lid_frame_bolt_{i}")

    # Internal rotating cradle, supported by the yoke through a visible shaft and
    # bearing collars. It carries a padded watch pillow and a retaining strap.
    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.011, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="shaft",
    )
    cradle.visual(
        Cylinder(radius=0.064, length=0.176),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="pillow_core",
    )
    cradle.visual(Box((0.158, 0.042, 0.075)), origin=Origin(xyz=(0.0, -0.052, 0.000)), material=rubber, name="watch_pillow")
    cradle.visual(Box((0.170, 0.010, 0.012)), origin=Origin(xyz=(0.0, -0.060, 0.032)), material=safety_yellow, name="retaining_strap")
    cradle.visual(Box((0.170, 0.010, 0.012)), origin=Origin(xyz=(0.0, -0.060, -0.032)), material=safety_yellow, name="strap_return")
    for i, x in enumerate((-0.135, 0.135)):
        _x_cylinder(cradle, f"bearing_collar_{i}", 0.023, 0.014, (x, 0.0, 0.0), satin_steel)
        cradle.visual(Box((0.012, 0.040, 0.014)), origin=Origin(xyz=(x, -0.030, 0.0)), material=satin_steel, name=f"collar_key_{i}")
    cradle.visual(Box((0.015, 0.070, 0.018)), origin=Origin(xyz=(0.090, 0.048, 0.000)), material=lockout_red, name="home_stop_flag")

    # Articulated red safety lockout hasp with a real front-mounted pivot.
    lockout = model.part("lockout")
    lockout.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot_pin",
    )
    lockout.visual(Box((0.035, 0.012, 0.112)), origin=Origin(xyz=(0.0, -0.008, 0.056)), material=lockout_red, name="hasp_plate")
    lockout.visual(Box((0.086, 0.014, 0.022)), origin=Origin(xyz=(0.0, -0.008, 0.112)), material=lockout_red, name="padlock_eye")
    lockout.visual(Cylinder(radius=0.010, length=0.004), origin=Origin(xyz=(0.0, -0.016, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)), material=black_plate, name="lock_hole")

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.198, 0.207)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=1.35),
        motion_properties=MotionProperties(damping=0.8, friction=0.12),
    )
    model.articulation(
        "base_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.006, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.5),
        motion_properties=MotionProperties(damping=0.15, friction=0.03),
    )
    model.articulation(
        "base_to_lockout",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lockout,
        origin=Origin(xyz=(0.0, -0.194, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.15),
        motion_properties=MotionProperties(damping=0.20, friction=0.08),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lockout = object_model.get_part("lockout")
    lid_hinge = object_model.get_articulation("base_to_lid")
    cradle_spin = object_model.get_articulation("base_to_cradle")
    lockout_hinge = object_model.get_articulation("base_to_lockout")

    ctx.allow_overlap(
        base,
        cradle,
        elem_a="bearing_boss_0",
        elem_b="shaft",
        reason="The cradle shaft is intentionally captured inside the left bearing boss proxy.",
    )
    ctx.allow_overlap(
        base,
        cradle,
        elem_a="bearing_boss_1",
        elem_b="shaft",
        reason="The cradle shaft is intentionally captured inside the right bearing boss proxy.",
    )
    ctx.allow_overlap(
        base,
        lockout,
        elem_a="lockout_backplate",
        elem_b="pivot_pin",
        reason="The lockout pivot pin is intentionally seated through the reinforced backplate boss.",
    )

    ctx.expect_overlap(lid, base, axes="xy", elem_a="inspection_window", elem_b="floor_plate", min_overlap=0.22, name="closed lid covers the open tray")
    ctx.expect_gap(lid, base, axis="z", positive_elem="rear_bar", negative_elem="rear_wall", min_gap=0.004, max_gap=0.040, name="closed lid clears the tray rim")
    ctx.expect_overlap(cradle, base, axes="x", elem_a="shaft", elem_b="cradle_yoke", min_overlap=0.32, name="cradle shaft spans both yoke cheeks")
    ctx.expect_within(cradle, base, axes="xy", inner_elem="pillow_core", outer_elem="floor_plate", margin=0.0, name="rotor is centered inside guarded tray footprint")
    ctx.expect_overlap(lockout, base, axes="xz", elem_a="pivot_pin", elem_b="lockout_backplate", min_overlap=0.015, name="lockout pivot is mounted on reinforced backplate")
    ctx.expect_overlap(cradle, base, axes="yz", elem_a="shaft", elem_b="bearing_boss_0", min_overlap=0.018, name="shaft enters left bearing boss")
    ctx.expect_overlap(cradle, base, axes="yz", elem_a="shaft", elem_b="bearing_boss_1", min_overlap=0.018, name="shaft enters right bearing boss")

    with ctx.pose({lid_hinge: 1.10}):
        open_bar = ctx.part_element_world_aabb(lid, elem="front_bar")
        ctx.check(
            "lid hinge opens upward on reinforced rear axis",
            open_bar is not None and open_bar[0][2] > 0.42,
            details=f"front_bar_aabb={open_bar}",
        )

    with ctx.pose({cradle_spin: -math.pi / 2.0}):
        ctx.expect_overlap(cradle, base, axes="x", elem_a="shaft", elem_b="cradle_yoke", min_overlap=0.32, name="spun cradle remains captured by supports")
        strap_box = ctx.part_element_world_aabb(cradle, elem="retaining_strap")
        ctx.check(
            "cradle spin visibly rotates strap around shaft",
            strap_box is not None and strap_box[1][2] > 0.165,
            details=f"strap_aabb={strap_box}",
        )

    with ctx.pose({lockout_hinge: 0.95}):
        hasp_box = ctx.part_element_world_aabb(lockout, elem="hasp_plate")
        ctx.check(
            "lockout hasp swings outward for service access",
            hasp_box is not None and hasp_box[0][1] < -0.275,
            details=f"hasp_aabb={hasp_box}",
        )

    return ctx.report()


object_model = build_object_model()
