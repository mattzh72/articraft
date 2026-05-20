from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

DECK_LENGTH = 1.80
DECK_WIDTH = 0.76
DECK_THICKNESS = 0.08
WHEEL_RADIUS = 0.18
WHEEL_WIDTH = 0.07
DECK_CENTER_Z = WHEEL_RADIUS + DECK_THICKNESS / 2.0
AXLE_STUB_LENGTH = 0.07

WHEEL_LAYOUT = (
    ("front_left", 0.56, 0.46),
    ("front_right", 0.56, -0.46),
    ("rear_left", -0.50, 0.46),
    ("rear_right", -0.50, -0.46),
)

HINGE_X = -0.66
HINGE_Z = DECK_CENTER_Z + DECK_THICKNESS / 2.0 + 0.05

MAST_SIDE = 0.18
MAST_TOP_Z = 2.24
JIB_LENGTH = 1.06


def _stabilize_cwd() -> None:
    try:
        os.getcwd()
        return
    except FileNotFoundError:
        pass
    spec = globals().get("__spec__")
    candidates = (
        globals().get("__file__"),
        getattr(spec, "origin", None),
        _stabilize_cwd.__code__.co_filename,
        "/",
    )
    for candidate in candidates:
        if isinstance(candidate, str) and candidate:
            target = candidate if os.path.isdir(candidate) else os.path.dirname(candidate)
            if target and os.path.isdir(target):
                os.chdir(target)
                return


_stabilize_cwd()


def _add_box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_tube_between(part, name, p0, p1, radius, material) -> None:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    center = (
        (p0[0] + p1[0]) / 2.0,
        (p0[1] + p1[1]) / 2.0,
        (p0[2] + p1[2]) / 2.0,
    )
    _add_cylinder(
        part,
        name=name,
        radius=radius,
        length=length,
        xyz=center,
        material=material,
        rpy=(0.0, pitch, yaw),
    )


def _build_wheel_part(model, name, tire_material, steel_material):
    wheel = model.part(name)
    _add_cylinder(
        wheel,
        "tire",
        radius=WHEEL_RADIUS,
        length=WHEEL_WIDTH,
        xyz=(0.0, 0.0, 0.0),
        material=tire_material,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _add_cylinder(
        wheel,
        "hub",
        radius=0.065,
        length=WHEEL_WIDTH + 0.02,
        xyz=(0.0, 0.0, 0.0),
        material=steel_material,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _add_cylinder(
        wheel,
        "rim_outer",
        radius=0.105,
        length=0.012,
        xyz=(0.0, 0.029, 0.0),
        material=steel_material,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _add_cylinder(
        wheel,
        "rim_inner",
        radius=0.105,
        length=0.012,
        xyz=(0.0, -0.029, 0.0),
        material=steel_material,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    return wheel


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) / 2.0 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_erecting_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.93, 0.76, 0.16, 1.0))
    chassis_grey = model.material("chassis_grey", rgba=(0.25, 0.27, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    tire = model.material("tire", rgba=(0.08, 0.08, 0.09, 1.0))
    cable = model.material("cable", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")
    _add_box(
        base,
        "deck",
        (DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS),
        (0.0, 0.0, DECK_CENTER_Z),
        chassis_grey,
    )
    _add_box(base, "center_spine", (1.48, 0.20, 0.12), (0.0, 0.0, 0.18), chassis_grey)
    _add_box(base, "drawbar", (0.42, 0.09, 0.10), (1.06, 0.0, 0.17), chassis_grey)
    _add_box(base, "rear_ballast_left", (0.20, 0.16, 0.06), (-0.80, 0.20, 0.29), chassis_grey)
    _add_box(base, "rear_ballast_right", (0.20, 0.16, 0.06), (-0.80, -0.20, 0.29), chassis_grey)
    _add_box(base, "hinge_left_cheek", (0.12, 0.05, 0.10), (HINGE_X, 0.135, HINGE_Z), steel)
    _add_box(base, "hinge_right_cheek", (0.12, 0.05, 0.10), (HINGE_X, -0.135, HINGE_Z), steel)
    _add_box(base, "hinge_saddle", (0.14, 0.22, 0.02), (HINGE_X, 0.0, HINGE_Z - 0.044), steel)

    for name, x_pos, y_pos in WHEEL_LAYOUT:
        y_sign = 1.0 if y_pos > 0.0 else -1.0
        _add_box(
            base,
            f"{name}_suspension_box",
            (0.16, 0.08, 0.10),
            (x_pos, y_sign * 0.34, 0.18),
            chassis_grey,
        )
        _add_cylinder(
            base,
            f"{name}_axle_stub",
            radius=0.03,
            length=AXLE_STUB_LENGTH,
            xyz=(x_pos, y_sign * (DECK_WIDTH / 2.0), WHEEL_RADIUS),
            material=steel,
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )

    mast = model.part("mast")
    _add_cylinder(
        mast,
        "hinge_barrel",
        radius=0.025,
        length=0.22,
        xyz=(0.0, 0.0, 0.0),
        material=steel,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    rail_radius = 0.015
    brace_radius = 0.007
    corner = MAST_SIDE / 2.0
    mast_rail_bottom = 0.10
    mast_rail_top = 2.08
    mast_levels = (0.28, 0.68, 1.08, 1.48, 1.88)
    barrel_pick_y = 0.075
    barrel_pick_z = 0.018
    mast_knee_z = 0.18
    corners = (
        (-corner, -corner),
        (-corner, corner),
        (corner, -corner),
        (corner, corner),
    )
    _add_tube_between(
        mast,
        "heel_brace_left_front",
        (0.0, barrel_pick_y, barrel_pick_z),
        (-corner, corner, mast_knee_z),
        0.010,
        crane_yellow,
    )
    _add_tube_between(
        mast,
        "heel_brace_left_rear",
        (0.0, barrel_pick_y, barrel_pick_z),
        (corner, corner, mast_knee_z),
        0.010,
        crane_yellow,
    )
    _add_tube_between(
        mast,
        "heel_brace_right_front",
        (0.0, -barrel_pick_y, barrel_pick_z),
        (-corner, -corner, mast_knee_z),
        0.010,
        crane_yellow,
    )
    _add_tube_between(
        mast,
        "heel_brace_right_rear",
        (0.0, -barrel_pick_y, barrel_pick_z),
        (corner, -corner, mast_knee_z),
        0.010,
        crane_yellow,
    )
    for idx, (x_pos, y_pos) in enumerate(corners):
        _add_tube_between(
            mast,
            f"rail_{idx}",
            (x_pos, y_pos, mast_rail_bottom),
            (x_pos, y_pos, mast_rail_top),
            rail_radius,
            crane_yellow,
        )

    for level_index, z_pos in enumerate(mast_levels):
        _add_tube_between(
            mast,
            f"x_tie_left_{level_index}",
            (-corner, -corner, z_pos),
            (corner, -corner, z_pos),
            brace_radius,
            crane_yellow,
        )
        _add_tube_between(
            mast,
            f"x_tie_right_{level_index}",
            (-corner, corner, z_pos),
            (corner, corner, z_pos),
            brace_radius,
            crane_yellow,
        )
        _add_tube_between(
            mast,
            f"y_tie_front_{level_index}",
            (-corner, -corner, z_pos),
            (-corner, corner, z_pos),
            brace_radius,
            crane_yellow,
        )
        _add_tube_between(
            mast,
            f"y_tie_rear_{level_index}",
            (corner, -corner, z_pos),
            (corner, corner, z_pos),
            brace_radius,
            crane_yellow,
        )

    mast_bays = (mast_rail_bottom,) + mast_levels + (mast_rail_top,)
    for bay_index, (z0, z1) in enumerate(zip(mast_bays[:-1], mast_bays[1:])):
        left_front = (-corner, -corner, z0)
        left_back = (-corner, corner, z0)
        right_front = (corner, -corner, z0)
        right_back = (corner, corner, z0)
        upper_left_front = (-corner, -corner, z1)
        upper_left_back = (-corner, corner, z1)
        upper_right_front = (corner, -corner, z1)
        upper_right_back = (corner, corner, z1)
        if bay_index % 2 == 0:
            _add_tube_between(mast, f"diag_xneg_{bay_index}", left_front, upper_left_back, brace_radius, crane_yellow)
            _add_tube_between(mast, f"diag_xpos_{bay_index}", right_back, upper_right_front, brace_radius, crane_yellow)
            _add_tube_between(mast, f"diag_yneg_{bay_index}", left_front, upper_right_front, brace_radius, crane_yellow)
            _add_tube_between(mast, f"diag_ypos_{bay_index}", left_back, upper_right_back, brace_radius, crane_yellow)
        else:
            _add_tube_between(mast, f"diag_xneg_{bay_index}", left_back, upper_left_front, brace_radius, crane_yellow)
            _add_tube_between(mast, f"diag_xpos_{bay_index}", right_front, upper_right_back, brace_radius, crane_yellow)
            _add_tube_between(mast, f"diag_yneg_{bay_index}", right_front, upper_left_front, brace_radius, crane_yellow)
            _add_tube_between(mast, f"diag_ypos_{bay_index}", right_back, upper_left_back, brace_radius, crane_yellow)

    _add_box(mast, "mast_head_block", (0.28, 0.28, 0.12), (0.0, 0.0, 2.135), steel)
    _add_cylinder(mast, "slew_ring", radius=0.14, length=0.06, xyz=(0.0, 0.0, 2.21), material=steel)
    _add_box(mast, "service_ladder", (0.04, 0.02, 1.40), (0.12, corner, 1.10), steel)

    jib = model.part("jib")
    _add_cylinder(jib, "turntable_plate", radius=0.12, length=0.03, xyz=(0.0, 0.0, 0.015), material=steel)
    _add_cylinder(jib, "machinery_post", radius=0.05, length=0.12, xyz=(0.0, 0.0, 0.09), material=steel)
    _add_box(jib, "machinery_house", (0.26, 0.24, 0.16), (-0.08, 0.0, 0.13), chassis_grey)

    jib_lower_y = 0.08
    jib_lower_z = 0.06
    jib_top_z = 0.22
    _add_tube_between(jib, "root_brace_left", (0.02, 0.0, 0.12), (0.12, jib_lower_y, jib_lower_z), 0.010, crane_yellow)
    _add_tube_between(jib, "root_brace_right", (0.02, 0.0, 0.12), (0.12, -jib_lower_y, jib_lower_z), 0.010, crane_yellow)
    _add_tube_between(jib, "main_lower_left", (0.10, jib_lower_y, jib_lower_z), (JIB_LENGTH, jib_lower_y, jib_lower_z), 0.015, crane_yellow)
    _add_tube_between(jib, "main_lower_right", (0.10, -jib_lower_y, jib_lower_z), (JIB_LENGTH, -jib_lower_y, jib_lower_z), 0.015, crane_yellow)
    _add_tube_between(jib, "main_top_chord", (0.22, 0.0, jib_top_z), (0.98, 0.0, jib_top_z), 0.013, crane_yellow)
    for idx, x_pos in enumerate((0.24, 0.52, 0.80, 1.02)):
        _add_tube_between(jib, f"main_cross_{idx}", (x_pos, -jib_lower_y, jib_lower_z), (x_pos, jib_lower_y, jib_lower_z), 0.006, crane_yellow)
        if x_pos < 0.99:
            _add_tube_between(jib, f"main_post_{idx}", (x_pos, 0.0, jib_top_z), (x_pos, 0.0, jib_lower_z), 0.006, crane_yellow)
    for idx, (x0, x1) in enumerate(((0.22, 0.52), (0.52, 0.80), (0.80, 1.02))):
        _add_tube_between(jib, f"diag_left_{idx}", (x0, jib_lower_y, jib_lower_z), (x1, 0.0, jib_top_z), 0.006, crane_yellow)
        _add_tube_between(jib, f"diag_right_{idx}", (x0, -jib_lower_y, jib_lower_z), (x1, 0.0, jib_top_z), 0.006, crane_yellow)
        _add_tube_between(jib, f"underbrace_left_{idx}", (x0, 0.0, jib_top_z), (x1, jib_lower_y, jib_lower_z), 0.006, crane_yellow)
        _add_tube_between(jib, f"underbrace_right_{idx}", (x0, 0.0, jib_top_z), (x1, -jib_lower_y, jib_lower_z), 0.006, crane_yellow)

    _add_tube_between(jib, "counter_lower_left", (-0.36, 0.07, 0.05), (-0.02, 0.07, 0.05), 0.014, crane_yellow)
    _add_tube_between(jib, "counter_lower_right", (-0.36, -0.07, 0.05), (-0.02, -0.07, 0.05), 0.014, crane_yellow)
    _add_tube_between(jib, "counter_top", (-0.22, 0.0, 0.18), (0.06, 0.0, 0.08), 0.012, crane_yellow)
    _add_tube_between(jib, "counter_brace_left", (-0.22, 0.0, 0.18), (-0.34, 0.07, 0.05), 0.006, crane_yellow)
    _add_tube_between(jib, "counter_brace_right", (-0.22, 0.0, 0.18), (-0.34, -0.07, 0.05), 0.006, crane_yellow)
    _add_box(jib, "counterweight", (0.18, 0.22, 0.16), (-0.33, 0.0, 0.13), steel)

    _add_cylinder(
        jib,
        "tip_sheave",
        radius=0.04,
        length=0.05,
        xyz=(1.08, 0.0, 0.10),
        material=steel,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _add_tube_between(jib, "tip_hanger_left", (1.04, jib_lower_y, jib_lower_z), (1.08, 0.0, 0.10), 0.008, crane_yellow)
    _add_tube_between(jib, "tip_hanger_right", (1.04, -jib_lower_y, jib_lower_z), (1.08, 0.0, 0.10), 0.008, crane_yellow)
    _add_tube_between(jib, "tip_backstay", (0.98, 0.0, jib_top_z), (1.08, 0.0, 0.10), 0.008, crane_yellow)
    _add_cylinder(jib, "hook_cable", radius=0.004, length=0.38, xyz=(1.08, 0.0, -0.10), material=cable)
    _add_box(jib, "hook_block", (0.08, 0.07, 0.10), (1.08, 0.0, -0.33), steel)

    wheels = {}
    for name, _, _ in WHEEL_LAYOUT:
        wheels[name] = _build_wheel_part(model, f"{name}_wheel", tire, steel)

    model.articulation(
        "base_to_mast",
        ArticulationType.REVOLUTE,
        parent=base,
        child=mast,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=-1.15, upper=0.15),
    )
    model.articulation(
        "mast_to_jib",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=jib,
        origin=Origin(xyz=(0.0, 0.0, MAST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.9, lower=-3.0, upper=3.0),
    )
    for name, x_pos, y_pos in WHEEL_LAYOUT:
        model.articulation(
            f"base_to_{name}_wheel",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheels[name],
            origin=Origin(xyz=(x_pos, y_pos, WHEEL_RADIUS)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    jib = object_model.get_part("jib")
    mast_fold = object_model.get_articulation("base_to_mast")
    slew = object_model.get_articulation("mast_to_jib")

    wheel_parts = {name: object_model.get_part(f"{name}_wheel") for name, _, _ in WHEEL_LAYOUT}
    wheel_joints = {
        name: object_model.get_articulation(f"base_to_{name}_wheel")
        for name, _, _ in WHEEL_LAYOUT
    }

    deck = base.get_visual("deck")
    hinge_left_cheek = base.get_visual("hinge_left_cheek")
    hinge_right_cheek = base.get_visual("hinge_right_cheek")
    hinge_barrel = mast.get_visual("hinge_barrel")
    slew_ring = mast.get_visual("slew_ring")
    turntable_plate = jib.get_visual("turntable_plate")
    tip_sheave = jib.get_visual("tip_sheave")
    front_left_axle = base.get_visual("front_left_axle_stub")
    front_right_axle = base.get_visual("front_right_axle_stub")
    rear_left_axle = base.get_visual("rear_left_axle_stub")
    rear_right_axle = base.get_visual("rear_right_axle_stub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=6)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    ctx.expect_contact(mast, base, elem_a=hinge_barrel, elem_b=hinge_left_cheek)
    ctx.expect_contact(mast, base, elem_a=hinge_barrel, elem_b=hinge_right_cheek)
    ctx.expect_gap(
        mast,
        base,
        axis="z",
        min_gap=1.80,
        positive_elem=slew_ring,
        negative_elem=deck,
        name="mast_keeps_slewing_ring_high_above_wheeled_base",
    )
    ctx.expect_contact(jib, mast, elem_a=turntable_plate, elem_b=slew_ring)
    ctx.expect_within(jib, mast, axes="xy", inner_elem=turntable_plate, outer_elem=slew_ring)
    ctx.expect_gap(
        jib,
        mast,
        axis="x",
        min_gap=0.88,
        positive_elem=tip_sheave,
        negative_elem=slew_ring,
        name="short_jib_projects_forward_from_the_slewing_ring",
    )

    wheel_contacts = {
        "front_left": front_left_axle,
        "front_right": front_right_axle,
        "rear_left": rear_left_axle,
        "rear_right": rear_right_axle,
    }
    for name, _, _ in WHEEL_LAYOUT:
        wheel = wheel_parts[name]
        wheel_hub = wheel.get_visual("hub")
        ctx.expect_contact(
            wheel,
            base,
            elem_a=wheel_hub,
            elem_b=wheel_contacts[name],
            name=f"{name}_wheel_hub_contacts_axle_stub",
        )
        ctx.expect_origin_distance(
            wheel,
            base,
            axes="y",
            min_dist=0.45,
            max_dist=0.47,
            name=f"{name}_wheel_sits_outboard_of_the_base",
        )
        ctx.check(
            f"{name}_wheel_spin_axis_is_axial",
            wheel_joints[name].axis == (0.0, 1.0, 0.0),
            details=f"axis={wheel_joints[name].axis}",
        )

    ctx.expect_origin_distance(
        wheel_parts["front_left"],
        wheel_parts["rear_left"],
        axes="x",
        min_dist=1.05,
        max_dist=1.07,
        name="left_wheels_span_the_trailer_wheelbase",
    )

    slew_ring_rest_aabb = ctx.part_element_world_aabb(mast, elem=slew_ring)
    tip_rest_aabb = ctx.part_element_world_aabb(jib, elem=tip_sheave)
    if slew_ring_rest_aabb is None:
        ctx.fail("slew_ring_rest_aabb_available", "Could not evaluate mast slew ring world AABB.")
    if tip_rest_aabb is None:
        ctx.fail("tip_sheave_rest_aabb_available", "Could not evaluate jib tip sheave world AABB.")

    with ctx.pose({mast_fold: mast_fold.motion_limits.lower}):
        ctx.expect_contact(mast, base, elem_a=hinge_barrel, elem_b=hinge_left_cheek)
        ctx.expect_contact(mast, base, elem_a=hinge_barrel, elem_b=hinge_right_cheek)
        ctx.expect_gap(
            mast,
            base,
            axis="z",
            min_gap=0.75,
            positive_elem=slew_ring,
            negative_elem=deck,
            name="folded_mast_still_clears_the_trailer_body",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="mast_fold_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="mast_fold_lower_no_floating")
        if slew_ring_rest_aabb is not None:
            slew_ring_folded_aabb = ctx.part_element_world_aabb(mast, elem=slew_ring)
            if slew_ring_folded_aabb is None:
                ctx.fail("slew_ring_folded_aabb_available", "Could not evaluate folded slew ring AABB.")
            else:
                slew_ring_rest_center = _aabb_center(slew_ring_rest_aabb)
                slew_ring_folded_center = _aabb_center(slew_ring_folded_aabb)
                ctx.check(
                    "mast_fold_sweeps_mast_head_rearward_and_down",
                    slew_ring_folded_center[0] < slew_ring_rest_center[0] - 1.0
                    and slew_ring_folded_center[2] < slew_ring_rest_center[2] - 1.0,
                    details=(
                        f"rest_center={slew_ring_rest_center}, "
                        f"folded_center={slew_ring_folded_center}"
                    ),
                )

    with ctx.pose({mast_fold: mast_fold.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="mast_fold_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="mast_fold_upper_no_floating")
        ctx.expect_contact(mast, base, elem_a=hinge_barrel, elem_b=hinge_left_cheek)
        ctx.expect_contact(mast, base, elem_a=hinge_barrel, elem_b=hinge_right_cheek)

    with ctx.pose({slew: 1.4}):
        ctx.expect_contact(jib, mast, elem_a=turntable_plate, elem_b=slew_ring)
        ctx.expect_within(jib, mast, axes="xy", inner_elem=turntable_plate, outer_elem=slew_ring)
        ctx.fail_if_parts_overlap_in_current_pose(name="slew_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="slew_pose_no_floating")
        if tip_rest_aabb is not None:
            tip_swung_aabb = ctx.part_element_world_aabb(jib, elem=tip_sheave)
            if tip_swung_aabb is None:
                ctx.fail("tip_sheave_slewed_aabb_available", "Could not evaluate slewed tip AABB.")
            else:
                tip_rest_center = _aabb_center(tip_rest_aabb)
                tip_swung_center = _aabb_center(tip_swung_aabb)
                ctx.check(
                    "jib_slew_moves_tip_sideways",
                    abs(tip_swung_center[1] - tip_rest_center[1]) > 0.95,
                    details=f"rest_center={tip_rest_center}, swung_center={tip_swung_center}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
