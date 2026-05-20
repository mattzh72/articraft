from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import ArticulatedObject, ArticulationType, Box, Cylinder, Inertial, MotionLimits, Origin, TestContext, TestReport


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def _aabb_size(aabb):
    if aabb is None:
        return None
    return tuple(aabb[1][i] - aabb[0][i] for i in range(3))


def _axis_matches(axis, expected) -> bool:
    return tuple(round(float(v), 6) for v in axis) == tuple(round(float(v), 6) for v in expected)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compound_miter_saw")

    cast_metal = model.material("cast_metal", rgba=(0.45, 0.48, 0.52, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    guard_red = model.material("guard_red", rgba=(0.77, 0.17, 0.12, 1.0))
    blade_silver = model.material("blade_silver", rgba=(0.86, 0.87, 0.88, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    amber = model.material("amber", rgba=(0.82, 0.56, 0.18, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((0.62, 0.42, 0.30)),
        mass=21.0,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )
    base.visual(Box((0.62, 0.42, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=cast_metal, name="base_foot")
    base.visual(Box((0.44, 0.28, 0.03)), origin=Origin(xyz=(0.0, 0.04, 0.065)), material=cast_metal, name="base_deck")
    base.visual(Box((0.18, 0.08, 0.20)), origin=Origin(xyz=(0.0, -0.16, 0.15)), material=dark_metal, name="pivot_pedestal")
    base.visual(Box((0.06, 0.06, 0.30)), origin=Origin(xyz=(-0.105, -0.16, 0.23)), material=dark_metal, name="left_pivot_ear")
    base.visual(Box((0.06, 0.06, 0.30)), origin=Origin(xyz=(0.105, -0.16, 0.23)), material=dark_metal, name="right_pivot_ear")
    base.visual(Box((0.08, 0.05, 0.03)), origin=Origin(xyz=(-0.23, 0.16, 0.015)), material=rubber_black, name="left_front_foot")
    base.visual(Box((0.08, 0.05, 0.03)), origin=Origin(xyz=(0.23, 0.16, 0.015)), material=rubber_black, name="right_front_foot")
    base.visual(Box((0.08, 0.05, 0.03)), origin=Origin(xyz=(-0.23, -0.15, 0.015)), material=rubber_black, name="left_rear_foot")
    base.visual(Box((0.08, 0.05, 0.03)), origin=Origin(xyz=(0.23, -0.15, 0.015)), material=rubber_black, name="right_rear_foot")

    table = model.part("miter_table")
    table.inertial = Inertial.from_geometry(
        Cylinder(radius=0.155, length=0.024),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )
    table.visual(Cylinder(radius=0.155, length=0.024), origin=Origin(xyz=(0.0, 0.0, 0.012)), material=aluminum, name="table_disc")
    table.visual(Box((0.18, 0.045, 0.01)), origin=Origin(xyz=(0.0, -0.05, 0.022)), material=dark_metal, name="table_kerf_plate")
    table.visual(Box((0.035, 0.09, 0.02)), origin=Origin(xyz=(0.0, 0.13, 0.012)), material=cast_metal, name="miter_handle")
    table.visual(Box((0.05, 0.012, 0.004)), origin=Origin(xyz=(0.0, 0.155, 0.026)), material=amber, name="angle_pointer")

    fence = model.part("fence")
    fence.inertial = Inertial.from_geometry(
        Box((0.52, 0.06, 0.11)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )
    fence.visual(Box((0.09, 0.06, 0.03)), origin=Origin(xyz=(-0.22, 0.0, 0.015)), material=cast_metal, name="left_riser")
    fence.visual(Box((0.09, 0.06, 0.03)), origin=Origin(xyz=(0.22, 0.0, 0.015)), material=cast_metal, name="right_riser")
    fence.visual(Box((0.18, 0.026, 0.075)), origin=Origin(xyz=(-0.20, 0.0, 0.0675)), material=aluminum, name="left_fence_wing")
    fence.visual(Box((0.18, 0.026, 0.075)), origin=Origin(xyz=(0.20, 0.0, 0.0675)), material=aluminum, name="right_fence_wing")
    fence.visual(Box((0.40, 0.016, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.092)), material=dark_metal, name="fence_bridge")

    arm = model.part("saw_arm")
    arm.inertial = Inertial.from_geometry(
        Box((0.22, 0.26, 0.16)),
        mass=5.4,
        origin=Origin(xyz=(0.0, 0.12, 0.08)),
    )
    arm.visual(
        Cylinder(radius=0.028, length=0.15),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="pivot_hub",
    )
    arm.visual(Box((0.18, 0.06, 0.10)), origin=Origin(xyz=(0.0, 0.04, 0.05)), material=cast_metal, name="rear_yoke")
    arm.visual(Box((0.028, 0.15, 0.03)), origin=Origin(xyz=(-0.07, 0.11, 0.09)), material=dark_metal, name="left_link")
    arm.visual(Box((0.028, 0.15, 0.03)), origin=Origin(xyz=(0.07, 0.11, 0.09)), material=dark_metal, name="right_link")
    arm.visual(Box((0.18, 0.05, 0.04)), origin=Origin(xyz=(0.0, 0.21, 0.12)), material=cast_metal, name="front_carriage")

    tilt_bracket = model.part("tilt_bracket")
    tilt_bracket.inertial = Inertial.from_geometry(
        Box((0.16, 0.05, 0.14)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
    )
    tilt_bracket.visual(Box((0.16, 0.05, 0.04)), origin=Origin(xyz=(0.0, 0.0, -0.02)), material=dark_metal, name="upper_bridge")
    tilt_bracket.visual(Box((0.012, 0.04, 0.14)), origin=Origin(xyz=(-0.066, 0.0, -0.11)), material=dark_metal, name="left_cheek")
    tilt_bracket.visual(Box((0.012, 0.04, 0.14)), origin=Origin(xyz=(0.066, 0.0, -0.11)), material=dark_metal, name="right_cheek")

    blade_head = model.part("blade_head")
    blade_head.inertial = Inertial.from_geometry(
        Box((0.18, 0.26, 0.24)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.01, -0.08)),
    )
    blade_head.visual(Box((0.075, 0.05, 0.04)), origin=Origin(xyz=(0.0, 0.0, -0.03)), material=cast_metal, name="bevel_saddle")
    blade_head.visual(Box((0.06, 0.08, 0.06)), origin=Origin(xyz=(0.0, -0.02, -0.08)), material=cast_metal, name="motor_bridge")
    blade_head.visual(Box((0.09, 0.07, 0.07)), origin=Origin(xyz=(-0.05, -0.055, -0.085)), material=cast_metal, name="drive_box")
    blade_head.visual(
        Cylinder(radius=0.038, length=0.12),
        origin=Origin(xyz=(0.0, -0.06, -0.09), rpy=(0.0, pi / 2.0, 0.0)),
        material=guard_red,
        name="motor_can",
    )
    blade_head.visual(Box((0.02, 0.10, 0.12)), origin=Origin(xyz=(-0.04, 0.07, -0.14)), material=guard_red, name="upper_guard")
    blade_head.visual(Box((0.048, 0.20, 0.05)), origin=Origin(xyz=(0.028, 0.11, -0.125)), material=cast_metal, name="spindle_arm")
    blade_head.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.052, 0.21, -0.145), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="spindle_mount",
    )
    blade_head.visual(Box((0.03, 0.04, 0.08)), origin=Origin(xyz=(0.03, 0.02, -0.06)), material=guard_red, name="rear_handle_post")
    blade_head.visual(Box((0.03, 0.10, 0.04)), origin=Origin(xyz=(0.03, 0.08, -0.01)), material=guard_red, name="top_handle_support")
    blade_head.visual(
        Cylinder(radius=0.015, length=0.10),
        origin=Origin(xyz=(0.03, 0.10, 0.01), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber_black,
        name="top_handle",
    )

    blade = model.part("blade")
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.008),
        mass=1.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    blade.visual(
        Cylinder(radius=0.125, length=0.005),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=blade_silver,
        name="blade_disc",
    )
    blade.visual(
        Cylinder(radius=0.02, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="arbor_flange",
    )
    blade.visual(
        Box((0.004, 0.012, 0.02)),
        origin=Origin(xyz=(0.0, 0.083, 0.089)),
        material=amber,
        name="blade_marker",
    )

    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.0, 0.045, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.6, lower=-0.82, upper=0.82),
    )
    model.articulation(
        "base_to_fence",
        ArticulationType.FIXED,
        parent=base,
        child=fence,
        origin=Origin(xyz=(0.0, -0.09, 0.08)),
    )
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, -0.16, 0.39)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=-0.65, upper=0.30),
    )
    model.articulation(
        "arm_to_tilt_bracket",
        ArticulationType.FIXED,
        parent=arm,
        child=tilt_bracket,
        origin=Origin(xyz=(0.0, 0.21, 0.10)),
    )
    model.articulation(
        "bracket_to_head",
        ArticulationType.REVOLUTE,
        parent=tilt_bracket,
        child=blade_head,
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.82, upper=0.82),
    )
    model.articulation(
        "head_to_blade",
        ArticulationType.CONTINUOUS,
        parent=blade_head,
        child=blade,
        origin=Origin(xyz=(0.065, 0.21, -0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=60.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    table = object_model.get_part("miter_table")
    fence = object_model.get_part("fence")
    arm = object_model.get_part("saw_arm")
    tilt_bracket = object_model.get_part("tilt_bracket")
    blade_head = object_model.get_part("blade_head")
    blade = object_model.get_part("blade")

    base_to_table = object_model.get_articulation("base_to_table")
    base_to_arm = object_model.get_articulation("base_to_arm")
    bracket_to_head = object_model.get_articulation("bracket_to_head")
    head_to_blade = object_model.get_articulation("head_to_blade")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("miter axis is vertical", _axis_matches(base_to_table.axis, (0.0, 0.0, 1.0)), str(base_to_table.axis))
    ctx.check("arm axis is transverse", _axis_matches(base_to_arm.axis, (1.0, 0.0, 0.0)), str(base_to_arm.axis))
    ctx.check("bevel axis is longitudinal", _axis_matches(bracket_to_head.axis, (0.0, 1.0, 0.0)), str(bracket_to_head.axis))
    ctx.check("blade spin axis matches spindle", _axis_matches(head_to_blade.axis, (1.0, 0.0, 0.0)), str(head_to_blade.axis))

    ctx.expect_contact(table, base, name="turntable sits on base")
    ctx.expect_contact(fence, base, name="fence mounts to base")
    ctx.expect_contact(arm, base, name="arm pivots from rear support")
    ctx.expect_contact(tilt_bracket, arm, name="tilt bracket mounts to arm")
    ctx.expect_contact(blade_head, tilt_bracket, name="head seats in tilt bracket")
    ctx.expect_contact(blade, blade_head, name="blade mounts to spindle")

    ctx.expect_overlap(table, base, axes="xy", min_overlap=0.18, name="table centered over base deck")
    ctx.expect_gap(blade, table, axis="z", min_gap=0.06, name="raised blade clears table")

    handle_rest = _aabb_center(ctx.part_element_world_aabb(table, elem="miter_handle"))
    if handle_rest is None:
        ctx.fail("miter handle measurable", "Named miter handle visual was not measurable.")
    else:
        with ctx.pose({base_to_table: 0.60}):
            handle_yawed = _aabb_center(ctx.part_element_world_aabb(table, elem="miter_handle"))
            ctx.check(
                "table yaw moves handle laterally",
                handle_yawed is not None and abs(handle_yawed[0] - handle_rest[0]) > 0.05,
                f"rest={handle_rest}, yawed={handle_yawed}",
            )

    blade_rest = _aabb_size(ctx.part_element_world_aabb(blade, elem="blade_disc"))
    if blade_rest is None:
        ctx.fail("blade disc measurable", "Blade disc visual was not measurable.")
    else:
        with ctx.pose({bracket_to_head: 0.55}):
            blade_beveled = _aabb_size(ctx.part_element_world_aabb(blade, elem="blade_disc"))
            ctx.check(
                "bevel tilts blade plane",
                blade_beveled is not None and blade_beveled[0] > blade_rest[0] + 0.10,
                f"rest={blade_rest}, beveled={blade_beveled}",
            )

    marker_rest = _aabb_center(ctx.part_element_world_aabb(blade, elem="blade_marker"))
    if marker_rest is None:
        ctx.fail("blade marker measurable", "Blade marker visual was not measurable.")
    else:
        with ctx.pose({head_to_blade: 1.10}):
            marker_spun = _aabb_center(ctx.part_element_world_aabb(blade, elem="blade_marker"))
            ctx.check(
                "blade spin moves marker around spindle",
                marker_spun is not None
                and (
                    abs(marker_spun[1] - marker_rest[1]) > 0.01
                    or abs(marker_spun[2] - marker_rest[2]) > 0.01
                ),
                f"rest={marker_rest}, spun={marker_spun}",
            )

    blade_origin_rest = ctx.part_world_position(blade)
    if blade_origin_rest is None:
        ctx.fail("blade world position measurable", "Blade part world position was not measurable.")
    else:
        with ctx.pose({base_to_arm: -0.20}):
            blade_origin_cut = ctx.part_world_position(blade)
            ctx.check(
                "arm pitch lowers blade head",
                blade_origin_cut is not None and blade_origin_cut[2] < blade_origin_rest[2] - 0.05,
                f"rest={blade_origin_rest}, lowered={blade_origin_cut}",
            )
            ctx.expect_gap(blade, table, axis="z", max_gap=0.05, max_penetration=0.0, name="lowered blade approaches table")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
