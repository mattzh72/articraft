from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_guillotine_cutter")

    cast_green = model.material("cast_green", rgba=(0.27, 0.40, 0.31, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.82, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.10, 1.0))

    bed = model.part("bed_frame")
    bed.visual(
        Box((0.62, 0.40, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=cast_green,
        name="bed_plate",
    )
    bed.visual(
        Box((0.56, 0.015, 0.020)),
        origin=Origin(xyz=(0.0, 0.185, 0.034)),
        material=steel,
        name="rear_fence",
    )
    bed.visual(
        Box((0.52, 0.015, 0.003)),
        origin=Origin(xyz=(0.0, 0.035, 0.0255)),
        material=dark_steel,
        name="cut_strip",
    )
    bed.visual(
        Box((0.53, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, -0.170, 0.028)),
        material=dark_steel,
        name="front_lip",
    )
    bed.visual(
        Box((0.045, 0.26, 0.048)),
        origin=Origin(xyz=(0.2875, 0.060, 0.048)),
        material=cast_green,
        name="side_spine",
    )
    bed.visual(
        Box((0.045, 0.090, 0.120)),
        origin=Origin(xyz=(0.2875, 0.000, 0.084)),
        material=cast_green,
        name="clamp_support",
    )
    bed.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.247, -0.020, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="clamp_pivot_rear_washer",
    )
    bed.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.247, 0.020, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="clamp_pivot_front_washer",
    )
    bed.visual(
        Box((0.045, 0.120, 0.170)),
        origin=Origin(xyz=(0.2875, 0.145, 0.109)),
        material=cast_green,
        name="blade_support",
    )
    bed.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.247, 0.122, 0.108), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="blade_pivot_rear_washer",
    )
    bed.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.247, 0.168, 0.108), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="blade_pivot_front_washer",
    )
    bed.inertial = Inertial.from_geometry(
        Box((0.62, 0.40, 0.20)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    clamp = model.part("clamp_lever")
    clamp.visual(
        Cylinder(radius=0.014, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_sleeve",
    )
    clamp.visual(
        Box((0.080, 0.020, 0.040)),
        origin=Origin(xyz=(-0.040, -0.028, -0.020)),
        material=dark_steel,
        name="pivot_clip",
    )
    clamp.visual(
        Box((0.180, 0.026, 0.070)),
        origin=Origin(xyz=(-0.090, -0.031, -0.028)),
        material=cast_green,
        name="side_arm",
    )
    clamp.visual(
        Box((0.490, 0.018, 0.016)),
        origin=Origin(xyz=(-0.245, -0.010, -0.058)),
        material=steel,
        name="pressure_bar",
    )
    clamp.visual(
        Box((0.038, 0.024, 0.150)),
        origin=Origin(xyz=(-0.180, -0.031, 0.082)),
        material=cast_green,
        name="lever_post",
    )
    clamp.visual(
        Cylinder(radius=0.013, length=0.100),
        origin=Origin(xyz=(-0.190, -0.031, 0.156), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="lever_grip",
    )
    clamp.inertial = Inertial.from_geometry(
        Box((0.50, 0.05, 0.22)),
        mass=3.0,
        origin=Origin(xyz=(-0.24, 0.0, 0.05)),
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=0.016, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_sleeve",
    )
    blade_arm.visual(
        Box((0.060, 0.086, 0.042)),
        origin=Origin(xyz=(-0.030, -0.015, -0.001)),
        material=dark_steel,
        name="hinge_web",
    )
    blade_arm.visual(
        Box((0.140, 0.080, 0.050)),
        origin=Origin(xyz=(-0.110, -0.070, -0.010)),
        material=dark_steel,
        name="pivot_gusset",
    )
    blade_arm.visual(
        Box((0.460, 0.040, 0.034)),
        origin=Origin(xyz=(-0.320, -0.112, -0.010)),
        material=cast_green,
        name="arm_beam",
    )
    blade_arm.visual(
        Box((0.460, 0.022, 0.032)),
        origin=Origin(xyz=(-0.320, -0.095, -0.039)),
        material=dark_steel,
        name="blade_mount",
    )
    blade_arm.visual(
        Box((0.460, 0.010, 0.050)),
        origin=Origin(xyz=(-0.320, -0.104, -0.059)),
        material=steel,
        name="blade_edge",
    )
    blade_arm.visual(
        Box((0.048, 0.024, 0.210)),
        origin=Origin(xyz=(-0.460, -0.112, 0.086)),
        material=cast_green,
        name="handle_post",
    )
    blade_arm.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(-0.460, -0.112, 0.190), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="handle_grip",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.56, 0.18, 0.28)),
        mass=4.5,
        origin=Origin(xyz=(-0.24, -0.11, 0.04)),
    )

    model.articulation(
        "bed_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=blade_arm,
        origin=Origin(xyz=(0.247, 0.145, 0.108)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "bed_to_clamp",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=clamp,
        origin=Origin(xyz=(0.247, 0.000, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed_frame")
    blade_arm = object_model.get_part("blade_arm")
    clamp = object_model.get_part("clamp_lever")
    blade_joint = object_model.get_articulation("bed_to_blade_arm")
    clamp_joint = object_model.get_articulation("bed_to_clamp")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "blade_arm_axis_is_side_pivot",
        tuple(abs(v) for v in blade_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected blade arm axis parallel to Y, got {blade_joint.axis}.",
    )
    ctx.check(
        "clamp_axis_is_side_pivot",
        tuple(abs(v) for v in clamp_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected clamp lever axis parallel to Y, got {clamp_joint.axis}.",
    )

    ctx.expect_contact(
        blade_arm,
        bed,
        elem_a="pivot_sleeve",
        elem_b="blade_pivot_front_washer",
        name="blade_pivot_stays_seated",
    )
    ctx.expect_contact(
        clamp,
        bed,
        elem_a="pivot_sleeve",
        elem_b="clamp_pivot_front_washer",
        name="clamp_pivot_stays_clipped",
    )
    ctx.expect_gap(
        clamp,
        bed,
        axis="z",
        max_gap=0.0015,
        max_penetration=1e-5,
        positive_elem="pressure_bar",
        negative_elem="bed_plate",
        name="pressure_bar_rests_on_bed",
    )
    ctx.expect_gap(
        blade_arm,
        bed,
        axis="z",
        max_gap=0.0015,
        max_penetration=1e-5,
        positive_elem="blade_edge",
        negative_elem="bed_plate",
        name="blade_edge_reaches_cutting_plane",
    )

    pressure_bar_aabb = ctx.part_element_world_aabb(clamp, elem="pressure_bar")
    cut_strip_aabb = ctx.part_element_world_aabb(bed, elem="cut_strip")
    if pressure_bar_aabb is None or cut_strip_aabb is None:
        ctx.fail("pressure_bar_ahead_of_cut_line", "Could not resolve named visuals for cut alignment.")
    else:
        ahead = cut_strip_aabb[0][1] - pressure_bar_aabb[1][1]
        ctx.check(
            "pressure_bar_ahead_of_cut_line",
            0.010 <= ahead <= 0.040,
            f"Expected pressure bar to sit 10–40 mm ahead of cut line, got {ahead:.4f} m.",
        )

    def elem_center_z(part_obj, elem_name: str) -> float:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        assert aabb is not None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def elem_max_z(part_obj, elem_name: str) -> float:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        assert aabb is not None
        return aabb[1][2]

    blade_handle_rest = elem_center_z(blade_arm, "handle_grip")
    clamp_grip_rest = elem_center_z(clamp, "lever_grip")
    blade_edge_rest_max = elem_max_z(blade_arm, "blade_edge")
    pressure_bar_rest_max = elem_max_z(clamp, "pressure_bar")

    with ctx.pose({blade_joint: math.radians(68.0)}):
        ctx.expect_contact(
            blade_arm,
            bed,
            elem_a="pivot_sleeve",
            elem_b="blade_pivot_front_washer",
            name="blade_pivot_contacts_when_open",
        )
        blade_handle_open = elem_center_z(blade_arm, "handle_grip")
        blade_edge_open_max = elem_max_z(blade_arm, "blade_edge")
        ctx.check(
            "blade_edge_lifts_clear_when_open",
            blade_edge_open_max > blade_edge_rest_max + 0.22,
            f"Expected distal blade edge height to rise by at least 0.22 m, got {blade_edge_open_max - blade_edge_rest_max:.4f} m.",
        )
        ctx.check(
            "blade_handle_rises",
            blade_handle_open > blade_handle_rest + 0.18,
            f"Expected blade handle to rise by at least 0.18 m, got {blade_handle_open - blade_handle_rest:.4f} m.",
        )

    with ctx.pose({clamp_joint: math.radians(45.0)}):
        ctx.expect_contact(
            clamp,
            bed,
            elem_a="pivot_sleeve",
            elem_b="clamp_pivot_front_washer",
            name="clamp_pivot_contacts_when_open",
        )
        clamp_grip_open = elem_center_z(clamp, "lever_grip")
        pressure_bar_open_max = elem_max_z(clamp, "pressure_bar")
        ctx.check(
            "pressure_bar_lifts_clear_when_open",
            pressure_bar_open_max > pressure_bar_rest_max + 0.18,
            f"Expected raised pressure bar height to increase by at least 0.18 m, got {pressure_bar_open_max - pressure_bar_rest_max:.4f} m.",
        )
        ctx.check(
            "clamp_lever_swings_up",
            clamp_grip_open > clamp_grip_rest + 0.08,
            f"Expected clamp grip to rise by at least 0.08 m, got {clamp_grip_open - clamp_grip_rest:.4f} m.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
