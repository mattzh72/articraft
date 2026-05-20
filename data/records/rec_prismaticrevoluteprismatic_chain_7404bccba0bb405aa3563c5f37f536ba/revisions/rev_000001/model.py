from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _mast_shapes() -> dict[str, cq.Workplane]:
    base = cq.Workplane("XY").box(0.10, 0.09, 0.02).translate((0.0, 0.0, 0.01))
    column = (
        cq.Workplane("XY")
        .box(0.056, 0.068, 0.36)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, 0.20))
    )
    left_rail = cq.Workplane("XY").box(0.014, 0.014, 0.26).translate((0.033, -0.021, 0.19))
    right_rail = cq.Workplane("XY").box(0.014, 0.014, 0.26).translate((0.033, 0.021, 0.19))
    top_cap = cq.Workplane("XY").box(0.068, 0.080, 0.014).translate((0.0, 0.0, 0.373))
    return {
        "base_foot": base,
        "column": column,
        "left_rail": left_rail,
        "right_rail": right_rail,
        "top_cap": top_cap,
    }


def _carriage_shapes() -> dict[str, cq.Workplane]:
    left_block = cq.Workplane("XY").box(0.030, 0.026, 0.062).translate((-0.019, -0.028, 0.0))
    right_block = cq.Workplane("XY").box(0.030, 0.026, 0.062).translate((-0.019, 0.028, 0.0))
    rear_bridge = cq.Workplane("XY").box(0.018, 0.040, 0.034).translate((-0.025, 0.0, 0.0))
    left_slider = cq.Workplane("XY").box(0.008, 0.014, 0.096).translate((-0.030, -0.021, 0.0))
    right_slider = cq.Workplane("XY").box(0.008, 0.014, 0.096).translate((-0.030, 0.021, 0.0))
    left_ear = cq.Workplane("XY").box(0.018, 0.010, 0.050).translate((0.005, -0.015, 0.0))
    right_ear = cq.Workplane("XY").box(0.018, 0.010, 0.050).translate((0.005, 0.015, 0.0))
    return {
        "carriage_frame_left": left_block,
        "carriage_frame_right": right_block,
        "rear_bridge": rear_bridge,
        "left_slider": left_slider,
        "right_slider": right_slider,
        "left_ear": left_ear,
        "right_ear": right_ear,
    }


def _center_plate_shapes() -> dict[str, cq.Workplane]:
    plate_profile = [
        (0.012, -0.026),
        (0.075, -0.024),
        (0.145, -0.018),
        (0.198, -0.011),
        (0.198, 0.011),
        (0.145, 0.018),
        (0.075, 0.024),
        (0.012, 0.026),
    ]
    arm_plate = (
        cq.Workplane("XZ")
        .polyline(plate_profile)
        .close()
        .extrude(0.016, both=True)
    )
    hinge_hub = cq.Workplane("XZ").circle(0.016).extrude(0.009, both=True)

    housing_outer = (
        cq.Workplane("XY")
        .box(0.052, 0.028, 0.038)
        .translate((0.214, 0.0, 0.0))
    )
    nose_outer = cq.Workplane("YZ").circle(0.011).extrude(0.018).translate((0.240, 0.0, 0.0))
    housing_bore = cq.Workplane("YZ").circle(0.007).extrude(0.090).translate((0.178, 0.0, 0.0))
    tip_housing = housing_outer.union(nose_outer).cut(housing_bore)

    return {
        "arm_plate": arm_plate,
        "hinge_hub": hinge_hub,
        "tip_housing_shell": tip_housing,
    }


def _probe_shape() -> cq.Workplane:
    main_rod = cq.Workplane("YZ").circle(0.006).extrude(0.060)
    nose_stem = cq.Workplane("YZ").circle(0.004).extrude(0.014).translate((0.060, 0.0, 0.0))
    tip_ball = cq.Workplane("XY").sphere(0.0045).translate((0.078, 0.0, 0.0))
    return main_rod.union(nose_stem).union(tip_ball)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lift_slide_arm")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    arm_silver = model.material("arm_silver", rgba=(0.73, 0.75, 0.77, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))

    mast = model.part(
        "mast",
        inertial=Inertial.from_geometry(
            Box((0.10, 0.09, 0.39)),
            mass=5.5,
            origin=Origin(xyz=(0.0, 0.0, 0.195)),
        ),
    )
    for name, shape in _mast_shapes().items():
        mast.visual(mesh_from_cadquery(shape, name), material=graphite, name=name)

    carriage = model.part(
        "carriage",
        inertial=Inertial.from_geometry(
            Box((0.050, 0.090, 0.100)),
            mass=1.2,
            origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        ),
    )
    for name, shape in _carriage_shapes().items():
        carriage.visual(mesh_from_cadquery(shape, name), material=carriage_gray, name=name)

    center_plate = model.part(
        "center_plate",
        inertial=Inertial.from_geometry(
            Box((0.250, 0.040, 0.060)),
            mass=0.95,
            origin=Origin(xyz=(0.125, 0.0, 0.0)),
        ),
    )
    plate_shapes = _center_plate_shapes()
    center_plate.visual(
        mesh_from_cadquery(plate_shapes["arm_plate"], "arm_plate"),
        material=arm_silver,
        name="arm_plate",
    )
    center_plate.visual(
        mesh_from_cadquery(plate_shapes["hinge_hub"], "hinge_hub"),
        material=carriage_gray,
        name="hinge_hub",
    )
    center_plate.visual(
        mesh_from_cadquery(plate_shapes["tip_housing_shell"], "tip_housing_shell"),
        material=graphite,
        name="tip_housing_shell",
    )

    probe = model.part(
        "probe",
        inertial=Inertial.from_geometry(
            Box((0.084, 0.013, 0.013)),
            mass=0.18,
            origin=Origin(xyz=(0.042, 0.0, 0.0)),
        ),
    )
    probe.visual(mesh_from_cadquery(_probe_shape(), "probe_rod"), material=steel, name="probe_rod")

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.074, 0.0, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.18, lower=0.0, upper=0.10),
    )
    model.articulation(
        "carriage_to_center_plate",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=center_plate,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=32.0,
            velocity=1.5,
            lower=radians(-40.0),
            upper=radians(55.0),
        ),
    )
    model.articulation(
        "center_plate_to_probe",
        ArticulationType.PRISMATIC,
        parent=center_plate,
        child=probe,
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.14, lower=0.0, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    center_plate = object_model.get_part("center_plate")
    probe = object_model.get_part("probe")

    mast_to_carriage = object_model.get_articulation("mast_to_carriage")
    carriage_to_center_plate = object_model.get_articulation("carriage_to_center_plate")
    center_plate_to_probe = object_model.get_articulation("center_plate_to_probe")

    left_rail = mast.get_visual("left_rail")
    right_rail = mast.get_visual("right_rail")
    left_slider = carriage.get_visual("left_slider")
    right_slider = carriage.get_visual("right_slider")
    tip_housing = center_plate.get_visual("tip_housing_shell")
    probe_rod = probe.get_visual("probe_rod")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    ctx.allow_isolated_part(
        probe,
        reason="Probe rides within a hollow tip guide with intentional running clearance rather than exterior contact.",
    )
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
        "expected_parts_present",
        all(part_obj is not None for part_obj in (mast, carriage, center_plate, probe)),
        "mast, carriage, center plate, and probe should all be authored parts.",
    )
    ctx.check(
        "joint_axes_match_mechanism",
        mast_to_carriage.axis == (0.0, 0.0, 1.0)
        and carriage_to_center_plate.axis == (0.0, 1.0, 0.0)
        and center_plate_to_probe.axis == (1.0, 0.0, 0.0),
        "Expected carriage lift on +Z, plate hinge on +Y, and probe extension on +X.",
    )

    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem=left_slider,
        negative_elem=left_rail,
        min_gap=-1e-6,
        max_gap=0.001,
        name="left_slider_runs_on_left_rail",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem=right_slider,
        negative_elem=right_rail,
        min_gap=-1e-6,
        max_gap=0.001,
        name="right_slider_runs_on_right_rail",
    )
    ctx.expect_within(
        probe,
        center_plate,
        axes="yz",
        inner_elem=probe_rod,
        outer_elem=tip_housing,
        margin=0.002,
        name="probe_stays_guided_inside_tip_housing",
    )

    with ctx.pose({mast_to_carriage: mast_to_carriage.motion_limits.lower}):
        carriage_low = ctx.part_world_position(carriage)
    with ctx.pose({mast_to_carriage: mast_to_carriage.motion_limits.upper}):
        carriage_high = ctx.part_world_position(carriage)
    ctx.check(
        "carriage_has_vertical_travel",
        carriage_low is not None
        and carriage_high is not None
        and carriage_high[2] > carriage_low[2] + 0.095
        and abs(carriage_high[0] - carriage_low[0]) < 1e-6
        and abs(carriage_high[1] - carriage_low[1]) < 1e-6,
        "The root carriage should translate vertically without drifting laterally.",
    )

    with ctx.pose({carriage_to_center_plate: carriage_to_center_plate.motion_limits.lower}):
        low_tip_aabb = ctx.part_element_world_aabb(center_plate, elem=tip_housing)
    with ctx.pose({carriage_to_center_plate: carriage_to_center_plate.motion_limits.upper}):
        high_tip_aabb = ctx.part_element_world_aabb(center_plate, elem=tip_housing)
    if low_tip_aabb is not None and high_tip_aabb is not None:
        low_tip_center = tuple((lo + hi) * 0.5 for lo, hi in zip(low_tip_aabb[0], low_tip_aabb[1]))
        high_tip_center = tuple((lo + hi) * 0.5 for lo, hi in zip(high_tip_aabb[0], high_tip_aabb[1]))
        ctx.check(
            "center_plate_hinges_in_vertical_plane",
            abs(high_tip_center[2] - low_tip_center[2]) > 0.12
            and abs(high_tip_center[0] - low_tip_center[0]) > 0.02
            and abs(high_tip_center[1] - low_tip_center[1]) < 0.01,
            "The distal housing should rise noticeably in Z while remaining in the hinge plane.",
        )
    else:
        ctx.fail("center_plate_hinges_in_vertical_plane", "Could not measure the tip housing in articulated poses.")

    with ctx.pose({center_plate_to_probe: center_plate_to_probe.motion_limits.lower}):
        probe_retracted = ctx.part_world_position(probe)
    with ctx.pose({center_plate_to_probe: center_plate_to_probe.motion_limits.upper}):
        probe_extended = ctx.part_world_position(probe)
    ctx.check(
        "probe_extends_from_tip_housing",
        probe_retracted is not None
        and probe_extended is not None
        and probe_extended[0] > probe_retracted[0] + 0.045
        and abs(probe_extended[1] - probe_retracted[1]) < 1e-6
        and abs(probe_extended[2] - probe_retracted[2]) < 1e-6,
        "The distal probe should extend forward through the tip housing without side drift.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
