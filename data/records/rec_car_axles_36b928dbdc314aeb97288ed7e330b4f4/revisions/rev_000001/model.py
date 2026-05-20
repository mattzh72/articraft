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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hub_shell(name: str):
    outer_profile = [
        (0.034, -0.070),
        (0.046, -0.066),
        (0.056, -0.048),
        (0.072, -0.020),
        (0.078, 0.000),
        (0.072, 0.020),
        (0.058, 0.042),
        (0.046, 0.060),
        (0.040, 0.070),
    ]
    inner_profile = [
        (0.031, -0.066),
        (0.031, -0.020),
        (0.033, 0.022),
        (0.031, 0.062),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
        ).rotate_y(math.pi / 2.0),
        name,
    )


def _add_stub_spindle_visuals(part, *, side_sign: float, steel_dark, steel_bright, prefix: str) -> None:
    part.visual(
        Box((0.072, 0.090, 0.056)),
        material=steel_dark,
        name=f"{prefix}_kingpin_carrier",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.056),
        material=steel_bright,
        name=f"{prefix}_kingpin_boss",
    )
    part.visual(
        Box((0.058, 0.110, 0.030)),
        origin=Origin(xyz=(0.0, -0.080, -0.013)),
        material=steel_dark,
        name=f"{prefix}_steering_arm",
    )
    part.visual(
        Cylinder(radius=0.045, length=0.040),
        origin=Origin(
            xyz=(side_sign * 0.065, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_dark,
        name=f"{prefix}_spindle_shoulder",
    )
    part.visual(
        Cylinder(radius=0.025, length=0.220),
        origin=Origin(
            xyz=(side_sign * 0.110, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_bright,
        name=f"{prefix}_spindle_shaft",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_stub_axle_kingpin")

    axle_paint = model.material("axle_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    forged_steel = model.material("forged_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.68, 0.69, 0.72, 1.0))

    beam_length = 1.34
    beam_depth = 0.16
    beam_height = 0.14
    tab_length = 0.09
    tab_depth = 0.10
    tab_thickness = 0.035
    tab_center_z = 0.0475
    kingpin_x = (beam_length * 0.5) + (tab_length * 0.5)

    beam_axle = model.part("beam_axle")
    beam_axle.visual(
        Box((beam_length, beam_depth, beam_height)),
        material=axle_paint,
        name="beam_body",
    )
    beam_axle.visual(
        Box((0.26, 0.14, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=axle_paint,
        name="spring_pad",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        beam_axle.visual(
            Box((tab_length, tab_depth, tab_thickness)),
            origin=Origin(xyz=(side_sign * kingpin_x, 0.0, tab_center_z)),
            material=axle_paint,
            name=f"{side_name}_upper_clevis",
        )
        beam_axle.visual(
            Box((tab_length, tab_depth, tab_thickness)),
            origin=Origin(xyz=(side_sign * kingpin_x, 0.0, -tab_center_z)),
            material=axle_paint,
            name=f"{side_name}_lower_clevis",
        )
        beam_axle.visual(
            Cylinder(radius=0.026, length=0.028),
            origin=Origin(xyz=(side_sign * kingpin_x, 0.0, 0.046)),
            material=machined_steel,
            name=f"{side_name}_upper_bushing",
        )
        beam_axle.visual(
            Cylinder(radius=0.026, length=0.028),
            origin=Origin(xyz=(side_sign * kingpin_x, 0.0, -0.046)),
            material=machined_steel,
            name=f"{side_name}_lower_bushing",
        )
    beam_axle.inertial = Inertial.from_geometry(
        Box((1.46, 0.18, 0.21)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    left_stub_spindle = model.part("left_stub_spindle")
    _add_stub_spindle_visuals(
        left_stub_spindle,
        side_sign=-1.0,
        steel_dark=forged_steel,
        steel_bright=machined_steel,
        prefix="left",
    )
    left_stub_spindle.inertial = Inertial.from_geometry(
        Box((0.25, 0.11, 0.10)),
        mass=12.0,
        origin=Origin(xyz=(-0.10, -0.02, 0.0)),
    )

    right_stub_spindle = model.part("right_stub_spindle")
    _add_stub_spindle_visuals(
        right_stub_spindle,
        side_sign=1.0,
        steel_dark=forged_steel,
        steel_bright=machined_steel,
        prefix="right",
    )
    right_stub_spindle.inertial = Inertial.from_geometry(
        Box((0.25, 0.11, 0.10)),
        mass=12.0,
        origin=Origin(xyz=(0.10, -0.02, 0.0)),
    )

    left_hub = model.part("left_hub")
    left_hub.visual(_hub_shell("left_hub_shell"), material=machined_steel, name="left_hub_shell")
    left_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.140),
        mass=7.5,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_hub = model.part("right_hub")
    right_hub.visual(_hub_shell("right_hub_shell"), material=machined_steel, name="right_hub_shell")
    right_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.140),
        mass=7.5,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "left_kingpin",
        ArticulationType.REVOLUTE,
        parent=beam_axle,
        child=left_stub_spindle,
        origin=Origin(xyz=(-kingpin_x, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=2.0,
            lower=math.radians(-38.0),
            upper=math.radians(38.0),
        ),
    )
    model.articulation(
        "right_kingpin",
        ArticulationType.REVOLUTE,
        parent=beam_axle,
        child=right_stub_spindle,
        origin=Origin(xyz=(kingpin_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=2.0,
            lower=math.radians(-38.0),
            upper=math.radians(38.0),
        ),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=left_stub_spindle,
        child=left_hub,
        origin=Origin(xyz=(-0.155, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=30.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=right_stub_spindle,
        child=right_hub,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    beam_axle = object_model.get_part("beam_axle")
    left_stub_spindle = object_model.get_part("left_stub_spindle")
    right_stub_spindle = object_model.get_part("right_stub_spindle")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")

    left_kingpin = object_model.get_articulation("left_kingpin")
    right_kingpin = object_model.get_articulation("right_kingpin")
    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")

    ctx.check(
        "kingpins are vertical revolute pivots",
        left_kingpin.articulation_type == ArticulationType.REVOLUTE
        and right_kingpin.articulation_type == ArticulationType.REVOLUTE
        and left_kingpin.axis == (0.0, 0.0, -1.0)
        and right_kingpin.axis == (0.0, 0.0, 1.0),
        details=f"left_axis={left_kingpin.axis}, right_axis={right_kingpin.axis}",
    )
    ctx.check(
        "wheel hubs use continuous bearing rotation",
        left_hub_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_hub_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_hub_spin.axis == (1.0, 0.0, 0.0)
        and right_hub_spin.axis == (1.0, 0.0, 0.0),
        details=f"left_axis={left_hub_spin.axis}, right_axis={right_hub_spin.axis}",
    )

    with ctx.pose({left_kingpin: 0.0, right_kingpin: 0.0}):
        ctx.expect_gap(
            beam_axle,
            left_stub_spindle,
            axis="z",
            positive_elem="left_upper_clevis",
            negative_elem="left_kingpin_carrier",
            min_gap=0.001,
            max_gap=0.004,
            name="left carrier clears upper clevis",
        )
        ctx.expect_gap(
            left_stub_spindle,
            beam_axle,
            axis="z",
            positive_elem="left_kingpin_carrier",
            negative_elem="left_lower_clevis",
            min_gap=0.001,
            max_gap=0.004,
            name="left carrier clears lower clevis",
        )
        ctx.expect_gap(
            beam_axle,
            right_stub_spindle,
            axis="z",
            positive_elem="right_upper_clevis",
            negative_elem="right_kingpin_carrier",
            min_gap=0.001,
            max_gap=0.004,
            name="right carrier clears upper clevis",
        )
        ctx.expect_gap(
            right_stub_spindle,
            beam_axle,
            axis="z",
            positive_elem="right_kingpin_carrier",
            negative_elem="right_lower_clevis",
            min_gap=0.001,
            max_gap=0.004,
            name="right carrier clears lower clevis",
        )
        ctx.expect_overlap(
            left_stub_spindle,
            left_hub,
            axes="x",
            elem_a="left_spindle_shaft",
            elem_b="left_hub_shell",
            min_overlap=0.11,
            name="left hub remains retained on spindle",
        )
        ctx.expect_overlap(
            left_stub_spindle,
            left_hub,
            axes="yz",
            elem_a="left_spindle_shaft",
            elem_b="left_hub_shell",
            min_overlap=0.05,
            name="left hub stays coaxial with spindle",
        )
        ctx.expect_overlap(
            right_stub_spindle,
            right_hub,
            axes="x",
            elem_a="right_spindle_shaft",
            elem_b="right_hub_shell",
            min_overlap=0.11,
            name="right hub remains retained on spindle",
        )
        ctx.expect_overlap(
            right_stub_spindle,
            right_hub,
            axes="yz",
            elem_a="right_spindle_shaft",
            elem_b="right_hub_shell",
            min_overlap=0.05,
            name="right hub stays coaxial with spindle",
        )

    left_rest = ctx.part_world_position(left_hub)
    right_rest = ctx.part_world_position(right_hub)
    with ctx.pose({left_kingpin: math.radians(25.0), right_kingpin: math.radians(25.0)}):
        left_steered = ctx.part_world_position(left_hub)
        right_steered = ctx.part_world_position(right_hub)

    ctx.check(
        "kingpin steering yaws both stub assemblies about the beam ends",
        left_rest is not None
        and right_rest is not None
        and left_steered is not None
        and right_steered is not None
        and left_steered[1] > left_rest[1] + 0.045
        and right_steered[1] > right_rest[1] + 0.045
        and abs(left_steered[2] - left_rest[2]) < 0.01
        and abs(right_steered[2] - right_rest[2]) < 0.01,
        details=(
            f"left_rest={left_rest}, left_steered={left_steered}, "
            f"right_rest={right_rest}, right_steered={right_steered}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
