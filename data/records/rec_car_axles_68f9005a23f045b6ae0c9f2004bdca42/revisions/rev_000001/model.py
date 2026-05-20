from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _add_beam_visuals(beam, *, steel, bushing_dark) -> None:
    beam.visual(
        Box((0.22, 0.84, 0.09)),
        origin=Origin(xyz=(0.04, 0.0, 0.10)),
        material=steel,
        name="beam_crossmember",
    )
    beam.visual(
        Box((0.14, 0.70, 0.04)),
        origin=Origin(xyz=(0.01, 0.0, 0.045)),
        material=steel,
        name="beam_lower_web",
    )

    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        bracket_y = side_sign * 0.48
        inner_ear_y = side_sign * 0.431
        outer_ear_y = side_sign * 0.529

        beam.visual(
            Box((0.28, 0.11, 0.10)),
            origin=Origin(xyz=(-0.15, side_sign * 0.365, 0.13)),
            material=steel,
            name=f"{side_name}_beam_end_box",
        )
        beam.visual(
            Box((0.05, 0.126, 0.028)),
            origin=Origin(xyz=(-0.335, bracket_y, 0.268)),
            material=steel,
            name=f"{side_name}_bracket_top_tie",
        )
        beam.visual(
            Box((0.05, 0.126, 0.020)),
            origin=Origin(xyz=(-0.335, bracket_y, 0.178)),
            material=steel,
            name=f"{side_name}_bracket_lower_tie",
        )
        beam.visual(
            Box((0.090, 0.028, 0.100)),
            origin=Origin(xyz=(-0.29, inner_ear_y, 0.223)),
            material=bushing_dark,
            name=f"{side_name}_pivot_inner_ear",
        )
        beam.visual(
            Box((0.090, 0.028, 0.100)),
            origin=Origin(xyz=(-0.29, outer_ear_y, 0.223)),
            material=bushing_dark,
            name=f"{side_name}_pivot_outer_ear",
        )


def _add_trailing_arm_visuals(arm, *, side_sign: float, steel, bushing_dark) -> None:
    arm.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=bushing_dark,
        name="bushing_sleeve",
    )
    arm.visual(
        Box((0.16, 0.058, 0.072)),
        origin=Origin(xyz=(0.085, 0.0, -0.020)),
        material=steel,
        name="arm_root_web",
    )
    arm.visual(
        Box((0.18, 0.088, 0.062)),
        origin=Origin(xyz=(0.18, 0.0, -0.048)),
        material=steel,
        name="arm_transition_box",
    )
    arm.visual(
        Box((0.48, 0.095, 0.065)),
        origin=Origin(xyz=(0.31, 0.0, -0.055), rpy=(0.0, -0.10, 0.0)),
        material=steel,
        name="arm_main_box",
    )
    arm.visual(
        Box((0.24, 0.060, 0.045)),
        origin=Origin(xyz=(0.19, 0.0, -0.085), rpy=(0.0, -0.12, 0.0)),
        material=steel,
        name="arm_lower_gusset",
    )
    arm.visual(
        Box((0.08, 0.11, 0.016)),
        origin=Origin(xyz=(0.36, 0.0, -0.010)),
        material=steel,
        name="spring_pad",
    )
    arm.visual(
        Box((0.12, 0.030, 0.115)),
        origin=Origin(xyz=(0.57, side_sign * 0.015, -0.080)),
        material=bushing_dark,
        name="hub_mount",
    )
    arm.visual(
        Box((0.11, 0.010, 0.100)),
        origin=Origin(xyz=(0.56, side_sign * 0.025, -0.080)),
        material=steel,
        name="outer_carrier_plate",
    )


def _add_hub_visuals(hub, *, side_sign: float, hub_steel, machined_steel) -> None:
    spin_along_y = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    hub.visual(
        Cylinder(radius=0.048, length=0.060),
        origin=Origin(xyz=(0.0, side_sign * 0.030, 0.0), rpy=spin_along_y.rpy),
        material=machined_steel,
        name="hub_shell",
    )
    hub.visual(
        Cylinder(radius=0.078, length=0.010),
        origin=Origin(xyz=(0.0, side_sign * 0.065, 0.0), rpy=spin_along_y.rpy),
        material=hub_steel,
        name="hub_flange",
    )
    hub.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, side_sign * 0.083, 0.0), rpy=spin_along_y.rpy),
        material=machined_steel,
        name="stub_nose",
    )
    for x, z in (
        (0.032, 0.0),
        (0.010, 0.030),
        (-0.026, 0.018),
        (-0.026, -0.018),
        (0.010, -0.030),
    ):
        hub.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(x, side_sign * 0.066, z), rpy=spin_along_y.rpy),
            material=machined_steel,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trailing_arm_rear_axle")

    beam_paint = model.material("beam_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    bushing_dark = model.material("bushing_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    hub_steel = model.material("hub_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.76, 0.77, 0.79, 1.0))

    beam = model.part("torsion_beam")
    beam.inertial = Inertial.from_geometry(
        Box((0.72, 1.30, 0.32)),
        mass=48.0,
        origin=Origin(xyz=(-0.10, 0.0, 0.12)),
    )
    _add_beam_visuals(beam, steel=beam_paint, bushing_dark=bushing_dark)

    left_arm = model.part("left_trailing_arm")
    left_arm.inertial = Inertial.from_geometry(
        Box((0.72, 0.11, 0.15)),
        mass=17.0,
        origin=Origin(xyz=(0.32, 0.0, -0.05)),
    )
    _add_trailing_arm_visuals(left_arm, side_sign=1.0, steel=beam_paint, bushing_dark=bushing_dark)

    right_arm = model.part("right_trailing_arm")
    right_arm.inertial = Inertial.from_geometry(
        Box((0.72, 0.11, 0.15)),
        mass=17.0,
        origin=Origin(xyz=(0.32, 0.0, -0.05)),
    )
    _add_trailing_arm_visuals(right_arm, side_sign=-1.0, steel=beam_paint, bushing_dark=bushing_dark)

    left_hub = model.part("left_wheel_hub")
    left_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.08, length=0.12),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.06, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    _add_hub_visuals(left_hub, side_sign=1.0, hub_steel=hub_steel, machined_steel=machined_steel)

    right_hub = model.part("right_wheel_hub")
    right_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.08, length=0.12),
        mass=5.5,
        origin=Origin(xyz=(0.0, -0.06, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    _add_hub_visuals(right_hub, side_sign=-1.0, hub_steel=hub_steel, machined_steel=machined_steel)

    model.articulation(
        "beam_to_left_arm",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=left_arm,
        origin=Origin(xyz=(-0.29, 0.48, 0.223)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=1.2, lower=-0.35, upper=0.40),
    )
    model.articulation(
        "beam_to_right_arm",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=right_arm,
        origin=Origin(xyz=(-0.29, -0.48, 0.223)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=1.2, lower=-0.35, upper=0.40),
    )
    model.articulation(
        "left_arm_to_hub",
        ArticulationType.CONTINUOUS,
        parent=left_arm,
        child=left_hub,
        origin=Origin(xyz=(0.645, 0.030, -0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=45.0),
    )
    model.articulation(
        "right_arm_to_hub",
        ArticulationType.CONTINUOUS,
        parent=right_arm,
        child=right_hub,
        origin=Origin(xyz=(0.645, -0.030, -0.080)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=45.0),
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

    beam = object_model.get_part("torsion_beam")
    left_arm = object_model.get_part("left_trailing_arm")
    right_arm = object_model.get_part("right_trailing_arm")
    left_hub = object_model.get_part("left_wheel_hub")
    right_hub = object_model.get_part("right_wheel_hub")

    left_pivot = object_model.get_articulation("beam_to_left_arm")
    right_pivot = object_model.get_articulation("beam_to_right_arm")
    left_spin = object_model.get_articulation("left_arm_to_hub")
    right_spin = object_model.get_articulation("right_arm_to_hub")

    left_sleeve = left_arm.get_visual("bushing_sleeve")
    right_sleeve = right_arm.get_visual("bushing_sleeve")
    left_inner_ear = beam.get_visual("left_pivot_inner_ear")
    right_inner_ear = beam.get_visual("right_pivot_inner_ear")
    left_hub_mount = left_arm.get_visual("hub_mount")
    right_hub_mount = right_arm.get_visual("hub_mount")
    left_hub_shell = left_hub.get_visual("hub_shell")
    right_hub_shell = right_hub.get_visual("hub_shell")

    ctx.check(
        "trailing arms use independent revolute pivots",
        (
            left_pivot.articulation_type == ArticulationType.REVOLUTE
            and right_pivot.articulation_type == ArticulationType.REVOLUTE
            and left_pivot.axis == (0.0, -1.0, 0.0)
            and right_pivot.axis == (0.0, -1.0, 0.0)
            and left_pivot.motion_limits is not None
            and right_pivot.motion_limits is not None
            and left_pivot.motion_limits.lower == -0.35
            and left_pivot.motion_limits.upper == 0.40
            and right_pivot.motion_limits.lower == -0.35
            and right_pivot.motion_limits.upper == 0.40
        ),
        details=f"left={left_pivot.axis} limits={left_pivot.motion_limits}, right={right_pivot.axis} limits={right_pivot.motion_limits}",
    )
    ctx.check(
        "wheel hubs use continuous lateral spin axes",
        (
            left_spin.articulation_type == ArticulationType.CONTINUOUS
            and right_spin.articulation_type == ArticulationType.CONTINUOUS
            and left_spin.axis == (0.0, 1.0, 0.0)
            and right_spin.axis == (0.0, -1.0, 0.0)
        ),
        details=f"left={left_spin.axis}, right={right_spin.axis}",
    )

    ctx.expect_contact(
        left_arm,
        beam,
        elem_a=left_sleeve,
        elem_b=left_inner_ear,
        name="left arm bushing seats against beam bracket",
    )
    ctx.expect_contact(
        right_arm,
        beam,
        elem_a=right_sleeve,
        elem_b=right_inner_ear,
        name="right arm bushing seats against beam bracket",
    )
    ctx.expect_contact(
        left_hub,
        left_arm,
        elem_a=left_hub_shell,
        elem_b=left_hub_mount,
        name="left hub seats on the arm carrier",
    )
    ctx.expect_contact(
        right_hub,
        right_arm,
        elem_a=right_hub_shell,
        elem_b=right_hub_mount,
        name="right hub seats on the arm carrier",
    )
    ctx.expect_origin_distance(
        left_hub,
        right_hub,
        axes="y",
        min_dist=0.95,
        max_dist=1.10,
        name="hub spacing stays axle width",
    )

    rest_left = ctx.part_world_position(left_hub)
    rest_right = ctx.part_world_position(right_hub)
    with ctx.pose({left_pivot: 0.40}):
        raised_left = ctx.part_world_position(left_hub)
        steady_right = ctx.part_world_position(right_hub)
    ctx.check(
        "left arm rises without moving the right side",
        (
            rest_left is not None
            and raised_left is not None
            and rest_right is not None
            and steady_right is not None
            and raised_left[2] > rest_left[2] + 0.12
            and abs(steady_right[2] - rest_right[2]) < 0.01
        ),
        details=f"rest_left={rest_left}, raised_left={raised_left}, rest_right={rest_right}, steady_right={steady_right}",
    )

    with ctx.pose({right_pivot: -0.35}):
        dropped_right = ctx.part_world_position(right_hub)
    ctx.check(
        "negative right-arm travel drops the hub",
        rest_right is not None and dropped_right is not None and dropped_right[2] < rest_right[2] - 0.08,
        details=f"rest_right={rest_right}, dropped_right={dropped_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
