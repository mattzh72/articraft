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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _xy_section(
    length: float,
    width: float,
    radius: float,
    z: float,
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + x_shift, y + y_shift, z) for x, y in rounded_rect_profile(length, width, radius)]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_shift) for y, z in rounded_rect_profile(width, height, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pastel_stand_mixer")

    body = model.material("pastel_body", rgba=(0.76, 0.85, 0.83, 1.0))
    cream = model.material("warm_cream", rgba=(0.95, 0.93, 0.88, 1.0))
    metal = model.material("polished_metal", rgba=(0.88, 0.89, 0.91, 1.0))
    dark = model.material("charcoal_trim", rgba=(0.23, 0.25, 0.27, 1.0))

    base = model.part("base")

    base_shell = section_loft(
        [
            _xy_section(0.35, 0.23, 0.085, 0.0, x_shift=0.01),
            _xy_section(0.31, 0.20, 0.074, 0.032, x_shift=0.02),
            _xy_section(0.27, 0.17, 0.060, 0.060, x_shift=0.03),
        ]
    )
    base.visual(
        mesh_from_geometry(base_shell, "mixer_base_shell"),
        material=body,
        name="base_shell",
    )

    pedestal_shell = section_loft(
        [
            _yz_section(0.092, 0.110, 0.034, -0.155, z_shift=0.110),
            _yz_section(0.112, 0.210, 0.046, -0.140, z_shift=0.168),
            _yz_section(0.096, 0.242, 0.040, -0.126, z_shift=0.196),
        ]
    )
    base.visual(
        mesh_from_geometry(pedestal_shell, "mixer_pedestal_shell"),
        material=body,
        name="pedestal_shell",
    )
    base.visual(
        Box((0.145, 0.145, 0.012)),
        origin=Origin(xyz=(0.070, 0.0, 0.066)),
        material=cream,
        name="carriage_deck",
    )
    base.visual(
        Box((0.135, 0.022, 0.012)),
        origin=Origin(xyz=(0.070, 0.042, 0.072)),
        material=cream,
        name="right_slide_rail",
    )
    base.visual(
        Box((0.135, 0.022, 0.012)),
        origin=Origin(xyz=(0.070, -0.042, 0.072)),
        material=cream,
        name="left_slide_rail",
    )
    base.visual(
        Box((0.050, 0.110, 0.200)),
        origin=Origin(xyz=(-0.130, 0.0, 0.150)),
        material=body,
        name="rear_neck_core",
    )
    base.visual(
        Box((0.030, 0.026, 0.064)),
        origin=Origin(xyz=(-0.129, 0.057, 0.246)),
        material=cream,
        name="right_hinge_cheek",
    )
    base.visual(
        Box((0.030, 0.026, 0.064)),
        origin=Origin(xyz=(-0.129, -0.057, 0.246)),
        material=cream,
        name="left_hinge_cheek",
    )
    base.visual(
        Box((0.040, 0.036, 0.044)),
        origin=Origin(xyz=(0.020, 0.092, 0.080)),
        material=cream,
        name="selector_pod",
    )
    base.visual(
        Box((0.060, 0.028, 0.008)),
        origin=Origin(xyz=(-0.094, -0.050, 0.124)),
        material=cream,
        name="lock_track",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.37, 0.24, 0.30)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.118, 0.136, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=body,
        name="carriage_plate",
    )
    bowl_carriage.visual(
        Box((0.102, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.041, 0.005)),
        material=cream,
        name="right_runner",
    )
    bowl_carriage.visual(
        Box((0.102, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, -0.041, 0.005)),
        material=cream,
        name="left_runner",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=cream,
        name="bowl_seat",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.118, 0.136, 0.030)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.070, 0.0, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.08, lower=0.0, upper=0.045),
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.030, 0.000),
            (0.048, 0.006),
            (0.082, 0.028),
            (0.101, 0.070),
            (0.108, 0.098),
            (0.116, 0.104),
            (0.118, 0.108),
        ],
        [
            (0.000, 0.009),
            (0.042, 0.014),
            (0.076, 0.032),
            (0.094, 0.072),
            (0.100, 0.096),
            (0.107, 0.101),
        ],
        segments=72,
        end_cap="round",
        lip_samples=10,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "rolled_rim_bowl"),
        material=metal,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.118, length=0.108),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_carriage,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    head = model.part("head")
    head_shell = section_loft(
        [
            _yz_section(0.090, 0.110, 0.034, 0.018, z_shift=0.002),
            _yz_section(0.148, 0.172, 0.055, 0.120, z_shift=0.018),
            _yz_section(0.146, 0.162, 0.052, 0.220, z_shift=0.012),
            _yz_section(0.108, 0.116, 0.040, 0.300, z_shift=-0.002),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "mixer_head_shell"),
        material=body,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.074),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.040, 0.066, 0.036)),
        origin=Origin(xyz=(0.036, 0.0, 0.006)),
        material=body,
        name="rear_hinge_fairing",
    )
    head.visual(
        Cylinder(radius=0.048, length=0.070),
        origin=Origin(xyz=(0.252, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream,
        name="nose_band",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(0.193, 0.0, -0.006)),
        material=cream,
        name="tool_socket",
    )
    head.visual(
        Box((0.128, 0.088, 0.094)),
        origin=Origin(xyz=(0.170, 0.0, 0.040)),
        material=body,
        name="nose_bridge",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.31, 0.18, 0.18)),
        mass=4.8,
        origin=Origin(xyz=(0.155, 0.0, 0.000)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.123, 0.0, 0.286)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="selector_dial",
    )
    speed_selector.visual(
        Box((0.004, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.021, 0.0)),
        material=dark,
        name="selector_pointer",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.030, 0.026, 0.052)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
    )

    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(0.020, 0.110, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=-1.0, upper=1.0),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.040, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=cream,
        name="lock_slider",
    )
    head_lock.visual(
        Box((0.022, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark,
        name="lock_ridge",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.040, 0.018, 0.016)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.060, -0.050, 0.128)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.014),
    )

    dough_hook = model.part("dough_hook")
    hook_geom = CylinderGeometry(radius=0.007, height=0.014).translate(0.0, 0.0, -0.050)
    hook_geom.merge(CylinderGeometry(radius=0.010, height=0.012).translate(0.0, 0.0, -0.062))
    hook_geom.merge(
        tube_from_spline_points(
            [
                (0.000, 0.000, -0.062),
                (0.008, 0.000, -0.078),
                (0.016, 0.000, -0.098),
                (0.014, 0.006, -0.117),
                (0.004, 0.013, -0.131),
                (-0.006, 0.009, -0.139),
                (-0.004, -0.003, -0.143),
            ],
            radius=0.006,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
    )
    dough_hook.visual(
        mesh_from_geometry(hook_geom, "dough_hook"),
        material=metal,
        name="dough_hook",
    )
    dough_hook.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.160),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    model.articulation(
        "head_to_dough_hook",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=dough_hook,
        origin=Origin(xyz=(0.193, 0.0, -0.010)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl_carriage = object_model.get_part("bowl_carriage")
    bowl = object_model.get_part("bowl")
    head_lock = object_model.get_part("head_lock")
    dough_hook = object_model.get_part("dough_hook")

    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    hook_spin = object_model.get_articulation("head_to_dough_hook")
    selector = object_model.get_articulation("base_to_speed_selector")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "golden articulation structure is preserved",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and hook_spin.articulation_type == ArticulationType.CONTINUOUS
        and selector.articulation_type == ArticulationType.REVOLUTE
        and lock_slide.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"bowl_slide={bowl_slide.articulation_type}, "
            f"head_tilt={head_tilt.articulation_type}, "
            f"hook_spin={hook_spin.articulation_type}, "
            f"selector={selector.articulation_type}, "
            f"lock_slide={lock_slide.articulation_type}"
        ),
    )

    ctx.expect_within(
        dough_hook,
        bowl,
        axes="xy",
        margin=0.012,
        name="dough hook sits within the bowl footprint at rest",
    )
    ctx.expect_overlap(
        bowl_carriage,
        object_model.get_part("base"),
        axes="xy",
        min_overlap=0.075,
        name="bowl carriage stays seated on the base rails at rest",
    )

    rest_carriage_pos = ctx.part_world_position(bowl_carriage)
    rest_lock_pos = ctx.part_world_position(head_lock)
    rest_hook_pos = ctx.part_world_position(dough_hook)

    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        extended_carriage_pos = ctx.part_world_position(bowl_carriage)
        ctx.check(
            "bowl carriage slides forward",
            rest_carriage_pos is not None
            and extended_carriage_pos is not None
            and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.030,
            details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
        )
        ctx.expect_overlap(
            bowl_carriage,
            object_model.get_part("base"),
            axes="xy",
            min_overlap=0.040,
            name="extended carriage still overlaps the base support footprint",
        )

    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        raised_hook_pos = ctx.part_world_position(dough_hook)
        ctx.check(
            "tilt head lifts the hook clear of the bowl",
            rest_hook_pos is not None
            and raised_hook_pos is not None
            and raised_hook_pos[2] > rest_hook_pos[2] + 0.120,
            details=f"rest={rest_hook_pos}, raised={raised_hook_pos}",
        )
        ctx.expect_gap(
            dough_hook,
            bowl,
            axis="z",
            min_gap=0.025,
            name="open head leaves the hook above the bowl rim",
        )

    with ctx.pose({lock_slide: lock_slide.motion_limits.upper}):
        extended_lock_pos = ctx.part_world_position(head_lock)
        ctx.check(
            "head lock control translates",
            rest_lock_pos is not None
            and extended_lock_pos is not None
            and extended_lock_pos[0] > rest_lock_pos[0] + 0.010,
            details=f"rest={rest_lock_pos}, extended={extended_lock_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
