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
    return [
        (x + x_shift, y + y_shift, z)
        for x, y in rounded_rect_profile(length, width, radius)
    ]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_center: float = 0.0,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_shift, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius)
    ]


def _build_dough_hook_mesh():
    hook_path = [
        (0.0, 0.0, -0.032),
        (0.007, 0.0, -0.042),
        (0.019, 0.0, -0.050),
        (0.028, 0.0, -0.062),
        (0.024, 0.0, -0.074),
        (0.008, 0.0, -0.086),
        (-0.003, 0.0, -0.074),
        (-0.002, 0.0, -0.056),
    ]
    geom = CylinderGeometry(radius=0.006, height=0.034).translate(0.0, 0.0, -0.017)
    geom.merge(CylinderGeometry(radius=0.010, height=0.014).translate(0.0, 0.0, -0.039))
    geom.merge(
        tube_from_spline_points(
            hook_path,
            radius=0.007,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        )
    )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="designer_stand_mixer")

    body_paint = model.material("body_paint", rgba=(0.92, 0.90, 0.83, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    bowl_bright = model.material("bowl_bright", rgba=(0.96, 0.79, 0.20, 1.0))
    tool_metal = model.material("tool_metal", rgba=(0.83, 0.84, 0.86, 1.0))
    control_dark = model.material("control_dark", rgba=(0.14, 0.15, 0.16, 1.0))

    base = model.part("base")
    base_shell = section_loft(
        [
            _xy_section(0.360, 0.238, 0.070, 0.000),
            _xy_section(0.350, 0.230, 0.068, 0.030),
            _xy_section(0.322, 0.210, 0.058, 0.060),
            _xy_section(0.278, 0.178, 0.045, 0.083),
        ]
    )
    base.visual(
        mesh_from_geometry(base_shell, "mixer_base_shell"),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=body_paint,
        name="base_shell",
    )

    pedestal = section_loft(
        [
            _xy_section(0.106, 0.094, 0.030, 0.000, x_shift=-0.010),
            _xy_section(0.086, 0.080, 0.026, 0.090, x_shift=-0.014),
            _xy_section(0.066, 0.066, 0.021, 0.170, x_shift=-0.018),
            _xy_section(0.054, 0.058, 0.018, 0.208, x_shift=-0.018),
        ]
    )
    base.visual(
        mesh_from_geometry(pedestal, "mixer_pedestal"),
        origin=Origin(xyz=(-0.105, 0.0, 0.078)),
        material=body_paint,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.096),
        origin=Origin(xyz=(-0.123, 0.0, 0.286), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    base.visual(
        Box((0.148, 0.090, 0.018)),
        origin=Origin(xyz=(0.106, 0.0, 0.087)),
        material=body_paint,
        name="slide_deck",
    )
    base.visual(
        Box((0.132, 0.016, 0.010)),
        origin=Origin(xyz=(0.106, -0.050, 0.101)),
        material=trim_dark,
        name="left_slide_rail",
    )
    base.visual(
        Box((0.132, 0.016, 0.010)),
        origin=Origin(xyz=(0.106, 0.050, 0.101)),
        material=trim_dark,
        name="right_slide_rail",
    )
    base.visual(
        Box((0.030, 0.028, 0.032)),
        origin=Origin(xyz=(0.078, 0.107, 0.082)),
        material=trim_dark,
        name="selector_pod",
    )
    base.visual(
        Box((0.022, 0.020, 0.024)),
        origin=Origin(xyz=(-0.124, 0.039, 0.214)),
        material=trim_dark,
        name="lock_guide",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.400, 0.250, 0.310)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        Box((0.110, 0.078, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim_dark,
        name="carriage_block",
    )
    bowl.visual(
        Cylinder(radius=0.024, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=trim_dark,
        name="pedestal_post",
    )
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.026, 0.000),
            (0.052, 0.010),
            (0.089, 0.040),
            (0.109, 0.086),
            (0.116, 0.118),
        ],
        [
            (0.000, 0.006),
            (0.046, 0.016),
            (0.082, 0.042),
            (0.101, 0.086),
            (0.108, 0.113),
        ],
        segments=60,
        lip_samples=10,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "mixer_bowl_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=bowl_bright,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.118, length=0.164),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
    )

    head = model.part("head")
    head_shell = section_loft(
        [
            _yz_section(0.076, 0.046, 0.018, 0.018, z_center=0.055),
            _yz_section(0.098, 0.076, 0.026, 0.072, z_center=0.060),
            _yz_section(0.140, 0.100, 0.038, 0.138, z_center=0.050),
            _yz_section(0.118, 0.088, 0.032, 0.202, z_center=0.034),
            _yz_section(0.072, 0.060, 0.020, 0.266, z_center=0.008),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "mixer_head_shell"),
        origin=Origin(),
        material=body_paint,
        name="head_shell",
    )
    head.visual(
        Box((0.028, 0.006, 0.044)),
        origin=Origin(xyz=(0.010, -0.051, 0.022)),
        material=body_paint,
        name="left_hinge_fork",
    )
    head.visual(
        Box((0.028, 0.006, 0.044)),
        origin=Origin(xyz=(0.010, 0.051, 0.022)),
        material=body_paint,
        name="right_hinge_fork",
    )
    head.visual(
        Box((0.060, 0.108, 0.026)),
        origin=Origin(xyz=(0.040, 0.0, 0.039)),
        material=body_paint,
        name="hinge_bridge",
    )
    head.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.236, 0.0, -0.013)),
        material=trim_dark,
        name="drive_collar",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.290, 0.150, 0.160)),
        mass=4.8,
        origin=Origin(xyz=(0.145, 0.0, 0.000)),
    )

    tool = model.part("tool")
    tool.visual(
        mesh_from_geometry(_build_dough_hook_mesh(), "mixer_dough_hook"),
        material=tool_metal,
        name="dough_hook",
    )
    tool.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.108),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_dark,
        name="selector_dial",
    )
    speed_selector.visual(
        Box((0.012, 0.010, 0.028)),
        origin=Origin(xyz=(0.018, 0.010, 0.006)),
        material=control_dark,
        name="selector_handle",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.038, 0.020, 0.034)),
        mass=0.09,
        origin=Origin(xyz=(0.010, 0.008, 0.004)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Cylinder(radius=0.0055, length=0.028),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_dark,
        name="lock_pin",
    )
    head_lock.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.031, 0.0)),
        material=control_dark,
        name="lock_knob",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.016, 0.042, 0.014)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.102, 0.0, 0.096)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.06, lower=0.0, upper=0.032),
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
            velocity=1.1,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )
    model.articulation(
        "head_to_tool",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=tool,
        origin=Origin(xyz=(0.236, 0.0, -0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(0.078, 0.121, 0.082)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=math.radians(-40.0),
            upper=math.radians(40.0),
        ),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.124, 0.049, 0.214)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.04, lower=0.0, upper=0.010),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    tool = object_model.get_part("tool")
    selector = object_model.get_part("speed_selector")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl")
    head_hinge = object_model.get_articulation("base_to_head")
    tool_spin = object_model.get_articulation("head_to_tool")
    selector_joint = object_model.get_articulation("base_to_speed_selector")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    ctx.check(
        "golden articulation tree preserved",
        bowl_slide.joint_type == ArticulationType.PRISMATIC
        and head_hinge.joint_type == ArticulationType.REVOLUTE
        and tool_spin.joint_type == ArticulationType.CONTINUOUS
        and selector_joint.joint_type == ArticulationType.REVOLUTE
        and lock_slide.joint_type == ArticulationType.PRISMATIC,
        details=(
            f"types={[bowl_slide.joint_type, head_hinge.joint_type, tool_spin.joint_type, selector_joint.joint_type, lock_slide.joint_type]}"
        ),
    )

    ctx.expect_overlap(
        bowl,
        base,
        axes="xy",
        elem_a="carriage_block",
        elem_b="slide_deck",
        min_overlap=0.070,
        name="bowl carriage stays seated on slide deck at rest",
    )

    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        bowl_extended = ctx.part_world_position(bowl)
        ctx.expect_overlap(
            bowl,
            base,
            axes="xy",
            elem_a="carriage_block",
            elem_b="slide_deck",
            min_overlap=0.038,
            name="bowl carriage retains deck overlap at full adjustment",
        )
    ctx.check(
        "bowl slide moves forward",
        bowl_rest is not None
        and bowl_extended is not None
        and bowl_extended[0] > bowl_rest[0] + 0.020,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )

    tool_rest = ctx.part_world_position(tool)
    with ctx.pose({head_hinge: head_hinge.motion_limits.upper}):
        tool_tilted = ctx.part_world_position(tool)
    ctx.check(
        "tilt head raises dough hook",
        tool_rest is not None
        and tool_tilted is not None
        and tool_tilted[2] > tool_rest[2] + 0.080,
        details=f"rest={tool_rest}, tilted={tool_tilted}",
    )

    selector_rest = aabb_center(ctx.part_element_world_aabb(selector, elem="selector_handle"))
    with ctx.pose({selector_joint: selector_joint.motion_limits.upper}):
        selector_turn = aabb_center(ctx.part_element_world_aabb(selector, elem="selector_handle"))
    ctx.check(
        "speed selector handle rotates through a visible arc",
        selector_rest is not None
        and selector_turn is not None
        and abs(selector_turn[2] - selector_rest[2]) > 0.010,
        details=f"rest={selector_rest}, turned={selector_turn}",
    )

    lock_rest = ctx.part_world_position(head_lock)
    with ctx.pose({lock_slide: lock_slide.motion_limits.upper}):
        lock_extended = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock pin slides outward",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[1] > lock_rest[1] + 0.008,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    tool_limits = tool_spin.motion_limits
    ctx.check(
        "tool spin remains continuous",
        tool_limits is not None and tool_limits.lower is None and tool_limits.upper is None,
        details=f"limits={tool_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
