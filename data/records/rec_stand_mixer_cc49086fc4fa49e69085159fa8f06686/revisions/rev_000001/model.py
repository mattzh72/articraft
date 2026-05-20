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
    ExtrudeGeometry,
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_countertop_stand_mixer")

    body_finish = model.material("body_finish", rgba=(0.86, 0.86, 0.84, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.16, 0.18, 1.0))

    def yz_section(
        x: float,
        width: float,
        height: float,
        radius: float,
        *,
        z_base: float = 0.0,
        z_center: float | None = None,
    ) -> list[tuple[float, float, float]]:
        if z_center is None:
            z_shift = z_base + height / 2.0
        else:
            z_shift = z_center
        return [(x, y, z + z_shift) for y, z in rounded_rect_profile(width, height, radius)]

    base = model.part("base")

    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.36, 0.24, 0.055), 0.036),
        "base_plate",
    )
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=body_finish,
        name="base_plate",
    )

    pedestal_mesh = mesh_from_geometry(
        section_loft(
            [
                yz_section(-0.03, 0.13, 0.11, 0.030, z_base=0.0),
                yz_section(0.01, 0.11, 0.19, 0.028, z_base=0.0),
                yz_section(0.05, 0.09, 0.29, 0.024, z_base=0.0),
                yz_section(0.075, 0.075, 0.31, 0.022, z_base=0.0),
            ]
        ),
        "rear_pedestal",
    )
    base.visual(
        pedestal_mesh,
        origin=Origin(xyz=(-0.14, 0.0, 0.036)),
        material=body_finish,
        name="rear_pedestal",
    )
    base.visual(
        Box((0.18, 0.022, 0.014)),
        origin=Origin(xyz=(0.09, 0.045, 0.043)),
        material=dark_trim,
        name="right_slide_rail",
    )
    base.visual(
        Box((0.18, 0.022, 0.014)),
        origin=Origin(xyz=(0.09, -0.045, 0.043)),
        material=dark_trim,
        name="left_slide_rail",
    )
    base.visual(
        Box((0.040, 0.028, 0.002)),
        origin=Origin(xyz=(-0.108, 0.0, 0.319)),
        material=dark_trim,
        name="head_lock_track",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.24, 0.32)),
        mass=9.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.16)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.12, 0.058, 0.014)),
        material=dark_trim,
        name="runner",
    )
    carriage.visual(
        Box((0.18, 0.14, 0.014)),
        origin=Origin(xyz=(0.01, 0.0, 0.014)),
        material=body_finish,
        name="carriage_platform",
    )
    carriage.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=dark_trim,
        name="bowl_seat",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 0.050)),
        mass=0.9,
        origin=Origin(xyz=(0.01, 0.0, 0.018)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.07, 0.0, 0.043)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.10, lower=0.0, upper=0.040),
    )

    bowl = model.part("bowl")
    bowl_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.026, 0.000),
                (0.052, 0.004),
                (0.085, 0.022),
                (0.108, 0.082),
                (0.116, 0.150),
                (0.120, 0.164),
            ],
            [
                (0.000, 0.004),
                (0.046, 0.010),
                (0.078, 0.025),
                (0.098, 0.084),
                (0.108, 0.156),
            ],
            segments=64,
            end_cap="round",
            lip_samples=8,
        ),
        "mixing_bowl",
    )
    bowl.visual(bowl_mesh, material=steel, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=0.164),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    head = model.part("head")
    head_mesh = mesh_from_geometry(
        section_loft(
            [
                yz_section(0.000, 0.105, 0.115, 0.030, z_center=0.003),
                yz_section(0.085, 0.150, 0.145, 0.040, z_center=0.004),
                yz_section(0.180, 0.165, 0.150, 0.042, z_center=0.000),
                yz_section(0.270, 0.120, 0.108, 0.030, z_center=-0.008),
                yz_section(0.315, 0.072, 0.074, 0.020, z_center=-0.016),
            ]
        ),
        "head_shell",
    )
    head.visual(head_mesh, material=body_finish, name="head_shell")
    head.inertial = Inertial.from_geometry(
        Box((0.33, 0.17, 0.16)),
        mass=4.8,
        origin=Origin(xyz=(0.165, 0.0, -0.020)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.07, 0.0, 0.335)),
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
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_trim,
        name="selector_skirt",
    )
    speed_selector.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_trim,
        name="selector_knob",
    )
    speed_selector.visual(
        Box((0.024, 0.006, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, 0.018)),
        material=steel,
        name="selector_pointer",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.036, 0.036, 0.022)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(0.108, 0.102, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(140.0),
        ),
    )

    head_lock_control = model.part("head_lock_control")
    head_lock_control.visual(
        Box((0.030, 0.022, 0.008)),
        origin=Origin(xyz=(0.015, 0.0, 0.004)),
        material=dark_trim,
        name="lock_tab",
    )
    head_lock_control.visual(
        Box((0.020, 0.010, 0.008)),
        origin=Origin(xyz=(0.006, 0.0, 0.010)),
        material=steel,
        name="lock_thumb",
    )
    head_lock_control.inertial = Inertial.from_geometry(
        Box((0.030, 0.022, 0.014)),
        mass=0.06,
        origin=Origin(xyz=(0.014, 0.0, 0.006)),
    )
    model.articulation(
        "base_to_head_lock_control",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock_control,
        origin=Origin(xyz=(-0.123, 0.0, 0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.012,
        ),
    )

    tool = model.part("tool")
    dough_hook_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.000, -0.014),
                (0.000, 0.000, -0.044),
                (0.012, 0.000, -0.064),
                (0.029, 0.000, -0.098),
                (0.025, 0.000, -0.130),
                (0.004, 0.000, -0.160),
                (-0.017, 0.000, -0.140),
                (-0.008, 0.000, -0.106),
            ],
            radius=0.008,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "dough_hook",
    )
    tool.visual(dough_hook_mesh, material=steel, name="dough_hook")
    tool.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=0.15),
        mass=0.4,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
    )

    model.articulation(
        "head_to_tool",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=tool,
        origin=Origin(xyz=(0.140, 0.0, -0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("head")
    speed_selector = object_model.get_part("speed_selector")
    head_lock_control = object_model.get_part("head_lock_control")
    tool = object_model.get_part("tool")

    bowl_slide = object_model.get_articulation("base_to_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    selector_turn = object_model.get_articulation("base_to_speed_selector")
    head_lock_slide = object_model.get_articulation("base_to_head_lock_control")
    tool_spin = object_model.get_articulation("head_to_tool")

    ctx.expect_origin_distance(
        bowl,
        tool,
        axes="xy",
        max_dist=0.005,
        name="tool axis stays centered over the bowl center at rest",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xy",
        min_overlap=0.050,
        name="carriage remains supported on the base footprint",
    )

    rest_bowl_pos = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: 0.040}):
        extended_bowl_pos = ctx.part_world_position(bowl)
    ctx.check(
        "bowl carriage slides forward",
        rest_bowl_pos is not None
        and extended_bowl_pos is not None
        and extended_bowl_pos[0] > rest_bowl_pos[0] + 0.030,
        details=f"rest={rest_bowl_pos}, extended={extended_bowl_pos}",
    )

    rest_tool_pos = ctx.part_world_position(tool)
    with ctx.pose({head_tilt: math.radians(55.0)}):
        opened_tool_pos = ctx.part_world_position(tool)
        ctx.expect_gap(
            tool,
            bowl,
            axis="z",
            min_gap=0.025,
            name="tilted head lifts the dough hook above the bowl rim",
        )
    ctx.check(
        "head tilts upward around the rear hinge",
        rest_tool_pos is not None
        and opened_tool_pos is not None
        and opened_tool_pos[2] > rest_tool_pos[2] + 0.030,
        details=f"rest={rest_tool_pos}, opened={opened_tool_pos}",
    )

    rest_selector_aabb = ctx.part_element_world_aabb(speed_selector, elem="selector_pointer")
    with ctx.pose({selector_turn: math.radians(90.0)}):
        turned_selector_aabb = ctx.part_element_world_aabb(speed_selector, elem="selector_pointer")
    rest_selector_center = (
        None
        if rest_selector_aabb is None
        else tuple((rest_selector_aabb[0][i] + rest_selector_aabb[1][i]) * 0.5 for i in range(3))
    )
    turned_selector_center = (
        None
        if turned_selector_aabb is None
        else tuple((turned_selector_aabb[0][i] + turned_selector_aabb[1][i]) * 0.5 for i in range(3))
    )
    ctx.check(
        "speed selector rotates about its dial axis",
        rest_selector_center is not None
        and turned_selector_center is not None
        and turned_selector_center[0] < rest_selector_center[0] - 0.006
        and turned_selector_center[1] > rest_selector_center[1] + 0.006,
        details=f"rest={rest_selector_center}, turned={turned_selector_center}",
    )

    rest_lock_pos = ctx.part_world_position(head_lock_control)
    with ctx.pose({head_lock_slide: 0.012}):
        extended_lock_pos = ctx.part_world_position(head_lock_control)
    ctx.check(
        "head lock control slides fore-aft on the base",
        rest_lock_pos is not None
        and extended_lock_pos is not None
        and extended_lock_pos[0] > rest_lock_pos[0] + 0.009,
        details=f"rest={rest_lock_pos}, extended={extended_lock_pos}",
    )

    rest_tool_aabb = ctx.part_world_aabb(tool)
    with ctx.pose({tool_spin: math.pi / 2.0}):
        spun_tool_aabb = ctx.part_world_aabb(tool)
    rest_tool_span = (
        None
        if rest_tool_aabb is None
        else tuple(rest_tool_aabb[1][i] - rest_tool_aabb[0][i] for i in range(3))
    )
    spun_tool_span = (
        None
        if spun_tool_aabb is None
        else tuple(spun_tool_aabb[1][i] - spun_tool_aabb[0][i] for i in range(3))
    )
    ctx.check(
        "dough hook rotates around the vertical mixing axis",
        rest_tool_span is not None
        and spun_tool_span is not None
        and spun_tool_span[0] < rest_tool_span[0] - 0.020
        and spun_tool_span[1] > rest_tool_span[1] + 0.020,
        details=f"rest={rest_tool_span}, spun={spun_tool_span}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
