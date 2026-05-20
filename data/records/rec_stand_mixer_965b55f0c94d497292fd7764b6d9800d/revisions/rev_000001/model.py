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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_apartment_stand_mixer")

    body_finish = model.material("body_finish", rgba=(0.82, 0.83, 0.80, 1.0))
    metal = model.material("metal", rgba=(0.88, 0.89, 0.90, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))

    base = model.part("base")

    foot_geom = ExtrudeGeometry(rounded_rect_profile(0.24, 0.155, 0.038), 0.022)
    base.visual(
        mesh_from_geometry(foot_geom, "base_foot"),
        origin=Origin(xyz=(0.015, 0.0, 0.011)),
        material=body_finish,
        name="foot_shell",
    )

    guide_geom = ExtrudeGeometry(rounded_rect_profile(0.130, 0.090, 0.020), 0.008)
    base.visual(
        mesh_from_geometry(guide_geom, "bowl_guide"),
        origin=Origin(xyz=(0.072, 0.0, 0.026)),
        material=body_finish,
        name="guide_deck",
    )

    def xy_section(width: float, depth: float, radius: float, z: float, x_shift: float) -> list[tuple[float, float, float]]:
        return [(x + x_shift, y, z) for x, y in rounded_rect_profile(width, depth, radius)]

    pedestal_sections = [
        xy_section(0.122, 0.116, 0.030, 0.000, -0.040),
        xy_section(0.108, 0.106, 0.027, 0.070, -0.048),
        xy_section(0.094, 0.098, 0.024, 0.138, -0.058),
        xy_section(0.082, 0.092, 0.022, 0.176, -0.070),
    ]
    pedestal_geom = section_loft(pedestal_sections)
    base.visual(
        mesh_from_geometry(pedestal_geom, "pedestal_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=body_finish,
        name="pedestal_shell",
    )

    base.visual(
        Cylinder(radius=0.014, length=0.086),
        origin=Origin(
            xyz=(-0.074, 0.0, 0.176),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="hinge_barrel",
    )
    base.visual(
        Box((0.030, 0.086, 0.026)),
        origin=Origin(xyz=(-0.072, 0.0, 0.185)),
        material=body_finish,
        name="hinge_mount",
    )
    base.visual(
        Box((0.022, 0.016, 0.024)),
        origin=Origin(xyz=(-0.030, -0.052, 0.118)),
        material=body_finish,
        name="speed_control_pod",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.24, 0.16, 0.20)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        Box((0.108, 0.070, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_trim,
        name="carriage_shoe",
    )
    bowl.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=body_finish,
        name="bowl_pedestal",
    )

    bowl_outer = [
        (0.020, 0.000),
        (0.044, 0.008),
        (0.063, 0.040),
        (0.070, 0.072),
        (0.076, 0.085),
    ]
    bowl_inner = [
        (0.000, 0.003),
        (0.039, 0.010),
        (0.057, 0.040),
        (0.066, 0.079),
    ]
    bowl_geom = LatheGeometry.from_shell_profiles(
        bowl_outer,
        bowl_inner,
        segments=56,
        start_cap="flat",
        end_cap="round",
    )
    bowl.visual(
        mesh_from_geometry(bowl_geom, "mixing_bowl_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=metal,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.076, length=0.090),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.072, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.10,
            lower=0.0,
            upper=0.035,
        ),
    )

    head = model.part("head")

    def yz_section(
        width: float,
        height: float,
        radius: float,
        x: float,
        z_offset: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_offset) for z, y in rounded_rect_profile(height, width, radius)]

    head_sections = [
        yz_section(0.078, 0.082, 0.024, 0.075, 0.010),
        yz_section(0.132, 0.122, 0.040, 0.140, 0.002),
        yz_section(0.110, 0.108, 0.034, 0.215, -0.004),
        yz_section(0.074, 0.084, 0.024, 0.280, -0.010),
    ]
    head_geom = section_loft(head_sections)
    head.visual(
        mesh_from_geometry(head_geom, "head_shell"),
        material=body_finish,
        name="head_shell",
    )
    head.visual(
        Box((0.064, 0.080, 0.028)),
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        material=body_finish,
        name="rear_neck",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(0.150, 0.0, -0.049)),
        material=dark_trim,
        name="hub_collar",
    )
    head.visual(
        Box((0.090, 0.068, 0.044)),
        origin=Origin(xyz=(0.148, 0.0, -0.034)),
        material=body_finish,
        name="hub_mount",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.25, 0.14, 0.14)),
        mass=3.4,
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.074, 0.0, 0.212)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    beater = model.part("beater")
    beater.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=metal,
        name="drive_cap",
    )
    beater.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=metal,
        name="shaft",
    )
    beater.visual(
        Box((0.010, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=metal,
        name="shaft_link",
    )
    beater.visual(
        Box((0.024, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=metal,
        name="upper_yoke",
    )
    beater.visual(
        Box((0.006, 0.032, 0.040)),
        origin=Origin(xyz=(-0.010, 0.0, -0.068)),
        material=metal,
        name="left_blade",
    )
    beater.visual(
        Box((0.006, 0.032, 0.040)),
        origin=Origin(xyz=(0.010, 0.0, -0.068)),
        material=metal,
        name="right_blade",
    )
    beater.visual(
        Box((0.028, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.013, -0.085)),
        material=metal,
        name="lower_crossbar",
    )
    beater.inertial = Inertial.from_geometry(
        Box((0.032, 0.038, 0.105)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
    )

    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.150, 0.0, -0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="lever_pivot",
    )
    speed_control.visual(
        Box((0.036, 0.008, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.012)),
        material=dark_trim,
        name="lever_arm",
    )
    speed_control.visual(
        Box((0.014, 0.014, 0.010)),
        origin=Origin(xyz=(0.036, 0.0, 0.021)),
        material=dark_trim,
        name="lever_grip",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.050, 0.018, 0.034)),
        mass=0.03,
        origin=Origin(xyz=(0.022, 0.0, 0.014)),
    )

    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.030, -0.066, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=-0.35,
            upper=0.35,
        ),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.020, 0.018, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.003)),
        material=dark_trim,
        name="lock_slider",
    )
    head_lock.visual(
        Box((0.014, 0.022, 0.010)),
        origin=Origin(xyz=(0.013, 0.0, 0.011)),
        material=dark_trim,
        name="lock_button",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.026, 0.022, 0.016)),
        mass=0.02,
        origin=Origin(xyz=(0.012, 0.0, 0.008)),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.097, 0.0, 0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.05,
            lower=0.0,
            upper=0.008,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl")
    head_tilt = object_model.get_articulation("base_to_head")
    speed_joint = object_model.get_articulation("base_to_speed_control")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="carriage_shoe",
        negative_elem="guide_deck",
        max_gap=0.0015,
        max_penetration=0.0,
        name="bowl carriage rests on guide deck",
    )
    ctx.expect_within(
        beater,
        bowl,
        axes="xy",
        inner_elem="upper_yoke",
        outer_elem="bowl_shell",
        margin=0.020,
        name="beater drive stays centered over the bowl",
    )
    ctx.expect_overlap(
        beater,
        bowl,
        axes="xy",
        elem_a="left_blade",
        elem_b="bowl_shell",
        min_overlap=0.003,
        name="beater reaches into the bowl footprint",
    )

    rest_bowl_pos = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        extended_bowl_pos = ctx.part_world_position(bowl)
    ctx.check(
        "bowl carriage slides forward",
        rest_bowl_pos is not None
        and extended_bowl_pos is not None
        and extended_bowl_pos[0] > rest_bowl_pos[0] + 0.020,
        details=f"rest={rest_bowl_pos}, extended={extended_bowl_pos}",
    )

    rest_beater_pos = ctx.part_world_position(beater)
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        raised_beater_pos = ctx.part_world_position(beater)
        ctx.expect_gap(
            beater,
            bowl,
            axis="z",
            positive_elem="lower_crossbar",
            negative_elem="bowl_shell",
            min_gap=0.015,
            name="tilted head lifts the beater above the bowl rim",
        )
    ctx.check(
        "head tilts upward",
        rest_beater_pos is not None
        and raised_beater_pos is not None
        and raised_beater_pos[2] > rest_beater_pos[2] + 0.040,
        details=f"rest={rest_beater_pos}, raised={raised_beater_pos}",
    )

    def elem_center(part_obj, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    rest_lever_pos = elem_center(speed_control, "lever_grip")
    with ctx.pose({speed_joint: speed_joint.motion_limits.upper}):
        raised_lever_pos = elem_center(speed_control, "lever_grip")
    ctx.check(
        "speed control lever swings through an arc",
        rest_lever_pos is not None
        and raised_lever_pos is not None
        and math.dist(rest_lever_pos, raised_lever_pos) > 0.010,
        details=f"rest={rest_lever_pos}, raised={raised_lever_pos}",
    )

    rest_lock_pos = ctx.part_world_position(head_lock)
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        pressed_lock_pos = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock push control slides forward",
        rest_lock_pos is not None
        and pressed_lock_pos is not None
        and pressed_lock_pos[0] > rest_lock_pos[0] + 0.004,
        details=f"rest={rest_lock_pos}, pressed={pressed_lock_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
