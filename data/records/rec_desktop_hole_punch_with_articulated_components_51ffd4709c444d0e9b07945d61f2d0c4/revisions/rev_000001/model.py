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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_hole_punch")

    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.168, 0.104, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="tray_floor",
    )
    base.visual(
        Box((0.156, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.050, 0.010)),
        material=steel,
        name="side_flange_0",
    )
    base.visual(
        Box((0.156, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.050, 0.010)),
        material=steel,
        name="side_flange_1",
    )
    base.visual(
        Box((0.018, 0.092, 0.018)),
        origin=Origin(xyz=(0.074, 0.0, 0.010)),
        material=steel,
        name="front_lip",
    )
    base.visual(
        Box((0.042, 0.092, 0.030)),
        origin=Origin(xyz=(-0.044, 0.0, 0.018)),
        material=dark_steel,
        name="rear_block",
    )
    base.visual(
        Box((0.020, 0.092, 0.020)),
        origin=Origin(xyz=(-0.018, 0.0, 0.035)),
        material=dark_steel,
        name="punch_head",
    )
    base.visual(
        Box((0.012, 0.012, 0.028)),
        origin=Origin(xyz=(0.035, 0.036, 0.017)),
        material=dark_steel,
        name="guide_post_0",
    )
    base.visual(
        Box((0.012, 0.012, 0.028)),
        origin=Origin(xyz=(0.035, -0.036, 0.017)),
        material=dark_steel,
        name="guide_post_1",
    )
    base.visual(
        Cylinder(radius=0.004, length=0.084),
        origin=Origin(xyz=(0.035, 0.0, 0.021), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="guide_rod",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(-0.068, 0.040, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_ear_0",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(-0.068, -0.040, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_ear_1",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.168, 0.104, 0.050)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    handle = model.part("handle")
    def yz_section(x: float, width: float, thickness: float, z_center: float) -> list[tuple[float, float, float]]:
        return [
            (x, y, z_center + z)
            for z, y in rounded_rect_profile(thickness, width, radius=thickness * 0.45, corner_segments=8)
        ]

    handle_geom = section_loft(
        [
            yz_section(0.004, 0.090, 0.010, 0.018),
            yz_section(0.016, 0.092, 0.011, 0.040),
            yz_section(0.046, 0.094, 0.012, 0.056),
            yz_section(0.080, 0.092, 0.011, 0.037),
            yz_section(0.097, 0.088, 0.010, 0.020),
        ]
    )
    handle.visual(
        mesh_from_geometry(handle_geom, "handle_shell"),
        material=steel,
        name="top_shell",
    )
    handle.visual(
        Cylinder(radius=0.0055, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_tube",
    )
    handle.visual(
        Box((0.014, 0.018, 0.020)),
        origin=Origin(xyz=(0.010, 0.029, 0.015)),
        material=dark_steel,
        name="hinge_cheek_0",
    )
    handle.visual(
        Box((0.014, 0.018, 0.020)),
        origin=Origin(xyz=(0.010, -0.029, 0.015)),
        material=dark_steel,
        name="hinge_cheek_1",
    )
    handle.visual(
        Box((0.052, 0.010, 0.036)),
        origin=Origin(xyz=(0.036, 0.032, 0.031)),
        material=dark_steel,
        name="arm_0",
    )
    handle.visual(
        Box((0.052, 0.010, 0.036)),
        origin=Origin(xyz=(0.036, -0.032, 0.031)),
        material=dark_steel,
        name="arm_1",
    )
    handle.visual(
        Box((0.020, 0.090, 0.016)),
        origin=Origin(xyz=(0.052, 0.0, 0.022)),
        material=dark_steel,
        name="press_bar",
    )
    handle.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.050, 0.040, 0.015)),
        material=dark_steel,
        name="punch_pin_0",
    )
    handle.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.050, -0.040, 0.015)),
        material=dark_steel,
        name="punch_pin_1",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.104, 0.094, 0.072)),
        mass=0.40,
        origin=Origin(xyz=(0.052, 0.0, 0.026)),
    )

    stop = model.part("stop")
    stop.visual(
        Box((0.016, 0.020, 0.006)),
        origin=Origin(xyz=(-0.002, 0.0, 0.007)),
        material=dark_steel,
        name="carriage_upper",
    )
    stop.visual(
        Box((0.016, 0.020, 0.004)),
        origin=Origin(xyz=(-0.002, 0.0, -0.007)),
        material=dark_steel,
        name="carriage_lower",
    )
    stop.visual(
        Box((0.004, 0.020, 0.009)),
        origin=Origin(xyz=(-0.008, 0.0, -0.0005)),
        material=dark_steel,
        name="carriage_back",
    )
    stop.visual(
        Box((0.024, 0.014, 0.006)),
        origin=Origin(xyz=(0.012, 0.0, 0.009)),
        material=dark_steel,
        name="arm",
    )
    stop.visual(
        Box((0.010, 0.088, 0.016)),
        origin=Origin(xyz=(0.024, 0.0, 0.004)),
        material=steel,
        name="fence",
    )
    stop.visual(
        Box((0.010, 0.010, 0.014)),
        origin=Origin(xyz=(0.020, 0.045, -0.001)),
        material=dark_steel,
        name="knob_lug",
    )
    stop.inertial = Inertial.from_geometry(
        Box((0.040, 0.088, 0.020)),
        mass=0.10,
        origin=Origin(xyz=(0.010, 0.0, 0.000)),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.0025, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="knob_body",
    )
    knob.visual(
        Box((0.009, 0.003, 0.004)),
        origin=Origin(xyz=(0.008, 0.010, 0.0)),
        material=black_plastic,
        name="pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.018, 0.012, 0.016)),
        mass=0.02,
        origin=Origin(xyz=(0.004, 0.008, 0.0)),
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.068, 0.0, 0.038)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    model.articulation(
        "base_to_stop",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stop,
        origin=Origin(xyz=(0.035, 0.0, 0.021)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.08,
            lower=-0.018,
            upper=0.018,
        ),
    )

    model.articulation(
        "stop_to_knob",
        ArticulationType.CONTINUOUS,
        parent=stop,
        child=knob,
        origin=Origin(xyz=(-0.010, 0.045, -0.001)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    stop = object_model.get_part("stop")
    knob = object_model.get_part("knob")
    handle_joint = object_model.get_articulation("base_to_handle")
    stop_joint = object_model.get_articulation("base_to_stop")
    knob_joint = object_model.get_articulation("stop_to_knob")

    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_overlap(
            "handle",
            base,
            axes="xy",
            min_overlap=0.050,
            name="handle covers punch body in closed pose",
        )
        ctx.expect_gap(
            "handle",
            base,
            axis="z",
            positive_elem="punch_pin_0",
            negative_elem="tray_floor",
            min_gap=0.039,
            max_gap=0.050,
            name="punch pins remain above tray floor in closed pose",
        )

    with ctx.pose({handle_joint: 0.0}):
        rest_bar = ctx.part_element_world_aabb("handle", elem="top_shell")
    with ctx.pose({handle_joint: math.radians(55.0)}):
        raised_bar = ctx.part_element_world_aabb("handle", elem="top_shell")

    ctx.check(
        "handle opens upward",
        rest_bar is not None
        and raised_bar is not None
        and raised_bar[1][2] > rest_bar[1][2] + 0.045,
        details=f"rest={rest_bar}, raised={raised_bar}",
    )

    with ctx.pose({stop_joint: -0.018}):
        stop_low = ctx.part_world_position(stop)
    with ctx.pose({stop_joint: 0.018}):
        stop_high = ctx.part_world_position(stop)

    ctx.check(
        "paper stop slides across front guide",
        stop_low is not None
        and stop_high is not None
        and abs(stop_high[0] - stop_low[0]) < 0.001
        and abs(stop_high[2] - stop_low[2]) < 0.001
        and stop_high[1] > stop_low[1] + 0.030,
        details=f"low={stop_low}, high={stop_high}",
    )

    with ctx.pose({knob_joint: 0.0}):
        pointer_rest = ctx.part_element_world_aabb(knob, elem="pointer")
    with ctx.pose({knob_joint: 1.2}):
        pointer_turned = ctx.part_element_world_aabb(knob, elem="pointer")

    ctx.check(
        "guide knob rotates on side shaft",
        pointer_rest is not None
        and pointer_turned is not None
        and abs(((pointer_turned[0][0] + pointer_turned[1][0]) * 0.5) - ((pointer_rest[0][0] + pointer_rest[1][0]) * 0.5))
        > 0.003
        and abs(((pointer_turned[0][1] + pointer_turned[1][1]) * 0.5) - ((pointer_rest[0][1] + pointer_rest[1][1]) * 0.5))
        < 0.0015,
        details=f"rest={pointer_rest}, turned={pointer_turned}, axis={knob_joint.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
