from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_floor_lamp")

    frame_black = model.material("frame_black", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    warm_cream = model.material("warm_cream", rgba=(0.87, 0.84, 0.77, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.66, 0.57, 0.36, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.92, 0.89, 0.78, 0.92))

    frame = model.part("frame")
    crown_z = 0.350
    hinge_radius = 0.062

    frame.visual(
        Cylinder(radius=0.032, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, crown_z)),
        material=frame_black,
        name="crown",
    )
    frame.visual(
        Cylinder(radius=0.021, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        material=frame_black,
        name="lower_collar",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.750),
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        material=frame_black,
        name="center_post",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 1.205)),
        material=frame_black,
        name="head_socket",
    )
    yoke = TrunnionYokeGeometry(
        (0.128, 0.050, 0.095),
        span_width=0.074,
        trunnion_diameter=0.010,
        trunnion_center_z=0.062,
        base_thickness=0.014,
        corner_radius=0.004,
        center=False,
    )
    frame.visual(
        mesh_from_geometry(yoke, "lamp_head_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 1.245)),
        material=frame_black,
        name="head_yoke",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        frame.visual(
            Box((0.020, 0.022, 0.028)),
            origin=Origin(
                xyz=(0.042 * math.cos(angle), 0.042 * math.sin(angle), crown_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=frame_black,
            name=f"crown_lug_{index}",
        )

    shade = model.part("shade")
    rest_shade_roll = -math.pi / 2.0 - 0.22
    shade_shell = LatheGeometry.from_shell_profiles(
        [
            (0.014, -0.060),
            (0.070, -0.050),
            (0.125, -0.018),
            (0.154, 0.024),
            (0.160, 0.052),
        ],
        [
            (0.0, -0.054),
            (0.062, -0.045),
            (0.117, -0.016),
            (0.146, 0.024),
            (0.152, 0.047),
        ],
        segments=64,
        lip_samples=10,
    )
    shade.visual(
        mesh_from_geometry(shade_shell, "lamp_shade_shell"),
        origin=Origin(xyz=(0.0, 0.120, -0.018), rpy=(rest_shade_roll, 0.0, 0.0)),
        material=warm_cream,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.020, length=0.070),
        origin=Origin(xyz=(0.0, 0.045, -0.014), rpy=(rest_shade_roll, 0.0, 0.0)),
        material=satin_brass,
        name="shade_neck",
    )
    shade.visual(
        Box((0.038, 0.032, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=frame_black,
        name="mount_block",
    )
    shade.visual(
        Cylinder(radius=0.0045, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_brass,
        name="trunnion_pin",
    )
    shade.visual(
        Cylinder(radius=0.016, length=0.048),
        origin=Origin(xyz=(0.0, 0.092, -0.026), rpy=(rest_shade_roll, 0.0, 0.0)),
        material=satin_brass,
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.029),
        origin=Origin(xyz=(0.0, 0.122, -0.033)),
        material=warm_glass,
        name="bulb",
    )

    model.articulation(
        "frame_to_shade",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, 1.312)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-0.45,
            upper=0.60,
        ),
    )

    leg_length = 0.445
    leg_radius = 0.011
    foot_radius = 0.014
    leg_spread = math.radians(40.0)
    leg_direction = (
        leg_length * math.sin(leg_spread),
        0.0,
        -leg_length * math.cos(leg_spread),
    )
    leg_axis_pitch = math.pi - leg_spread

    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.010, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=frame_black,
            name="hinge_barrel",
        )
        leg.visual(
            Cylinder(radius=leg_radius, length=leg_length),
            origin=Origin(
                xyz=(0.5 * leg_direction[0], 0.0, 0.5 * leg_direction[2]),
                rpy=(0.0, leg_axis_pitch, 0.0),
            ),
            material=frame_black,
            name="leg_tube",
        )
        leg.visual(
            Sphere(radius=foot_radius),
            origin=Origin(xyz=leg_direction),
            material=rubber_black,
            name="foot",
        )

        model.articulation(
            f"frame_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=leg,
            origin=Origin(
                xyz=(
                    hinge_radius * math.cos(angle),
                    hinge_radius * math.sin(angle),
                    crown_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=1.5,
                lower=-1.55,
                upper=0.12,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    shade = object_model.get_part("shade")
    leg_0 = object_model.get_part("leg_0")
    leg_1 = object_model.get_part("leg_1")
    leg_2 = object_model.get_part("leg_2")
    shade_tilt = object_model.get_articulation("frame_to_shade")
    leg_fold = object_model.get_articulation("frame_to_leg_0")

    ctx.allow_overlap(
        frame,
        shade,
        elem_a="head_yoke",
        elem_b="trunnion_pin",
        reason="The shade pivots on a trunnion pin intentionally nested inside the head yoke support.",
    )
    ctx.expect_within(
        shade,
        frame,
        axes="yz",
        inner_elem="trunnion_pin",
        outer_elem="head_yoke",
        margin=0.001,
        name="trunnion pin stays captured by the head yoke",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    foot_centers = []
    for leg in (leg_0, leg_1, leg_2):
        foot_aabb = ctx.part_element_world_aabb(leg, elem="foot")
        foot_center = aabb_center(foot_aabb)
        if foot_center is not None:
            foot_centers.append(foot_center)

    ctx.check(
        "tripod feet sit at a common height",
        len(foot_centers) == 3
        and max(center[2] for center in foot_centers) - min(center[2] for center in foot_centers) <= 0.01,
        details=f"foot_centers={foot_centers}",
    )

    rest_bulb = aabb_center(ctx.part_element_world_aabb(shade, elem="bulb"))
    with ctx.pose({shade_tilt: 0.50}):
        raised_bulb = aabb_center(ctx.part_element_world_aabb(shade, elem="bulb"))
    with ctx.pose({shade_tilt: -0.35}):
        lowered_bulb = aabb_center(ctx.part_element_world_aabb(shade, elem="bulb"))

    ctx.check(
        "shade tilt raises and lowers the beam aim",
        rest_bulb is not None
        and raised_bulb is not None
        and lowered_bulb is not None
        and raised_bulb[2] > rest_bulb[2] + 0.02
        and lowered_bulb[2] < rest_bulb[2] - 0.02,
        details=f"rest={rest_bulb}, raised={raised_bulb}, lowered={lowered_bulb}",
    )

    rest_foot = aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot"))
    with ctx.pose({leg_fold: -1.35}):
        folded_foot = aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot"))

    ctx.check(
        "leg folds upward at the crown hinge",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[2] > rest_foot[2] + 0.20,
        details=f"rest={rest_foot}, folded={folded_foot}",
    )

    return ctx.report()


object_model = build_object_model()
