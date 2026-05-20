from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_telescoping_work_light")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.016, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.10, 0.105, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    white_nylon = model.material("white_nylon", rgba=(0.86, 0.86, 0.80, 1.0))
    warm_glass = model.material("warm_glass", rgba=(1.0, 0.86, 0.38, 0.55))
    led_yellow = model.material("led_yellow", rgba=(1.0, 0.72, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.30, 0.20, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_grey,
        name="weighted_base",
    )
    base.visual(
        Box((0.24, 0.15, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=rubber,
        name="top_pad",
    )
    base.visual(
        Box((0.080, 0.136, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=matte_black,
        name="yoke_bridge",
    )
    for side, y in (("side_0", -0.055), ("side_1", 0.055)):
        base.visual(
            Box((0.070, 0.014, 0.112)),
            origin=Origin(xyz=(0.0, y, 0.096)),
            material=matte_black,
            name=f"yoke_cheek_{side}",
        )
        base.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(
                xyz=(0.0, y + (0.009 if y > 0.0 else -0.009), 0.115),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_steel,
            name=f"hinge_washer_{side}",
        )
    base.visual(
        Cylinder(radius=0.0095, length=0.148),
        origin=Origin(xyz=(0.0, 0.0, 0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_pin",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.020,
            body_style="lobed",
            grip=KnobGrip(style="fluted", count=12, depth=0.002),
        ),
        "tilt_lock_knob",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.0, 0.074, 0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="knob_shaft",
    )
    base.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.082, 0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="tilt_lock_knob",
    )

    arm_sleeve = model.part("arm_sleeve")
    barrel_mesh = LatheGeometry(
        [(0.009, -0.041), (0.028, -0.041), (0.028, 0.041), (0.009, 0.041)],
        segments=64,
        closed=True,
    ).rotate_x(-math.pi / 2.0)
    arm_sleeve.visual(
        mesh_from_geometry(barrel_mesh, "hollow_tilt_barrel"),
        origin=Origin(),
        material=matte_black,
        name="tilt_barrel",
    )
    arm_sleeve.visual(
        Box((0.470, 0.052, 0.008)),
        origin=Origin(xyz=(0.255, 0.0, 0.016)),
        material=matte_black,
        name="sleeve_top_wall",
    )
    arm_sleeve.visual(
        Box((0.470, 0.052, 0.008)),
        origin=Origin(xyz=(0.255, 0.0, -0.016)),
        material=matte_black,
        name="sleeve_bottom_wall",
    )
    arm_sleeve.visual(
        Box((0.470, 0.008, 0.040)),
        origin=Origin(xyz=(0.255, -0.022, 0.0)),
        material=matte_black,
        name="sleeve_side_wall_0",
    )
    arm_sleeve.visual(
        Box((0.470, 0.008, 0.040)),
        origin=Origin(xyz=(0.255, 0.022, 0.0)),
        material=matte_black,
        name="sleeve_side_wall_1",
    )
    arm_sleeve.visual(
        Box((0.030, 0.060, 0.008)),
        origin=Origin(xyz=(0.490, 0.0, 0.020)),
        material=dark_grey,
        name="front_collar_top",
    )
    arm_sleeve.visual(
        Box((0.030, 0.060, 0.008)),
        origin=Origin(xyz=(0.490, 0.0, -0.020)),
        material=dark_grey,
        name="front_collar_bottom",
    )
    arm_sleeve.visual(
        Box((0.030, 0.008, 0.048)),
        origin=Origin(xyz=(0.490, -0.026, 0.0)),
        material=dark_grey,
        name="front_collar_side_0",
    )
    arm_sleeve.visual(
        Box((0.030, 0.008, 0.048)),
        origin=Origin(xyz=(0.490, 0.026, 0.0)),
        material=dark_grey,
        name="front_collar_side_1",
    )

    extension = model.part("extension")
    extension.visual(
        Box((0.650, 0.030, 0.020)),
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        material=brushed_steel,
        name="inner_tube",
    )
    extension.visual(
        Box((0.300, 0.024, 0.004)),
        origin=Origin(xyz=(-0.190, 0.0, 0.011)),
        material=white_nylon,
        name="top_glide",
    )
    extension.visual(
        Box((0.300, 0.024, 0.004)),
        origin=Origin(xyz=(-0.190, 0.0, -0.011)),
        material=white_nylon,
        name="bottom_glide",
    )
    extension.visual(
        Box((0.090, 0.036, 0.026)),
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        material=brushed_steel,
        name="lamp_neck",
    )
    extension.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(0.370, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="rear_socket",
    )

    shade_profile = [
        (0.044, -0.056),
        (0.062, -0.030),
        (0.080, 0.050),
        (0.072, 0.058),
        (0.054, -0.018),
        (0.038, -0.048),
    ]
    shade_mesh = LatheGeometry(shade_profile, segments=64, closed=True).rotate_y(math.pi / 2.0)
    extension.visual(
        mesh_from_geometry(shade_mesh, "tapered_lamp_housing"),
        origin=Origin(xyz=(0.418, 0.0, 0.0)),
        material=matte_black,
        name="lamp_housing",
    )
    front_ring_profile = [
        (0.067, -0.006),
        (0.083, -0.006),
        (0.083, 0.006),
        (0.067, 0.006),
    ]
    ring_mesh = LatheGeometry(front_ring_profile, segments=64, closed=True).rotate_y(math.pi / 2.0)
    extension.visual(
        mesh_from_geometry(ring_mesh, "front_bezel"),
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        material=matte_black,
        name="front_bezel",
    )
    extension.visual(
        Cylinder(radius=0.069, length=0.006),
        origin=Origin(xyz=(0.486, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_glass,
        name="warm_lens",
    )
    for index, (y, z) in enumerate(((0.0, 0.0), (-0.026, 0.018), (0.026, 0.018), (-0.021, -0.022), (0.021, -0.022))):
        extension.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(0.489, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=led_yellow,
            name=f"led_{index}",
        )

    model.articulation(
        "base_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.115), rpy=(0.0, -0.45, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-0.35, upper=0.65),
    )
    model.articulation(
        "telescoping_slide",
        ArticulationType.PRISMATIC,
        parent=arm_sleeve,
        child=extension,
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.220),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm_sleeve = object_model.get_part("arm_sleeve")
    extension = object_model.get_part("extension")
    base_tilt = object_model.get_articulation("base_tilt")
    telescoping_slide = object_model.get_articulation("telescoping_slide")

    ctx.allow_overlap(
        base,
        arm_sleeve,
        elem_a="hinge_pin",
        elem_b="tilt_barrel",
        reason="The steel hinge pin is intentionally captured inside the hollow tilt barrel bearing.",
    )
    ctx.allow_overlap(
        arm_sleeve,
        extension,
        elem_a="sleeve_top_wall",
        elem_b="top_glide",
        reason="A thin nylon glide strip intentionally bears against the sleeve wall to support the telescoping arm.",
    )
    ctx.allow_overlap(
        arm_sleeve,
        extension,
        elem_a="sleeve_bottom_wall",
        elem_b="bottom_glide",
        reason="The opposing nylon glide strip intentionally preloads against the sleeve wall for a snug sliding fit.",
    )
    ctx.expect_overlap(
        base,
        arm_sleeve,
        axes="yz",
        elem_a="hinge_pin",
        elem_b="tilt_barrel",
        min_overlap=0.015,
        name="tilt hinge pin is coaxially captured in the barrel",
    )
    ctx.expect_within(
        extension,
        arm_sleeve,
        axes="y",
        inner_elem="inner_tube",
        margin=0.0,
        name="inner tube is side-to-side centered within the sleeve envelope",
    )
    ctx.expect_overlap(
        extension,
        arm_sleeve,
        axes="x",
        elem_a="top_glide",
        elem_b="sleeve_top_wall",
        min_overlap=0.18,
        name="upper glide is engaged in the sleeve at rest",
    )
    ctx.expect_overlap(
        extension,
        arm_sleeve,
        axes="x",
        elem_a="bottom_glide",
        elem_b="sleeve_bottom_wall",
        min_overlap=0.18,
        name="lower glide is engaged in the sleeve at rest",
    )
    ctx.expect_overlap(
        extension,
        arm_sleeve,
        axes="x",
        elem_a="inner_tube",
        elem_b="sleeve_top_wall",
        min_overlap=0.20,
        name="collapsed telescoping tube remains inserted",
    )

    rest_pos = ctx.part_world_position(extension)
    with ctx.pose({telescoping_slide: 0.220}):
        ctx.expect_overlap(
            extension,
            arm_sleeve,
            axes="x",
            elem_a="inner_tube",
            elem_b="sleeve_top_wall",
            min_overlap=0.10,
            name="extended telescoping tube keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(extension)

    ctx.check(
        "telescoping segment extends along the lamp arm",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.16
        and extended_pos[2] > rest_pos[2] + 0.06,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({base_tilt: -0.30}):
        low_aabb = ctx.part_world_aabb(extension)
    with ctx.pose({base_tilt: 0.60}):
        high_aabb = ctx.part_world_aabb(extension)

    ctx.check(
        "base tilt joint raises the lamp head at positive angle",
        low_aabb is not None and high_aabb is not None and high_aabb[1][2] > low_aabb[1][2] + 0.20,
        details=f"low_aabb={low_aabb}, high_aabb={high_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
