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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_light_boom")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.38, 0.40, 0.43, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.96, 0.97, 0.98, 0.92))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    ring_housing_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.240, 72),
            [_circle_profile(0.160, 72)],
            0.050,
            cap=True,
            center=True,
            closed=True,
        ),
        "ring_light_housing",
    )
    ring_diffuser_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.232, 72),
            [_circle_profile(0.168, 72)],
            0.012,
            cap=True,
            center=True,
            closed=True,
        ),
        "ring_light_diffuser",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.170, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=matte_black,
        name="weighted_base",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(xyz=(0.108 * c, 0.108 * s, 0.005)),
            material=rubber,
            name=f"foot_{index}",
        )
    base.visual(
        Cylinder(radius=0.046, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=steel,
        name="base_collar",
    )
    base.visual(
        Cylinder(radius=0.022, length=1.182),
        origin=Origin(xyz=(0.0, 0.0, 0.666)),
        material=steel,
        name="post_tube",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.225)),
        material=matte_black,
        name="top_collar",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(
            xyz=(0.040, 0.0, 1.205),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=matte_black,
        name="height_lock_handle",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.340, 0.340, 1.315)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.6575)),
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.028, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="pivot_barrel",
    )
    boom.visual(
        Box((0.720, 0.040, 0.032)),
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        material=steel,
        name="front_beam",
    )
    boom.visual(
        Box((0.320, 0.040, 0.032)),
        origin=Origin(xyz=(-0.160, 0.0, 0.0)),
        material=steel,
        name="rear_beam",
    )
    boom.visual(
        Cylinder(radius=0.012, length=0.260),
        origin=Origin(xyz=(-0.310, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="counterweight_shaft",
    )
    for index, x_center in enumerate((-0.335, -0.380, -0.423)):
        boom.visual(
            Cylinder(radius=0.056, length=0.034),
            origin=Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=f"counterweight_plate_{index}",
        )
    boom.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(xyz=(0.636, 0.0, -0.055)),
        material=steel,
        name="drop_stem",
    )
    boom.visual(
        Box((0.026, 0.570, 0.024)),
        origin=Origin(xyz=(0.636, 0.0, -0.015)),
        material=matte_black,
        name="yoke_bridge",
    )
    boom.visual(
        Box((0.020, 0.024, 0.270)),
        origin=Origin(xyz=(0.636, 0.273, -0.145)),
        material=matte_black,
        name="left_yoke_arm",
    )
    boom.visual(
        Box((0.020, 0.024, 0.270)),
        origin=Origin(xyz=(0.636, -0.273, -0.145)),
        material=matte_black,
        name="right_yoke_arm",
    )
    boom.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.636, 0.296, -0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="left_tilt_knob",
    )
    boom.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.636, -0.296, -0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="right_tilt_knob",
    )
    boom.inertial = Inertial.from_geometry(
        Box((1.050, 0.620, 0.360)),
        mass=3.0,
        origin=Origin(xyz=(0.095, 0.0, -0.090)),
    )

    ring_light = model.part("ring_light")
    ring_light.visual(
        ring_housing_mesh,
        origin=Origin(xyz=(0.020, 0.0, -0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="ring_housing",
    )
    ring_light.visual(
        ring_diffuser_mesh,
        origin=Origin(xyz=(0.040, 0.0, -0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=diffuser_white,
        name="ring_diffuser",
    )
    ring_light.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, 0.247, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="left_trunnion",
    )
    ring_light.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, -0.247, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="right_trunnion",
    )
    ring_light.visual(
        Box((0.090, 0.055, 0.065)),
        origin=Origin(xyz=(-0.018, 0.0, -0.170)),
        material=steel,
        name="rear_driver_box",
    )
    ring_light.inertial = Inertial.from_geometry(
        Box((0.520, 0.520, 0.110)),
        mass=1.8,
        origin=Origin(xyz=(0.020, 0.0, -0.025)),
    )

    model.articulation(
        "post_to_boom",
        ArticulationType.REVOLUTE,
        parent=base,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, 1.285)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.85,
        ),
    )
    model.articulation(
        "boom_to_ring_light",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=ring_light,
        origin=Origin(xyz=(0.656, 0.0, -0.280)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.75,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    boom = object_model.get_part("boom")
    ring_light = object_model.get_part("ring_light")
    boom_joint = object_model.get_articulation("post_to_boom")
    tilt_joint = object_model.get_articulation("boom_to_ring_light")

    ctx.check("base exists", base is not None)
    ctx.check("boom exists", boom is not None)
    ctx.check("ring light exists", ring_light is not None)

    with ctx.pose({boom_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_gap(
            boom,
            base,
            axis="z",
            positive_elem="pivot_barrel",
            negative_elem="post_tube",
            max_gap=0.004,
            max_penetration=1e-5,
            name="boom pivot sits directly atop the post",
        )
        ctx.expect_overlap(
            boom,
            base,
            axes="xy",
            elem_a="pivot_barrel",
            elem_b="post_tube",
            min_overlap=0.040,
            name="boom pivot remains centered over the post",
        )
        ctx.expect_gap(
            boom,
            ring_light,
            axis="z",
            positive_elem="yoke_bridge",
            negative_elem="ring_housing",
            min_gap=0.035,
            name="ring light hangs below the boom yoke with clear headroom",
        )

        rest_ring_pos = ctx.part_world_position(ring_light)
        rest_housing_aabb = ctx.part_element_world_aabb(ring_light, elem="ring_housing")

    with ctx.pose({boom_joint: 0.55, tilt_joint: 0.0}):
        raised_ring_pos = ctx.part_world_position(ring_light)

    with ctx.pose({boom_joint: 0.0, tilt_joint: 0.45}):
        tilted_housing_aabb = ctx.part_element_world_aabb(ring_light, elem="ring_housing")

    ctx.check(
        "boom articulation raises the lamp end",
        rest_ring_pos is not None
        and raised_ring_pos is not None
        and raised_ring_pos[2] > rest_ring_pos[2] + 0.18,
        details=f"rest={rest_ring_pos}, raised={raised_ring_pos}",
    )

    if rest_housing_aabb is None or tilted_housing_aabb is None:
        ctx.fail(
            "ring tilt moves the lamp face",
            details=f"rest_aabb={rest_housing_aabb}, tilted_aabb={tilted_housing_aabb}",
        )
    else:
        rest_center_x = 0.5 * (rest_housing_aabb[0][0] + rest_housing_aabb[1][0])
        tilted_center_x = 0.5 * (tilted_housing_aabb[0][0] + tilted_housing_aabb[1][0])
        ctx.check(
            "ring tilt pitches the lamp forward",
            tilted_center_x > rest_center_x + 0.005,
            details=f"rest_center_x={rest_center_x}, tilted_center_x={tilted_center_x}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
