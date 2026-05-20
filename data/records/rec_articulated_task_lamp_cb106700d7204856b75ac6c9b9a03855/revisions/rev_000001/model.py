from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


BLACK = Material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
RUBBER = Material("rubber_black", rgba=(0.005, 0.005, 0.004, 1.0))
CHROME = Material("brushed_chrome", rgba=(0.67, 0.68, 0.66, 1.0))
WHITE = Material("soft_white_reflector", rgba=(0.94, 0.92, 0.86, 1.0))
SILVER = Material("matte_silver", rgba=(0.72, 0.72, 0.70, 1.0))


def _dish_shell_mesh():
    """Thin lathed beauty-dish reflector, oriented with its optical axis on +X."""
    outer = [
        (0.060, 0.020),
        (0.105, 0.055),
        (0.205, 0.125),
        (0.320, 0.205),
    ]
    inner = [
        (0.045, 0.033),
        (0.090, 0.066),
        (0.185, 0.132),
        (0.300, 0.190),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )
    shell.rotate_y(pi / 2)
    return mesh_from_geometry(shell, "dish_shell")


def _rim_mesh():
    rim = TorusGeometry(0.318, 0.011, radial_segments=18, tubular_segments=96)
    rim.rotate_y(pi / 2).translate(0.205, 0.0, 0.0)
    return mesh_from_geometry(rim, "dish_rolled_rim")


def _straight_tube_mesh(start, end, radius, name, segments=18):
    tube = wire_from_points(
        [start, end],
        radius=radius,
        radial_segments=segments,
        cap_ends=True,
        corner_mode="miter",
    )
    return mesh_from_geometry(tube, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="beauty_dish_boom_arm",
        materials=[BLACK, RUBBER, CHROME, WHITE, SILVER],
    )

    stand = model.part("stand")

    # Low, wide tripod base.  The three tubular legs all run into a real central
    # hub so the base reads as one welded/supporting assembly.
    for i, angle in enumerate((pi / 2, pi / 2 + 2 * pi / 3, pi / 2 + 4 * pi / 3)):
        end = (0.68 * cos(angle), 0.68 * sin(angle), 0.032)
        stand.visual(
            _straight_tube_mesh((0.0, 0.0, 0.095), end, 0.018, f"tripod_leg_{i}"),
            material=BLACK,
            name=f"tripod_leg_{i}",
        )
        stand.visual(
            Sphere(0.034),
            origin=Origin(xyz=end),
            material=RUBBER,
            name=f"rubber_foot_{i}",
        )

    stand.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=BLACK,
        name="base_hub",
    )
    stand.visual(
        Cylinder(radius=0.029, length=1.56),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=CHROME,
        name="lower_post",
    )
    stand.visual(
        Cylinder(radius=0.023, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 1.39)),
        material=CHROME,
        name="upper_post",
    )
    stand.visual(
        Cylinder(radius=0.042, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 1.63)),
        material=BLACK,
        name="top_collar",
    )

    # Forked post-top cradle for the boom tilt pin.
    stand.visual(
        Box((0.13, 0.026, 0.15)),
        origin=Origin(xyz=(0.0, 0.073, 1.72)),
        material=BLACK,
        name="top_cheek_0",
    )
    stand.visual(
        Box((0.13, 0.026, 0.15)),
        origin=Origin(xyz=(0.0, -0.073, 1.72)),
        material=BLACK,
        name="top_cheek_1",
    )
    stand.visual(
        Box((0.095, 0.170, 0.040)),
        origin=Origin(xyz=(-0.025, 0.0, 1.655)),
        material=BLACK,
        name="top_bridge",
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.031, length=0.180),
        origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
        material=BLACK,
        name="pivot_pin",
    )
    boom.visual(
        Cylinder(radius=0.021, length=1.08),
        origin=Origin(xyz=(0.56, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=CHROME,
        name="boom_tube",
    )
    boom.visual(
        Cylinder(radius=0.019, length=0.30),
        origin=Origin(xyz=(-0.15, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=CHROME,
        name="rear_stub",
    )
    boom.visual(
        Cylinder(radius=0.055, length=0.085),
        origin=Origin(xyz=(-0.325, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=BLACK,
        name="counterweight",
    )
    boom.visual(
        Box((0.090, 0.090, 0.085)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=BLACK,
        name="pivot_block",
    )

    # Wide end yoke: arms sit outside the dish rim and carry the dish tilt pin.
    boom.visual(
        Box((0.080, 0.080, 0.145)),
        origin=Origin(xyz=(1.055, 0.0, -0.035)),
        material=BLACK,
        name="end_drop_block",
    )
    boom.visual(
        Box((0.110, 0.770, 0.042)),
        origin=Origin(xyz=(1.075, 0.0, -0.090)),
        material=BLACK,
        name="end_yoke_bridge",
    )
    boom.visual(
        Box((0.155, 0.026, 0.170)),
        origin=Origin(xyz=(1.130, 0.365, 0.0)),
        material=BLACK,
        name="end_cheek_0",
    )
    boom.visual(
        Box((0.155, 0.026, 0.170)),
        origin=Origin(xyz=(1.130, -0.365, 0.0)),
        material=BLACK,
        name="end_cheek_1",
    )

    dish = model.part("dish")
    dish.visual(
        Cylinder(radius=0.018, length=0.780),
        origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
        material=BLACK,
        name="dish_pin",
    )
    dish.visual(
        Cylinder(radius=0.060, length=0.075),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=BLACK,
        name="rear_hub",
    )
    dish.visual(
        Cylinder(radius=0.012, length=0.170),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=CHROME,
        name="center_stem",
    )
    dish.visual(_dish_shell_mesh(), material=WHITE, name="reflector_shell")
    dish.visual(_rim_mesh(), material=WHITE, name="rolled_rim")
    dish.visual(
        Cylinder(radius=0.105, length=0.012),
        origin=Origin(xyz=(0.158, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=SILVER,
        name="center_deflector",
    )
    for i, angle in enumerate((pi / 4, 3 * pi / 4, 5 * pi / 4, 7 * pi / 4)):
        start = (0.158, 0.090 * cos(angle), 0.090 * sin(angle))
        end = (0.196, 0.307 * cos(angle), 0.307 * sin(angle))
        dish.visual(
            _straight_tube_mesh(start, end, 0.004, f"deflector_spoke_{i}", segments=12),
            material=CHROME,
            name=f"deflector_spoke_{i}",
        )

    model.articulation(
        "post_to_boom",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, 1.72)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.3, lower=-0.60, upper=0.85),
    )
    model.articulation(
        "boom_to_dish",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=dish,
        origin=Origin(xyz=(1.130, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.6, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    boom = object_model.get_part("boom")
    dish = object_model.get_part("dish")
    post_joint = object_model.get_articulation("post_to_boom")
    dish_joint = object_model.get_articulation("boom_to_dish")

    for cheek in ("top_cheek_0", "top_cheek_1"):
        ctx.allow_overlap(
            stand,
            boom,
            elem_a=cheek,
            elem_b="pivot_pin",
            reason="The boom tilt pin is intentionally captured in the post-top fork cheeks.",
        )
        ctx.expect_overlap(
            stand,
            boom,
            axes="xz",
            elem_a=cheek,
            elem_b="pivot_pin",
            min_overlap=0.015,
            name=f"{cheek} captures boom pivot pin",
        )

    for cheek in ("end_cheek_0", "end_cheek_1"):
        ctx.allow_overlap(
            boom,
            dish,
            elem_a=cheek,
            elem_b="dish_pin",
            reason="The dish trunnion pin is intentionally seated through the end-yoke cheeks.",
        )
        ctx.expect_overlap(
            boom,
            dish,
            axes="xz",
            elem_a=cheek,
            elem_b="dish_pin",
            min_overlap=0.015,
            name=f"{cheek} captures dish tilt pin",
        )

    ctx.expect_gap(
        boom,
        stand,
        axis="z",
        positive_elem="boom_tube",
        negative_elem="top_bridge",
        min_gap=0.020,
        name="boom tube clears the lower post-top bridge",
    )
    ctx.expect_overlap(
        dish,
        boom,
        axes="y",
        elem_a="dish_pin",
        elem_b="end_yoke_bridge",
        min_overlap=0.70,
        name="dish pin spans the wide yoke",
    )

    rest_dish_position = ctx.part_world_position(dish)
    with ctx.pose({post_joint: 0.65}):
        raised_dish_position = ctx.part_world_position(dish)
    ctx.check(
        "post top joint raises boom end",
        rest_dish_position is not None
        and raised_dish_position is not None
        and raised_dish_position[2] > rest_dish_position[2] + 0.45,
        details=f"rest={rest_dish_position}, raised={raised_dish_position}",
    )

    rest_aabb = ctx.part_world_aabb(dish)
    with ctx.pose({dish_joint: 0.55}):
        tilted_aabb = ctx.part_world_aabb(dish)
    rest_center_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
    tilted_center_z = None if tilted_aabb is None else (tilted_aabb[0][2] + tilted_aabb[1][2]) / 2.0
    ctx.check(
        "dish tilt joint tips reflector upward",
        rest_center_z is not None
        and tilted_center_z is not None
        and tilted_center_z > rest_center_z + 0.04,
        details=f"rest_center_z={rest_center_z}, tilted_center_z={tilted_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
