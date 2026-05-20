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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


POST_RADIUS = 0.016
COLLAR_INNER_RADIUS = 0.0225
COLLAR_OUTER_RADIUS = 0.031
COLLAR_HEIGHT = 0.046


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_collar_mesh(name: str):
    outer_profile = [
        (COLLAR_OUTER_RADIUS, -COLLAR_HEIGHT * 0.5),
        (COLLAR_OUTER_RADIUS, COLLAR_HEIGHT * 0.5),
    ]
    inner_profile = [
        (COLLAR_INNER_RADIUS, -COLLAR_HEIGHT * 0.5),
        (COLLAR_INNER_RADIUS, COLLAR_HEIGHT * 0.5),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=40,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        ),
    )


def _build_globe_shade_mesh(name: str):
    outer_profile = [
        (0.018, -0.030),
        (0.050, -0.055),
        (0.083, -0.090),
        (0.090, -0.118),
        (0.076, -0.158),
        (0.044, -0.192),
    ]
    inner_profile = [
        (0.014, -0.030),
        (0.046, -0.057),
        (0.079, -0.090),
        (0.086, -0.118),
        (0.072, -0.155),
        (0.040, -0.192),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=10,
        ),
    )


def _add_branch_arm(
    model: ArticulatedObject,
    *,
    index: int,
    arm_material,
    collar_mesh,
    tip_x: float,
    tip_z: float,
) -> None:
    arm = model.part(f"arm_{index}")
    arm.visual(collar_mesh, material=arm_material, name="collar_sleeve")
    arm.visual(
        Box((0.040, 0.026, 0.018)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=arm_material,
        name="collar_bracket",
    )

    tube_points = [
        (0.070, 0.0, 0.000),
        (0.150, 0.0, tip_z * 0.10),
        (tip_x * 0.55, 0.0, tip_z * 0.48),
        (tip_x * 0.83, 0.0, tip_z * 0.92),
        (tip_x - 0.020, 0.0, tip_z),
    ]
    arm.visual(
        _save_mesh(
            f"five_branch_lamp_arm_tube_{index}",
            tube_from_spline_points(
                tube_points,
                radius=0.011,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=arm_material,
        name="branch_tube",
    )
    arm.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(tip_x - 0.030, 0.0, tip_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=arm_material,
        name="tip_housing",
    )
    arm.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(tip_x, 0.0, tip_z + 0.003)),
        material=arm_material,
        name="pivot_cap",
    )
    arm.visual(
        Box((0.020, 0.018, 0.008)),
        origin=Origin(xyz=(tip_x - 0.010, 0.0, tip_z + 0.010)),
        material=arm_material,
        name="tip_connector",
    )
    arm.inertial = Inertial.from_geometry(
        Box((tip_x + 0.08, 0.09, tip_z + 0.12)),
        mass=1.05,
        origin=Origin(xyz=(tip_x * 0.50, 0.0, tip_z * 0.50)),
    )

    shade = model.part(f"shade_{index}")
    shade.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=arm_material,
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=arm_material,
        name="socket_stem",
    )
    shade.visual(
        _build_globe_shade_mesh(f"five_branch_lamp_globe_{index}"),
        material="frosted_glass",
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.19, 0.19, 0.22)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
    )

    model.articulation(
        f"arm_{index}_to_shade_{index}",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(tip_x, 0.0, tip_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.6,
            lower=-0.85,
            upper=0.65,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_branch_tree_floor_lamp")

    blackened_steel = model.material("blackened_steel", rgba=(0.18, 0.16, 0.15, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.68, 0.57, 0.38, 1.0))
    model.material("frosted_glass", rgba=(0.95, 0.94, 0.90, 0.58))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.170, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=blackened_steel,
        name="base_disc",
    )
    stand.visual(
        Cylinder(radius=0.086, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=warm_brass,
        name="base_cap",
    )
    stand.visual(
        Cylinder(radius=POST_RADIUS, length=1.560),
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=warm_brass,
        name="center_post",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 1.613)),
        material=warm_brass,
        name="top_finial",
    )

    arm_specs = [
        {"height": 0.580, "yaw": 0.10, "tip_x": 0.380, "tip_z": 0.190},
        {"height": 0.790, "yaw": 1.33, "tip_x": 0.420, "tip_z": 0.215},
        {"height": 1.000, "yaw": 2.56, "tip_x": 0.455, "tip_z": 0.230},
        {"height": 1.220, "yaw": 3.82, "tip_x": 0.490, "tip_z": 0.250},
        {"height": 1.440, "yaw": 5.05, "tip_x": 0.520, "tip_z": 0.265},
    ]
    for index, spec in enumerate(arm_specs):
        stand.visual(
            Cylinder(radius=0.0295, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, spec["height"] - COLLAR_HEIGHT * 0.5 - 0.003)),
            material=blackened_steel,
            name=f"post_collar_lower_{index}",
        )
        stand.visual(
            Cylinder(radius=0.0295, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, spec["height"] + COLLAR_HEIGHT * 0.5 + 0.003)),
            material=blackened_steel,
            name=f"post_collar_upper_{index}",
        )

    stand.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 1.65)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
    )

    collar_mesh = _build_collar_mesh("five_branch_lamp_rotating_collar")
    for index, spec in enumerate(arm_specs):
        _add_branch_arm(
            model,
            index=index,
            arm_material=warm_brass,
            collar_mesh=collar_mesh,
            tip_x=spec["tip_x"],
            tip_z=spec["tip_z"],
        )
        model.articulation(
            f"stand_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=f"arm_{index}",
            origin=Origin(xyz=(0.0, 0.0, spec["height"]), rpy=(0.0, 0.0, spec["yaw"])),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.2,
                lower=-2.7,
                upper=2.7,
            ),
        )

    return model


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((low + high) * 0.5 for low, high in zip(mins, maxs))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")

    for index in range(5):
        shade = object_model.get_part(f"shade_{index}")
        arm_joint = object_model.get_articulation(f"stand_to_arm_{index}")
        shade_joint = object_model.get_articulation(f"arm_{index}_to_shade_{index}")

        ctx.expect_origin_distance(
            shade,
            stand,
            axes="xy",
            min_dist=0.34,
            name=f"shade_{index} sits away from the center post",
        )

        rest_pos = ctx.part_world_position(shade)
        with ctx.pose({arm_joint: 0.45}):
            swung_pos = ctx.part_world_position(shade)
        ctx.check(
            f"arm_{index} revolves around the post",
            rest_pos is not None
            and swung_pos is not None
            and ((swung_pos[0] - rest_pos[0]) ** 2 + (swung_pos[1] - rest_pos[1]) ** 2) ** 0.5 > 0.12
            and abs(swung_pos[2] - rest_pos[2]) < 0.04,
            details=f"rest={rest_pos}, swung={swung_pos}",
        )

        rest_shell = _center_from_aabb(ctx.part_element_world_aabb(shade, elem="shade_shell"))
        with ctx.pose({shade_joint: 0.45}):
            tilted_shell = _center_from_aabb(ctx.part_element_world_aabb(shade, elem="shade_shell"))
        ctx.check(
            f"shade_{index} tilts on its tip joint",
            rest_shell is not None
            and tilted_shell is not None
            and (
                (tilted_shell[0] - rest_shell[0]) ** 2
                + (tilted_shell[1] - rest_shell[1]) ** 2
                + (tilted_shell[2] - rest_shell[2]) ** 2
            ) ** 0.5
            > 0.025
            and tilted_shell[2] > rest_shell[2] + 0.008,
            details=f"rest={rest_shell}, tilted={tilted_shell}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
