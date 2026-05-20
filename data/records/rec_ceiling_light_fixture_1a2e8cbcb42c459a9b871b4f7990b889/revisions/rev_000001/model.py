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
    mesh_from_geometry,
)


CANOPY_RADIUS = 0.055
CANOPY_HEIGHT = 0.022
HUB_Z = -0.058
ARM_HINGE_RADIUS = 0.036
ARM_ROD_LENGTH = 0.144
ARM_TIP_X = 0.160
SHADE_PITCH_REST = math.radians(52.0)


def _build_shade_shell():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.004, 0.000),
            (0.006, 0.005),
            (0.010, 0.014),
            (0.017, 0.028),
            (0.024, 0.044),
            (0.028, 0.056),
        ],
        [
            (0.000, 0.002),
            (0.003, 0.006),
            (0.007, 0.015),
            (0.013, 0.029),
            (0.020, 0.044),
            (0.024, 0.054),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )
    shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, "shade_shell")


def _add_hub_hinge_clevis(part, angle: float, *, material: str, index: int) -> None:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    part.visual(
        Box((0.012, 0.010, 0.016)),
        origin=Origin(
            xyz=(0.0245 * cos_a, 0.0245 * sin_a, HUB_Z),
            rpy=(0.0, 0.0, angle),
        ),
        material=material,
        name=f"hub_clevis_{index}",
    )
    for vertical_sign, z_offset, name_suffix in (
        (1.0, 0.006, "upper"),
        (-1.0, -0.006, "lower"),
    ):
        part.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(
                xyz=(0.036 * cos_a, 0.036 * sin_a, HUB_Z + z_offset),
                rpy=(0.0, 0.0, 0.0),
            ),
            material=material,
            name=f"hub_knuckle_{index}_{name_suffix}",
        )


def _add_arm_geometry(part, *, material: str) -> None:
    part.visual(
        Cylinder(radius=0.005, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=material,
        name="inner_barrel",
    )
    part.visual(
        Box((0.016, 0.010, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=material,
        name="inner_collar",
    )
    part.visual(
        Cylinder(radius=0.0046, length=ARM_ROD_LENGTH),
        origin=Origin(
            xyz=(0.078, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name="rod",
    )
    part.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(0.146, 0.0, 0.0)),
        material=material,
        name="tip_block",
    )
    for side, y_pos in (("a", -0.0045), ("b", 0.0045)):
        part.visual(
            Box((0.006, 0.003, 0.006)),
            origin=Origin(xyz=(0.154, y_pos, 0.0)),
            material=material,
            name=f"tip_bridge_{side}",
        )
    for side, y_pos in (("a", -0.0045), ("b", 0.0045)):
        part.visual(
            Box((0.008, 0.003, 0.014)),
            origin=Origin(xyz=(ARM_TIP_X, y_pos, 0.0)),
            material=material,
            name=f"tip_ear_{side}",
        )


def _add_shade_geometry(part, shade_shell, *, shade_material: str, hinge_material: str) -> None:
    part.visual(
        Cylinder(radius=0.0025, length=0.0056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_material,
        name="shade_barrel",
    )
    part.visual(
        Box((0.010, 0.005, 0.010)),
        origin=Origin(xyz=(0.005, 0.0, -0.004)),
        material=hinge_material,
        name="shade_neck",
    )
    part.visual(
        shade_shell,
        origin=Origin(xyz=(0.009, 0.0, -0.012)),
        material=shade_material,
        name="shade_shell",
    )
    part.visual(
        Box((0.004, 0.005, 0.004)),
        origin=Origin(xyz=(0.001, 0.0, 0.001)),
        material=hinge_material,
        name="switch_bridge",
    )
    for side, y_pos in (("a", -0.0024), ("b", 0.0024)):
        part.visual(
            Box((0.004, 0.0018, 0.005)),
            origin=Origin(xyz=(-0.001, y_pos, 0.001)),
            material=hinge_material,
            name=f"switch_ear_{side}",
        )


def _add_switch_geometry(part, *, material: str) -> None:
    part.visual(
        Cylinder(radius=0.0012, length=0.0034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="switch_barrel",
    )
    part.visual(
        Box((0.009, 0.003, 0.003)),
        origin=Origin(xyz=(0.0045, 0.0, 0.0)),
        material=material,
        name="switch_paddle",
    )
    part.visual(
        Box((0.003, 0.0038, 0.0026)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=material,
        name="switch_tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_light_fixture")

    body_black = model.material("body_black", rgba=(0.12, 0.12, 0.13, 1.0))
    shade_white = model.material("shade_white", rgba=(0.93, 0.93, 0.91, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.48, 0.49, 0.50, 1.0))
    switch_black = model.material("switch_black", rgba=(0.08, 0.08, 0.09, 1.0))

    shade_shell = _build_shade_shell()

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=CANOPY_RADIUS, length=CANOPY_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, -CANOPY_HEIGHT / 2.0)),
        material=body_black,
        name="canopy_shell",
    )
    canopy.visual(
        Cylinder(radius=0.042, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=body_black,
        name="canopy_trim",
    )
    canopy.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=body_black,
        name="stem",
    )
    canopy.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        material=body_black,
        name="hub_body",
    )
    canopy.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        material=body_black,
        name="hub_band",
    )

    arm_angles = (0.0, 2.0 * math.pi / 3.0, -2.0 * math.pi / 3.0)
    for index, angle in enumerate(arm_angles):
        _add_hub_hinge_clevis(canopy, angle, material=body_black, index=index)

    for index, angle in enumerate(arm_angles):
        arm = model.part(f"arm_{index}")
        _add_arm_geometry(arm, material=body_black)

        shade = model.part(f"shade_{index}")
        _add_shade_geometry(
            shade,
            shade_shell,
            shade_material=shade_white,
            hinge_material=hinge_metal,
        )

        switch = model.part(f"switch_{index}")
        _add_switch_geometry(switch, material=switch_black)

        model.articulation(
            f"hub_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=canopy,
            child=arm,
            origin=Origin(
                xyz=(
                    ARM_HINGE_RADIUS * math.cos(angle),
                    ARM_HINGE_RADIUS * math.sin(angle),
                    HUB_Z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=1.6,
                lower=-math.radians(80.0),
                upper=math.radians(80.0),
            ),
        )
        model.articulation(
            f"arm_{index}_to_shade_{index}",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=shade,
            origin=Origin(
                xyz=(ARM_TIP_X, 0.0, 0.0),
                rpy=(0.0, SHADE_PITCH_REST, 0.0),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.0,
                lower=-math.radians(32.0),
                upper=math.radians(42.0),
            ),
        )
        model.articulation(
            f"shade_{index}_to_switch_{index}",
            ArticulationType.REVOLUTE,
            parent=shade,
            child=switch,
            origin=Origin(
                xyz=(-0.001, 0.0, 0.001),
                rpy=(0.0, 0.0, math.pi),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=4.0,
                lower=-math.radians(20.0),
                upper=math.radians(25.0),
            ),
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((a + b) * 0.5 for a, b in zip(low, high))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    arm_0 = object_model.get_part("arm_0")
    shade_0 = object_model.get_part("shade_0")
    switch_0 = object_model.get_part("switch_0")

    arm_swivel = object_model.get_articulation("hub_to_arm_0")
    shade_tilt = object_model.get_articulation("arm_0_to_shade_0")
    switch_pivot = object_model.get_articulation("shade_0_to_switch_0")

    for index in range(3):
        shade = object_model.get_part(f"shade_{index}")
        ctx.expect_origin_distance(
            shade,
            canopy,
            axes="xy",
            min_dist=0.13,
            max_dist=0.205,
            name=f"shade_{index} sits outboard of the canopy",
        )
        ctx.expect_origin_gap(
            canopy,
            shade,
            axis="z",
            min_gap=0.045,
            max_gap=0.075,
            name=f"shade_{index} hangs below the canopy",
        )

    arm_rest_center = _aabb_center(ctx.part_world_aabb(arm_0))
    with ctx.pose({arm_swivel: math.radians(38.0)}):
        arm_swung_center = _aabb_center(ctx.part_world_aabb(arm_0))
    ctx.check(
        "arm_0 swivels around the hub",
        arm_rest_center is not None
        and arm_swung_center is not None
        and arm_swung_center[1] > arm_rest_center[1] + 0.04,
        details=f"rest={arm_rest_center}, swung={arm_swung_center}",
    )

    shade_rest_center = _aabb_center(ctx.part_world_aabb(shade_0))
    with ctx.pose({shade_tilt: math.radians(32.0)}):
        shade_tipped_center = _aabb_center(ctx.part_world_aabb(shade_0))
    ctx.check(
        "shade_0 tilts at the tip hinge",
        shade_rest_center is not None
        and shade_tipped_center is not None
        and shade_tipped_center[0] > shade_rest_center[0] + 0.012
        and shade_tipped_center[2] > shade_rest_center[2] + 0.008,
        details=f"rest={shade_rest_center}, tipped={shade_tipped_center}",
    )

    switch_rest_center = _aabb_center(ctx.part_world_aabb(switch_0))
    with ctx.pose({switch_pivot: math.radians(20.0)}):
        switch_toggled_center = _aabb_center(ctx.part_world_aabb(switch_0))
    ctx.check(
        "switch_0 pivots on its rear hinge",
        switch_rest_center is not None
        and switch_toggled_center is not None
        and switch_toggled_center[2] < switch_rest_center[2] - 0.0008,
        details=f"rest={switch_rest_center}, toggled={switch_toggled_center}",
    )

    return ctx.report()


object_model = build_object_model()
