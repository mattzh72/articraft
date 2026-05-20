from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


BASE_RADIUS = 0.155
BASE_THICKNESS = 0.030
POST_RADIUS = 0.013
POST_LENGTH = 1.420
COLLAR_RADIUS = 0.026
COLLAR_LENGTH = 0.330
COLLAR_Z = 1.285
BRANCH_PIN_RADIUS = 0.006
BRANCH_PIN_LENGTH = 0.090
BRANCH_SLEEVE_OUTER = 0.013
BRANCH_SLEEVE_INNER = 0.0083
BRANCH_SLEEVE_LENGTH = 0.050
MOUNT_RADIUS = 0.041
BRANCH_CONFIGS = (
    {
        "name": "branch_0",
        "shade": "shade_0",
        "yaw": math.radians(20.0),
        "mount_z": 1.155,
        "reach": 0.405,
        "rise": 0.100,
        "shade_pitch": -0.24,
    },
    {
        "name": "branch_1",
        "shade": "shade_1",
        "yaw": math.radians(150.0),
        "mount_z": 1.275,
        "reach": 0.455,
        "rise": 0.145,
        "shade_pitch": -0.34,
    },
    {
        "name": "branch_2",
        "shade": "shade_2",
        "yaw": math.radians(-105.0),
        "mount_z": 1.395,
        "reach": 0.430,
        "rise": 0.118,
        "shade_pitch": -0.28,
    },
)


def _centered_tube(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length).translate((0.0, 0.0, -length / 2.0))
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.004)
        .translate((0.0, 0.0, -(length + 0.004) / 2.0))
    )
    return outer.cut(inner)


def _cone_shade_shell(length: float, rear_radius: float, front_radius: float, wall: float) -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(rear_radius).workplane(offset=length).circle(front_radius).loft(combine=True)
    inner = (
        cq.Workplane("YZ")
        .circle(max(rear_radius - wall, wall * 0.8))
        .workplane(offset=length + wall * 1.5)
        .circle(max(front_radius - wall, wall * 0.8))
        .loft(combine=True)
        .translate((wall * 2.5, 0.0, 0.0))
    )
    return outer.cut(inner)


def _branch_points(reach: float, rise: float) -> list[tuple[float, float, float]]:
    return [
        (0.018, 0.0, 0.006),
        (reach * 0.28, 0.0, rise * 0.22),
        (reach * 0.58, 0.0, rise * 0.63),
        (reach * 0.86, 0.0, rise * 0.92),
        (reach, 0.0, rise),
    ]


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tree_floor_lamp")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.68, 0.57, 0.31, 1.0))
    parchment = model.material("parchment", rgba=(0.92, 0.89, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=matte_black,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.010)),
        material=matte_black,
        name="base_cap",
    )
    base.visual(
        Cylinder(radius=POST_RADIUS, length=POST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_LENGTH / 2.0)),
        material=warm_brass,
        name="post",
    )
    base.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        material=warm_brass,
        name="collar",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_LENGTH + 0.020)),
        material=warm_brass,
        name="top_cap",
    )

    for index, cfg in enumerate(BRANCH_CONFIGS):
        base.visual(
            Cylinder(radius=BRANCH_PIN_RADIUS, length=BRANCH_PIN_LENGTH),
            origin=Origin(
                xyz=(
                    MOUNT_RADIUS * math.cos(cfg["yaw"]),
                    MOUNT_RADIUS * math.sin(cfg["yaw"]),
                    cfg["mount_z"],
                )
            ),
            material=warm_brass,
            name=f"collar_pin_{index}",
        )
        base.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(
                xyz=(
                    MOUNT_RADIUS * math.cos(cfg["yaw"]),
                    MOUNT_RADIUS * math.sin(cfg["yaw"]),
                    cfg["mount_z"] - BRANCH_SLEEVE_LENGTH / 2.0 - 0.002,
                )
            ),
            material=warm_brass,
            name=f"pivot_pad_{index}",
        )
        base.visual(
            Box((0.036, 0.014, 0.018)),
            origin=Origin(
                xyz=(
                    0.024 * math.cos(cfg["yaw"]),
                    0.024 * math.sin(cfg["yaw"]),
                    cfg["mount_z"] - 0.040,
                ),
                rpy=(0.0, 0.0, cfg["yaw"]),
            ),
            material=warm_brass,
            name=f"collar_gusset_{index}",
        )

    for index, cfg in enumerate(BRANCH_CONFIGS):
        branch = model.part(cfg["name"])
        branch.visual(
            mesh_from_cadquery(
                _centered_tube(
                    length=BRANCH_SLEEVE_LENGTH,
                    outer_radius=BRANCH_SLEEVE_OUTER,
                    inner_radius=BRANCH_SLEEVE_INNER,
                ),
                f"{cfg['name']}_sleeve",
            ),
            material=warm_brass,
            name="hub_sleeve",
        )
        branch.visual(
            Box((0.020, 0.018, 0.022)),
            origin=Origin(xyz=(0.022, 0.0, 0.004)),
            material=warm_brass,
            name="shoulder_block",
        )
        branch.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _branch_points(cfg["reach"], cfg["rise"]),
                    radius=0.008,
                    samples_per_segment=20,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"{cfg['name']}_tube",
            ),
            material=warm_brass,
            name="arm_tube",
        )
        branch.visual(
            Box((0.032, 0.022, 0.024)),
            origin=Origin(xyz=(cfg["reach"] - 0.022, 0.0, cfg["rise"])),
            material=warm_brass,
            name="tip_block",
        )
        branch.visual(
            Box((0.018, 0.012, 0.020)),
            origin=Origin(xyz=(cfg["reach"] + 0.003, 0.0, cfg["rise"])),
            material=warm_brass,
            name="hinge_pad",
        )
        branch.visual(
            Box((0.018, 0.003, 0.030)),
            origin=Origin(xyz=(cfg["reach"] + 0.001, 0.012, cfg["rise"])),
            material=warm_brass,
            name="yoke_0",
        )
        branch.visual(
            Box((0.018, 0.003, 0.030)),
            origin=Origin(xyz=(cfg["reach"] + 0.001, -0.012, cfg["rise"])),
            material=warm_brass,
            name="yoke_1",
        )

        shade = model.part(cfg["shade"])
        shade.visual(
            Box((0.016, 0.010, 0.020)),
            origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, cfg["shade_pitch"], 0.0)),
            material=warm_brass,
            name="hinge_block",
        )
        shade.visual(
            Cylinder(radius=0.015, length=0.032),
            origin=Origin(
                xyz=(0.032, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0 + cfg["shade_pitch"], 0.0),
            ),
            material=warm_brass,
            name="socket_housing",
        )
        shade.visual(
            mesh_from_cadquery(
                _cone_shade_shell(
                    length=0.205,
                    rear_radius=0.022,
                    front_radius=0.074,
                    wall=0.0026,
                ),
                f"{cfg['shade']}_shell",
            ),
            origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, cfg["shade_pitch"], 0.0)),
            material=parchment,
            name="shell",
        )

        branch_joint = model.articulation(
            f"base_to_{cfg['name']}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=branch,
            origin=Origin(
                xyz=(
                    MOUNT_RADIUS * math.cos(cfg["yaw"]),
                    MOUNT_RADIUS * math.sin(cfg["yaw"]),
                    cfg["mount_z"],
                ),
                rpy=(0.0, 0.0, cfg["yaw"]),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                lower=-0.95,
                upper=0.95,
                effort=8.0,
                velocity=1.2,
            ),
        )
        branch_joint.meta["qc_samples"] = [0.0, -0.6, 0.6]

        shade_joint = model.articulation(
            f"{cfg['name']}_to_{cfg['shade']}",
            ArticulationType.REVOLUTE,
            parent=branch,
            child=shade,
            origin=Origin(xyz=(cfg["reach"] + 0.012, 0.0, cfg["rise"])),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                lower=-0.70,
                upper=0.55,
                effort=5.0,
                velocity=1.5,
            ),
        )
        shade_joint.meta["qc_samples"] = [0.0, -0.45, 0.40]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    branch_0 = object_model.get_part("branch_0")
    shade_0 = object_model.get_part("shade_0")
    shade_1 = object_model.get_part("shade_1")
    shade_2 = object_model.get_part("shade_2")

    base_to_branch_0 = object_model.get_articulation("base_to_branch_0")
    branch_0_to_shade_0 = object_model.get_articulation("branch_0_to_shade_0")

    for index in range(3):
        ctx.allow_overlap(
            f"branch_{index}",
            f"shade_{index}",
            elem_a="hinge_pad",
            elem_b="hinge_block",
            reason="The compact branch knuckle is intentionally simplified as a shallow embedded hinge mount around the shade pivot block.",
        )

    ctx.expect_gap(
        shade_0,
        base,
        axis="z",
        min_gap=1.05,
        negative_elem="base_disk",
        name="lower shade clears the base",
    )
    ctx.expect_gap(
        shade_1,
        base,
        axis="z",
        min_gap=1.20,
        negative_elem="base_disk",
        name="middle shade clears the base",
    )
    ctx.expect_gap(
        shade_2,
        base,
        axis="z",
        min_gap=1.30,
        negative_elem="base_disk",
        name="upper shade clears the base",
    )
    ctx.expect_gap(
        shade_0,
        branch_0,
        axis="x",
        min_gap=0.008,
        positive_elem="shell",
        negative_elem="tip_block",
        name="shade shell stays forward of the branch tip",
    )
    ctx.expect_contact(
        branch_0,
        base,
        name="branch_0 hub seats on the collar",
    )

    rest_pos = ctx.part_world_position(shade_0)
    with ctx.pose({base_to_branch_0: 0.70}):
        swung_pos = ctx.part_world_position(shade_0)

    ctx.check(
        "branch_0 sweeps around the post",
        rest_pos is not None
        and swung_pos is not None
        and math.hypot(swung_pos[0] - rest_pos[0], swung_pos[1] - rest_pos[1]) > 0.18,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(shade_0, elem="shell")
    with ctx.pose({branch_0_to_shade_0: 0.40}):
        tipped_shell_aabb = ctx.part_element_world_aabb(shade_0, elem="shell")

    rest_center_z = _aabb_center_z(rest_shell_aabb)
    tipped_center_z = _aabb_center_z(tipped_shell_aabb)
    ctx.check(
        "shade_0 tilts downward",
        rest_center_z is not None and tipped_center_z is not None and tipped_center_z < rest_center_z - 0.03,
        details=f"rest_center_z={rest_center_z}, tipped_center_z={tipped_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
