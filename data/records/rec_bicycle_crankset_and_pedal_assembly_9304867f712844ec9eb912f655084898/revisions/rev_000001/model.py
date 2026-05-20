from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _chainring_tooth_profile(teeth: int = 44) -> list[tuple[float, float]]:
    """Alternating root/tip radii for a recognizable single-speed chainring."""
    points: list[tuple[float, float]] = []
    root_radius = 0.096
    tip_radius = 0.105
    for i in range(teeth):
        base = 2.0 * math.pi * i / teeth
        # Three points per tooth gives a narrow steel tooth and wider root valley.
        for frac, radius in ((0.08, root_radius), (0.50, tip_radius), (0.92, root_radius)):
            a = base + frac * 2.0 * math.pi / teeth
            points.append((radius * math.cos(a), radius * math.sin(a)))
    return points


def _annular_mesh(outer_radius: float, inner_radius: float, thickness: float, name: str):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, 96),
        [_circle_profile(inner_radius, 96)],
        thickness,
        center=True,
    )
    # Mesh extrusion is local +Z; rotate so the tube/ring axis is the crankset's Y axis.
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _chainring_mesh():
    geom = ExtrudeWithHolesGeometry(
        _chainring_tooth_profile(44),
        [_circle_profile(0.063, 96)],
        0.004,
        center=True,
    )
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, "toothed_chainring")


def _flat_pedal_mesh(name: str):
    outer = rounded_rect_profile(0.108, 0.070, 0.010, corner_segments=5)
    inner = rounded_rect_profile(0.058, 0.032, 0.006, corner_segments=5)
    geom = ExtrudeWithHolesGeometry(outer, [inner], 0.016, center=True)
    return mesh_from_geometry(geom, name)


def _tapered_arm_mesh(name: str, length: float = 0.170):
    """A flat tapered crank arm in local X/Z plane, extruded along local Y."""
    top_half_width = 0.014
    tip_half_width = 0.010
    y0 = 0.018
    y1 = length - 0.018
    profile = [
        (-top_half_width, y0),
        (top_half_width, y0),
        (tip_half_width, y1),
        (-tip_half_width, y1),
    ]
    geom = ExtrudeGeometry(profile, 0.012, center=True)
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_speed_urban_crankset")

    satin_black = model.material("satin_black", rgba=(0.01, 0.012, 0.014, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.05, 0.055, 0.060, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.003, 0.003, 0.004, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    shell = model.part("bottom_shell")
    shell.visual(
        _annular_mesh(0.030, 0.0135, 0.074, "bottom_shell_body"),
        material=dark_anodized,
        name="shell_body",
    )
    for y, nm in ((-0.041, "cup_0"), (0.041, "cup_1")):
        shell.visual(
            _annular_mesh(0.036, 0.0130, 0.010, f"bearing_{nm}"),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=bearing_black,
            name=nm,
        )

    crank = model.part("crank")
    # The spindle is visible at both ends and passes cleanly through the hollow shell.
    crank.visual(
        Cylinder(radius=0.010, length=0.168),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="spindle",
    )

    drive_y = 0.056
    non_drive_y = -0.056
    arm_len = 0.170

    crank.visual(
        _chainring_mesh(),
        origin=Origin(xyz=(0.0, 0.044, 0.0)),
        material=satin_black,
        name="chainring",
    )
    # Five simple spider arms tie the chainring to the drive-side hub.
    for i in range(5):
        a = 2.0 * math.pi * i / 5.0 + math.radians(18.0)
        radial_center = 0.046
        crank.visual(
            Box((0.074, 0.005, 0.012)),
            origin=Origin(
                xyz=(radial_center * math.cos(a), 0.046, radial_center * math.sin(a)),
                rpy=(0.0, -a, 0.0),
            ),
            material=satin_black,
            name=f"spider_{i}",
        )

    # Root and pedal-eye bosses: flat cylinders in the crank plane.
    crank.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, drive_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="drive_hub",
    )
    crank.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.0, non_drive_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="non_drive_hub",
    )
    crank.visual(
        _tapered_arm_mesh("drive_arm_mesh", arm_len),
        origin=Origin(xyz=(0.0, drive_y, 0.0)),
        material=satin_black,
        name="drive_arm",
    )
    crank.visual(
        _tapered_arm_mesh("non_drive_arm_mesh", arm_len),
        origin=Origin(xyz=(0.0, non_drive_y, 0.0), rpy=(0.0, math.pi, 0.0)),
        material=satin_black,
        name="non_drive_arm",
    )
    crank.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.0, drive_y, -arm_len), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="drive_boss",
    )
    crank.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.0, non_drive_y, arm_len), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="non_drive_boss",
    )

    def add_pedal(name: str, side_sign: float, joint_y: float, joint_z: float):
        pedal = model.part(name)
        # Threaded axle: a small inboard portion is intentionally seated in the arm eye.
        pedal.visual(
            Cylinder(radius=0.0055, length=0.056),
            origin=Origin(xyz=(0.0, side_sign * 0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name="axle",
        )
        pedal.visual(
            _flat_pedal_mesh(f"{name}_platform"),
            origin=Origin(xyz=(0.0, side_sign * 0.070, 0.0)),
            material=rubber,
            name="platform",
        )
        # Raised grip bars on top and bottom make the block read as a real platform pedal.
        for z, row in ((0.010, "top"), (-0.010, "bottom")):
            for x, col in ((-0.035, "front"), (0.0, "mid"), (0.035, "rear")):
                pedal.visual(
                    Box((0.010, 0.060, 0.004)),
                    origin=Origin(xyz=(x, side_sign * 0.070, z)),
                    material=bearing_black,
                    name=f"{row}_grip_{col}",
                )
        model.articulation(
            f"crank_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=crank,
            child=pedal,
            origin=Origin(xyz=(0.0, joint_y, joint_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=8.0),
        )
        return pedal

    add_pedal("pedal_0", 1.0, drive_y + 0.010, -arm_len)
    add_pedal("pedal_1", -1.0, non_drive_y - 0.010, arm_len)

    model.articulation(
        "shell_to_crank",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=crank,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=12.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crank = object_model.get_part("crank")
    shell = object_model.get_part("bottom_shell")
    pedal_0 = object_model.get_part("pedal_0")
    pedal_1 = object_model.get_part("pedal_1")
    shell_joint = object_model.get_articulation("shell_to_crank")
    pedal_joint_0 = object_model.get_articulation("crank_to_pedal_0")
    pedal_joint_1 = object_model.get_articulation("crank_to_pedal_1")

    ctx.check(
        "spindle and both pedals are continuous joints",
        shell_joint.articulation_type == ArticulationType.CONTINUOUS
        and pedal_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and pedal_joint_1.articulation_type == ArticulationType.CONTINUOUS,
    )

    ctx.expect_overlap(
        crank,
        shell,
        axes="y",
        elem_a="spindle",
        elem_b="shell_body",
        min_overlap=0.070,
        name="spindle runs through bottom bracket shell",
    )
    ctx.allow_overlap(
        shell,
        crank,
        elem_a="shell_body",
        elem_b="spindle",
        reason=(
            "The steel spindle is intentionally captured through the hollow bottom-bracket bore; "
            "the visual shell is annular but the mesh collision proxy reports the bore as filled."
        ),
    )
    ctx.expect_within(
        crank,
        shell,
        axes="xz",
        inner_elem="spindle",
        outer_elem="shell_body",
        margin=0.0,
        name="spindle is concentric within the round shell",
    )
    for cup_name in ("cup_0", "cup_1"):
        ctx.allow_overlap(
            shell,
            crank,
            elem_a=cup_name,
            elem_b="spindle",
            reason=(
                "The spindle passes through the hollow bearing cup bore; the annular visual mesh "
                "uses a conservative filled collision proxy."
            ),
        )
        ctx.expect_overlap(
            crank,
            shell,
            axes="y",
            elem_a="spindle",
            elem_b=cup_name,
            min_overlap=0.008,
            name=f"spindle passes through {cup_name}",
        )
    ctx.expect_overlap(
        crank,
        crank,
        axes="xz",
        elem_a="chainring",
        elem_b="drive_hub",
        min_overlap=0.015,
        name="chainring surrounds the drive hub",
    )

    # Pedal axles are threaded into the crank-eye bosses; the tiny local insertion
    # is intentional and scoped to the axle/boss pairs.
    ctx.allow_overlap(
        crank,
        pedal_0,
        elem_a="drive_boss",
        elem_b="axle",
        reason="The pedal axle is represented with a short threaded insertion into the crank eye.",
    )
    ctx.allow_overlap(
        crank,
        pedal_1,
        elem_a="non_drive_boss",
        elem_b="axle",
        reason="The pedal axle is represented with a short threaded insertion into the crank eye.",
    )
    ctx.expect_overlap(
        pedal_0,
        crank,
        axes="xz",
        elem_a="axle",
        elem_b="drive_boss",
        min_overlap=0.010,
        name="drive pedal axle is centered in its boss",
    )
    ctx.expect_gap(
        pedal_0,
        crank,
        axis="y",
        positive_elem="axle",
        negative_elem="drive_boss",
        max_penetration=0.008,
        max_gap=0.001,
        name="drive pedal axle has shallow threaded insertion",
    )
    ctx.expect_overlap(
        pedal_1,
        crank,
        axes="xz",
        elem_a="axle",
        elem_b="non_drive_boss",
        min_overlap=0.010,
        name="opposite pedal axle is centered in its boss",
    )
    ctx.expect_gap(
        crank,
        pedal_1,
        axis="y",
        positive_elem="non_drive_boss",
        negative_elem="axle",
        max_penetration=0.008,
        max_gap=0.001,
        name="opposite pedal axle has shallow threaded insertion",
    )

    rest_drive_pos = ctx.part_world_position(pedal_0)
    with ctx.pose({shell_joint: math.pi / 2.0}):
        quarter_drive_pos = ctx.part_world_position(pedal_0)
    ctx.check(
        "crank rotation carries the drive pedal around the spindle",
        rest_drive_pos is not None
        and quarter_drive_pos is not None
        and rest_drive_pos[2] < -0.15
        and quarter_drive_pos[0] < -0.15,
        details=f"rest={rest_drive_pos}, quarter={quarter_drive_pos}",
    )

    with ctx.pose({pedal_joint_0: math.pi / 2.0}):
        aabb = ctx.part_world_aabb(pedal_0)
    if aabb is not None:
        lo, hi = aabb
        ctx.check(
            "pedal platform rotates about its axle",
            (hi[2] - lo[2]) > 0.09 and (hi[0] - lo[0]) < 0.08,
            details=f"pedal_0 aabb={aabb}",
        )

    return ctx.report()


object_model = build_object_model()
