from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LINK_LENGTH = 0.28
LINK_WIDTH = 0.055
LINK_THICKNESS = 0.024
LAYER_OFFSET = LINK_THICKNESS
STOW_ANGLE = math.radians(150.0)


def _cylinder_along_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_boxed_link(part, *, layer_z: float, material, accent, include_terminal: bool = False) -> None:
    """Add a short boxed boom link whose part frame is at its proximal pivot."""

    rail_w = 0.012
    web_len = 0.064
    rail_len = LINK_LENGTH - web_len
    rail_y = LINK_WIDTH / 2.0 - rail_w / 2.0

    for idx, y in enumerate((-rail_y, rail_y)):
        part.visual(
            Box((rail_len, rail_w, LINK_THICKNESS)),
            origin=Origin(xyz=(LINK_LENGTH / 2.0, y, layer_z)),
            material=material,
            name=f"side_rail_{idx}",
        )

    for name, x in (("proximal_web", 0.0), ("distal_web", LINK_LENGTH)):
        part.visual(
            Box((web_len, LINK_WIDTH, LINK_THICKNESS)),
            origin=Origin(xyz=(x, 0.0, layer_z)),
            material=material,
            name=name,
        )
        washer_z = layer_z + (LINK_THICKNESS / 2.0 + 0.0016 if layer_z > 0.0 else -LINK_THICKNESS / 2.0 - 0.0016)
        part.visual(
            Cylinder(radius=0.025, length=0.004),
            origin=Origin(xyz=(x, 0.0, washer_z)),
            material=accent,
            name=f"{name}_washer",
        )

    if include_terminal:
        fork_z = layer_z
        part.visual(
            Box((0.048, 0.082, 0.026)),
            origin=Origin(xyz=(LINK_LENGTH + 0.037, 0.0, fork_z)),
            material=material,
            name="terminal_bridge",
        )
        for idx, y in enumerate((-0.030, 0.030)):
            part.visual(
                Box((0.105, 0.022, 0.024)),
                origin=Origin(xyz=(LINK_LENGTH + 0.092, y, fork_z)),
                material=material,
                name=f"terminal_tine_{idx}",
            )
            part.visual(
                Cylinder(radius=0.008, length=0.003),
                origin=Origin(xyz=(LINK_LENGTH + 0.122, y, fork_z + 0.013)),
                material=accent,
                name=f"terminal_bore_mark_{idx}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_fold_out_boom")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    orange = model.material("safety_orange", rgba=(0.95, 0.42, 0.08, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.48, 0.50, 0.48, 1.0))

    base = model.part("base_plate")
    base.visual(
        Box((0.30, 0.18, 0.025)),
        origin=Origin(xyz=(0.030, 0.0, -0.075)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Box((0.095, 0.115, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=dark_steel,
        name="raised_boss",
    )
    for idx, y in enumerate((-0.0365, 0.0365)):
        base.visual(
            Box((0.070, 0.018, 0.095)),
            origin=Origin(xyz=(0.0, y, -0.010)),
            material=dark_steel,
            name=f"clevis_cheek_{idx}",
        )
        cap_geom, cap_origin = _cylinder_along_y(radius=0.020, length=0.006)
        base.visual(
            cap_geom,
            origin=Origin(xyz=(0.0, y + math.copysign(0.012, y), 0.0), rpy=cap_origin.rpy),
            material=warm_grey,
            name=f"base_pin_cap_{idx}",
        )

    for idx, (x, y) in enumerate(((-0.075, -0.055), (-0.075, 0.055), (0.125, -0.055), (0.125, 0.055))):
        base.visual(
            Cylinder(radius=0.010, length=0.002),
            origin=Origin(xyz=(x, y, -0.062)),
            material=satin_black,
            name=f"mount_hole_{idx}",
        )

    links = []
    for i, layer_z in enumerate((0.0, LAYER_OFFSET, 0.0, LAYER_OFFSET)):
        link = model.part(f"link_{i}")
        _add_boxed_link(
            link,
            layer_z=layer_z,
            material=orange,
            accent=warm_grey,
            include_terminal=(i == 3),
        )
        links.append(link)

    joint_limits = MotionLimits(effort=12.0, velocity=2.0, lower=-1.35, upper=1.35)
    model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "fold_joint_0",
        ArticulationType.REVOLUTE,
        parent=links[0],
        child=links[1],
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0), rpy=(0.0, 0.0, STOW_ANGLE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-STOW_ANGLE, upper=0.08),
    )
    model.articulation(
        "fold_joint_1",
        ArticulationType.REVOLUTE,
        parent=links[1],
        child=links[2],
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0), rpy=(0.0, 0.0, -STOW_ANGLE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.08, upper=STOW_ANGLE),
    )
    model.articulation(
        "fold_joint_2",
        ArticulationType.REVOLUTE,
        parent=links[2],
        child=links[3],
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0), rpy=(0.0, 0.0, STOW_ANGLE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-STOW_ANGLE, upper=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [object_model.get_articulation(name) for name in ("base_pivot", "fold_joint_0", "fold_joint_1", "fold_joint_2")]
    links = [object_model.get_part(f"link_{i}") for i in range(4)]

    ctx.check(
        "four serial revolute joints",
        len(object_model.articulations) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "planar vertical joint axes",
        all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    stowed = [ctx.part_world_position(link) for link in links]
    ctx.check(
        "stowed origins zigzag",
        all(p is not None for p in stowed)
        and stowed[1][0] > stowed[2][0] + 0.18
        and stowed[2][1] > stowed[0][1] + 0.10
        and stowed[3][0] > stowed[2][0] + 0.18,
        details=f"stowed={stowed}",
    )

    open_pose = {
        joints[0]: 0.0,
        joints[1]: -STOW_ANGLE,
        joints[2]: STOW_ANGLE,
        joints[3]: -STOW_ANGLE,
    }
    with ctx.pose(open_pose):
        opened = [ctx.part_world_position(link) for link in links]
        terminal_aabb = ctx.part_element_world_aabb(links[3], elem="terminal_tine_0")
        ctx.check(
            "opened boom forms long reach",
            all(p is not None for p in opened)
            and opened[3][0] > opened[0][0] + 0.80
            and max(abs(p[1] - opened[0][1]) for p in opened) < 0.004
            and terminal_aabb is not None
            and terminal_aabb[1][0] > 1.20,
            details=f"opened={opened}, terminal_aabb={terminal_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
