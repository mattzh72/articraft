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


LINK_HEIGHT = 0.026
PLATE_THICKNESS = 0.008
BOSS_LENGTH = 0.020
LUG_RADIUS = 0.027
PIVOT_Z = 0.20
AXIS_Y = (0.0, -1.0, 0.0)


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder descriptor and transform for a pin/boss running along local Y."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_y_cylinder(part, *, name: str, x: float, y: float, z: float, radius: float, length: float, material: str) -> None:
    geometry, orient = _cyl_y(radius, length)
    part.visual(
        geometry,
        origin=Origin(xyz=(x, y, z), rpy=orient.rpy),
        material=material,
        name=name,
    )


def _add_link(part, *, length: float, half_width: float, material: str, hub_material: str) -> None:
    """Add a ladder-frame lever link whose local origin is its proximal pivot."""
    side_y = (-half_width, half_width)
    rail_length = length
    for idx, y in enumerate(side_y):
        part.visual(
            Box((rail_length, PLATE_THICKNESS, LINK_HEIGHT)),
            origin=Origin(xyz=(length / 2.0, y, 0.0)),
            material=material,
            name=f"side_plate_{idx}",
        )
        for x, end_name in ((0.0, "proximal"), (length, "distal")):
            _add_y_cylinder(
                part,
                name=f"{end_name}_boss_{idx}",
                x=x,
                y=y,
                z=0.0,
                radius=LUG_RADIUS,
                length=BOSS_LENGTH,
                material=hub_material,
            )
            # Dark face disks read as drilled pivot bores without forcing
            # inter-part pin overlap.
            face_y = y + (PLATE_THICKNESS * 0.95 if y > 0.0 else -PLATE_THICKNESS * 0.95)
            _add_y_cylinder(
                part,
                name=f"{end_name}_bore_{idx}",
                x=x,
                y=face_y,
                z=0.0,
                radius=0.009,
                length=0.0025,
                material="bore_shadow",
            )

    # Two narrow rungs keep the two side plates a single supported ladder frame
    # while preserving the open center visible between them.
    for idx, x in enumerate((length * 0.34, length * 0.70)):
        part.visual(
            Box((0.030, 2.0 * half_width + PLATE_THICKNESS, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=hub_material,
            name=f"cross_rung_{idx}",
        )


def _add_tip_tab(part, *, link_length: float) -> None:
    """Small bolted tab at the free end of the final lever."""
    tab_center_x = link_length + 0.055
    part.visual(
        Box((0.080, 0.034, 0.016)),
        origin=Origin(xyz=(link_length + 0.036, 0.0, 0.0)),
        material="tab_steel",
        name="end_tab_neck",
    )
    _add_y_cylinder(
        part,
        name="end_tab_eye",
        x=tab_center_x + 0.035,
        y=0.0,
        z=0.0,
        radius=0.022,
        length=0.034,
        material="tab_steel",
    )
    _add_y_cylinder(
        part,
        name="end_tab_hole",
        x=tab_center_x + 0.035,
        y=0.018,
        z=0.0,
        radius=0.007,
        length=0.0025,
        material="bore_shadow",
    )
    _add_y_cylinder(
        part,
        name="end_tab_hole_back",
        x=tab_center_x + 0.035,
        y=-0.018,
        z=0.0,
        radius=0.007,
        length=0.0025,
        material="bore_shadow",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_lever_chain")

    model.material("bracket_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("base_black", rgba=(0.055, 0.058, 0.065, 1.0))
    model.material("link_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    model.material("hub_steel", rgba=(0.44, 0.46, 0.48, 1.0))
    model.material("tab_steel", rgba=(0.74, 0.70, 0.58, 1.0))
    model.material("bore_shadow", rgba=(0.015, 0.016, 0.018, 1.0))

    root = model.part("root_bracket")
    root.visual(
        Box((0.22, 0.16, 0.035)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0175)),
        material="base_black",
        name="floor_plate",
    )
    root.visual(
        Box((0.095, 0.120, 0.025)),
        origin=Origin(xyz=(-0.015, 0.0, 0.0475)),
        material="bracket_dark",
        name="raised_pad",
    )
    # Grounded clevis cheeks straddle the first moving link but stay outside
    # its inboard side plates, giving the first revolute joint credible support.
    for idx, y in enumerate((-0.036, 0.036)):
        root.visual(
            Box((0.070, 0.014, 0.170)),
            origin=Origin(xyz=(0.0, y, 0.120)),
            material="bracket_dark",
            name=f"cheek_{idx}",
        )
        _add_y_cylinder(
            root,
            name=f"cheek_boss_{idx}",
            x=0.0,
            y=y,
            z=PIVOT_Z,
            radius=0.036,
            length=0.016,
            material="bracket_dark",
        )
        face_y = y + (0.009 if y > 0.0 else -0.009)
        _add_y_cylinder(
            root,
            name=f"cheek_bore_{idx}",
            x=0.0,
            y=face_y,
            z=PIVOT_Z,
            radius=0.011,
            length=0.004,
            material="bore_shadow",
        )
    root.visual(
        Box((0.026, 0.124, 0.085)),
        origin=Origin(xyz=(-0.046, 0.0, 0.090)),
        material="bracket_dark",
        name="rear_web",
    )
    # The grounded member is also drawn as a short ladder-frame link, making the
    # kinematic four-link chain visible: ground link plus three moving links.
    for idx, y in enumerate((-0.036, 0.036)):
        root.visual(
            Box((0.285, PLATE_THICKNESS, LINK_HEIGHT)),
            origin=Origin(xyz=(-0.1425, y, PIVOT_Z)),
            material="bracket_dark",
            name=f"ground_side_plate_{idx}",
        )
        _add_y_cylinder(
            root,
            name=f"ground_tail_boss_{idx}",
            x=-0.285,
            y=y,
            z=PIVOT_Z,
            radius=LUG_RADIUS,
            length=BOSS_LENGTH,
            material="bracket_dark",
        )
        face_y = y + (0.009 if y > 0.0 else -0.009)
        _add_y_cylinder(
            root,
            name=f"ground_tail_bore_{idx}",
            x=-0.285,
            y=face_y,
            z=PIVOT_Z,
            radius=0.009,
            length=0.004,
            material="bore_shadow",
        )
    root.visual(
        Box((0.030, 0.084, 0.014)),
        origin=Origin(xyz=(-0.155, 0.0, PIVOT_Z)),
        material="bracket_dark",
        name="ground_cross_rung",
    )

    link_0 = model.part("link_0")
    _add_link(link_0, length=0.40, half_width=0.018, material="link_steel", hub_material="hub_steel")

    link_1 = model.part("link_1")
    _add_link(link_1, length=0.33, half_width=0.038, material="link_steel", hub_material="hub_steel")

    tip_link = model.part("tip_link")
    _add_link(tip_link, length=0.27, half_width=0.018, material="link_steel", hub_material="hub_steel")
    _add_tip_tab(tip_link, link_length=0.27)

    model.articulation(
        "bracket_to_link_0",
        ArticulationType.REVOLUTE,
        parent=root,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=AXIS_Y,
        motion_limits=MotionLimits(lower=-0.70, upper=0.95, effort=30.0, velocity=1.5),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.40, 0.0, 0.0)),
        axis=AXIS_Y,
        motion_limits=MotionLimits(lower=-1.05, upper=1.05, effort=24.0, velocity=1.6),
    )
    model.articulation(
        "link_1_to_tip_link",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=tip_link,
        origin=Origin(xyz=(0.33, 0.0, 0.0)),
        axis=AXIS_Y,
        motion_limits=MotionLimits(lower=-1.10, upper=0.90, effort=18.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = list(object_model.articulations)
    ctx.check(
        "three serial revolute joints",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "joint axes are parallel",
        all(tuple(round(v, 4) for v in (j.axis or ())) == AXIS_Y for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    root = object_model.get_part("root_bracket")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    tip_link = object_model.get_part("tip_link")
    j0 = object_model.get_articulation("bracket_to_link_0")
    j1 = object_model.get_articulation("link_0_to_link_1")

    ctx.expect_origin_gap(link_0, root, axis="z", min_gap=0.16, max_gap=0.22, name="first pivot is elevated in bracket")
    ctx.expect_origin_distance(link_0, link_1, axes="xz", min_dist=0.395, max_dist=0.405, name="first link reaches second pivot")
    ctx.expect_origin_distance(link_1, tip_link, axes="xz", min_dist=0.325, max_dist=0.335, name="second link reaches third pivot")
    ctx.expect_overlap(root, link_0, axes="xz", min_overlap=0.045, name="root clevis surrounds first pivot")
    ctx.expect_overlap(link_0, link_1, axes="xz", min_overlap=0.045, name="middle joint lugs share a pivot")
    ctx.expect_overlap(link_1, tip_link, axes="xz", min_overlap=0.045, name="tip joint lugs share a pivot")

    rest_tip = ctx.part_world_position(tip_link)
    with ctx.pose({j0: 0.55, j1: 0.35}):
        raised_tip = ctx.part_world_position(tip_link)
    ctx.check(
        "positive revolute motion raises chain",
        rest_tip is not None and raised_tip is not None and raised_tip[2] > rest_tip[2] + 0.16,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
