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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _leaf_body_mesh(*, length: float, width: float):
    side_profile = [
        (0.0, 0.07),
        (length - 1.3, 0.07),
        (length - 0.5, 0.05),
        (length, 0.02),
        (length, -0.10),
        (length - 0.45, -0.16),
        (length - 3.5, -0.23),
        (length - 8.6, -0.35),
        (3.8, -0.54),
        (0.0, -0.72),
    ]
    body = ExtrudeGeometry.centered(side_profile, width, cap=True, closed=True)
    body.rotate_x(pi / 2.0)
    return mesh_from_geometry(body, "drawbridge_leaf_body")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    leaf_length = 18.0
    leaf_width = 7.8
    hinge_axis_z = 3.88

    concrete = model.material("concrete", rgba=(0.60, 0.61, 0.62, 1.0))
    steel = model.material("steel", rgba=(0.28, 0.31, 0.35, 1.0))
    deck_asphalt = model.material("deck_asphalt", rgba=(0.16, 0.17, 0.18, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.78, 0.66, 0.14, 1.0))
    bearing_dark = model.material("bearing_dark", rgba=(0.16, 0.17, 0.19, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((6.0, 9.8, 4.0)),
        origin=Origin(xyz=(-3.0, 0.0, 2.0)),
        material=concrete,
        name="abutment_block",
    )
    shore_frame.visual(
        Box((2.2, 8.6, 0.28)),
        origin=Origin(xyz=(-1.15, 0.0, 3.86)),
        material=concrete,
        name="apron_deck",
    )
    shore_frame.visual(
        Box((2.2, 0.34, 0.46)),
        origin=Origin(xyz=(-1.15, 4.13, 4.23)),
        material=concrete,
        name="left_apron_barrier",
    )
    shore_frame.visual(
        Box((2.2, 0.34, 0.46)),
        origin=Origin(xyz=(-1.15, -4.13, 4.23)),
        material=concrete,
        name="right_apron_barrier",
    )
    for side, y in (("left", 4.45), ("right", -4.45)):
        shore_frame.visual(
            Box((1.4, 0.90, 1.40)),
            origin=Origin(xyz=(-0.15, y, 4.70)),
            material=steel,
            name=f"{side}_bearing_pedestal",
        )
        shore_frame.visual(
            Cylinder(radius=0.42, length=0.52),
            origin=Origin(xyz=(0.0, y, hinge_axis_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bearing_dark,
            name=f"{side}_side_bearing",
        )
    shore_frame.inertial = Inertial.from_geometry(
        Box((6.0, 9.8, 4.0)),
        mass=380000.0,
        origin=Origin(xyz=(-3.0, 0.0, 2.0)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        _leaf_body_mesh(length=leaf_length, width=leaf_width),
        material=steel,
        name="steel_leaf",
    )
    bridge_leaf.visual(
        Box((17.5, 7.2, 0.05)),
        origin=Origin(xyz=(8.8, 0.0, 0.095)),
        material=deck_asphalt,
        name="road_deck",
    )
    bridge_leaf.visual(
        Box((17.1, 0.20, 0.18)),
        origin=Origin(xyz=(8.60, 3.55, 0.16)),
        material=safety_yellow,
        name="left_curb",
    )
    bridge_leaf.visual(
        Box((17.1, 0.20, 0.18)),
        origin=Origin(xyz=(8.60, -3.55, 0.16)),
        material=safety_yellow,
        name="right_curb",
    )
    bridge_leaf.visual(
        Box((0.42, 7.8, 0.34)),
        origin=Origin(xyz=(0.25, 0.0, -0.30)),
        material=steel,
        name="hinge_bulkhead",
    )
    bridge_leaf.visual(
        Box((0.70, 0.28, 0.62)),
        origin=Origin(xyz=(0.35, 3.76, -0.23)),
        material=steel,
        name="left_hinge_cheek",
    )
    bridge_leaf.visual(
        Box((0.70, 0.28, 0.62)),
        origin=Origin(xyz=(0.35, -3.76, -0.23)),
        material=steel,
        name="right_hinge_cheek",
    )
    bridge_leaf.visual(
        Box((0.24, 7.4, 0.16)),
        origin=Origin(xyz=(17.88, 0.0, -0.02)),
        material=steel,
        name="nose_beam",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((18.0, 7.8, 0.9)),
        mass=145000.0,
        origin=Origin(xyz=(9.0, 0.0, -0.25)),
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8_000_000.0,
            velocity=0.35,
            lower=0.0,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_joint = object_model.get_articulation("shore_to_leaf")

    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="y",
        elem_a="road_deck",
        elem_b="apron_deck",
        min_overlap=7.0,
        name="leaf roadway stays aligned with shore apron width",
    )
    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="x",
        positive_elem="steel_leaf",
        negative_elem="apron_deck",
        min_gap=0.0,
        max_gap=0.10,
        name="hinge side clearance stays tight when closed",
    )

    closed_nose = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")
    upper = leaf_joint.motion_limits.upper if leaf_joint.motion_limits is not None else None
    with ctx.pose({leaf_joint: upper if upper is not None else 1.0}):
        open_nose = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="nose_beam",
            negative_elem="apron_deck",
            min_gap=6.0,
            name="opened nose rises well above the shore deck",
        )

    closed_nose_z = None if closed_nose is None else 0.5 * (closed_nose[0][2] + closed_nose[1][2])
    open_nose_z = None if open_nose is None else 0.5 * (open_nose[0][2] + open_nose[1][2])
    ctx.check(
        "leaf opens upward about the shore bearings",
        closed_nose_z is not None and open_nose_z is not None and open_nose_z > closed_nose_z + 8.0,
        details=f"closed_nose_z={closed_nose_z}, open_nose_z={open_nose_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
