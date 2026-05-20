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
    mesh_from_cadquery,
)
import cadquery as cq


HOOD_WIDTH = 0.92
HOOD_DEPTH = 0.52
CANOPY_HEIGHT = 0.30
FILTER_WIDTH = 0.62
FILTER_DEPTH = 0.31
FILTER_THICKNESS = 0.012
HINGE_Y = -0.155
HINGE_Z = -0.015
ROCKER_XS = (-0.12, 0.0, 0.12)


def _rect_wire(width: float, depth: float, center_y: float, z: float) -> cq.Wire:
    """Closed rectangle wire in world XYZ used for the hood's tapered lofts."""
    hw = width / 2.0
    hd = depth / 2.0
    pts = [
        cq.Vector(-hw, center_y - hd, z),
        cq.Vector(hw, center_y - hd, z),
        cq.Vector(hw, center_y + hd, z),
        cq.Vector(-hw, center_y + hd, z),
        cq.Vector(-hw, center_y - hd, z),
    ]
    return cq.Wire.makePolygon(pts)


def _canopy_shell() -> cq.Workplane:
    """A hollow stainless frustum: broad underside tapering into the chimney."""
    outer = cq.Solid.makeLoft(
        [
            _rect_wire(HOOD_WIDTH, HOOD_DEPTH, 0.02, 0.0),
            _rect_wire(0.34, 0.18, -0.15, CANOPY_HEIGHT),
        ],
        True,
    )
    inner = cq.Solid.makeLoft(
        [
            _rect_wire(0.70, 0.34, 0.01, -0.025),
            _rect_wire(0.22, 0.10, -0.15, CANOPY_HEIGHT + 0.035),
        ],
        True,
    )
    return cq.Workplane("XY").add(outer).cut(cq.Workplane("XY").add(inner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_chimney_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.69, 1.0))
    dark_metal = model.material("dark_filter_metal", rgba=(0.06, 0.065, 0.07, 1.0))
    black = model.material("black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    white = model.material("white_marking", rgba=(0.92, 0.92, 0.86, 1.0))
    shadow = model.material("deep_intake_shadow", rgba=(0.015, 0.017, 0.018, 1.0))

    hood = model.part("hood")
    hood.visual(
        mesh_from_cadquery(_canopy_shell(), "canopy_shell", tolerance=0.0012),
        material=stainless,
        name="canopy_shell",
    )
    hood.visual(
        Box((0.32, 0.16, 0.90)),
        origin=Origin(xyz=(0.0, -0.18, 0.75)),
        material=stainless,
        name="chimney_stack",
    )
    hood.visual(
        Box((0.44, 0.024, 1.08)),
        origin=Origin(xyz=(0.0, -0.272, 0.66)),
        material=stainless,
        name="wall_mount_plate",
    )
    hood.visual(
        Box((0.36, 0.19, 0.026)),
        origin=Origin(xyz=(0.0, -0.17, 0.313)),
        material=stainless,
        name="chimney_base_collar",
    )
    # A front control strip, proud of the canopy lip and tied into the shell.
    hood.visual(
        Box((0.84, 0.040, 0.082)),
        origin=Origin(xyz=(0.0, 0.292, 0.035)),
        material=stainless,
        name="control_strip",
    )
    # Dark recess visible behind the closed filter door through the intake opening.
    hood.visual(
        Box((0.74, 0.388, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, -0.0085)),
        material=shadow,
        name="intake_shadow",
    )
    # Underside rectangular frame around the hinged intake/filter door.
    hood.visual(
        Box((0.72, 0.028, 0.014)),
        origin=Origin(xyz=(0.0, -0.184, -0.007)),
        material=stainless,
        name="intake_frame_rear",
    )
    hood.visual(
        Box((0.64, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, -0.166, -0.015)),
        material=stainless,
        name="hinge_leaf",
    )
    hood.visual(
        Box((0.72, 0.028, 0.014)),
        origin=Origin(xyz=(0.0, 0.204, -0.007)),
        material=stainless,
        name="intake_frame_front",
    )
    for x, name in ((-0.374, "intake_frame_side_0"), (0.374, "intake_frame_side_1")):
        hood.visual(
            Box((0.028, 0.388, 0.014)),
            origin=Origin(xyz=(x, 0.010, -0.007)),
            material=stainless,
            name=name,
        )

    # Rocker switch side bearings molded into the fixed front control strip.
    for i, sx in enumerate(ROCKER_XS):
        hood.visual(
            Box((0.006, 0.010, 0.046)),
            origin=Origin(xyz=(sx - 0.037, 0.316, 0.035)),
            material=black,
            name=f"rocker_bearing_{i}_0",
        )
        hood.visual(
            Box((0.006, 0.010, 0.046)),
            origin=Origin(xyz=(sx + 0.037, 0.316, 0.035)),
            material=black,
            name=f"rocker_bearing_{i}_1",
        )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Cylinder(radius=0.006, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    filter_door.visual(
        Box((FILTER_WIDTH, FILTER_DEPTH, FILTER_THICKNESS)),
        origin=Origin(xyz=(0.0, FILTER_DEPTH / 2.0, -0.0055)),
        material=dark_metal,
        name="filter_panel",
    )
    # Raised longitudinal mesh ribs on the underside of the filter panel.
    for i, x in enumerate((-0.24, -0.16, -0.08, 0.0, 0.08, 0.16, 0.24)):
        filter_door.visual(
            Box((0.016, FILTER_DEPTH - 0.048, 0.002)),
            origin=Origin(xyz=(x, FILTER_DEPTH / 2.0, -0.0125)),
            material=stainless,
            name=f"filter_rib_{i}",
        )
    filter_door.visual(
        Box((0.22, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, FILTER_DEPTH + 0.006, -0.011)),
        material=stainless,
        name="pull_lip",
    )
    model.articulation(
        "hood_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=hood,
        child=filter_door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.18, effort=8.0, velocity=1.0),
    )

    for i, sx in enumerate(ROCKER_XS):
        rocker = model.part(f"rocker_{i}")
        rocker.visual(
            Cylinder(radius=0.004, length=0.068),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name="pivot_pin",
        )
        rocker.visual(
            Box((0.055, 0.014, 0.036)),
            origin=Origin(xyz=(0.0, 0.009, 0.0)),
            material=black,
            name="rocker_cap",
        )
        rocker.visual(
            Box((0.028, 0.0015, 0.003)),
            origin=Origin(xyz=(0.0, 0.0166, 0.010)),
            material=white,
            name="on_mark",
        )
        rocker.visual(
            Box((0.018, 0.0015, 0.003)),
            origin=Origin(xyz=(0.0, 0.0166, -0.010)),
            material=white,
            name="off_mark",
        )
        model.articulation(
            f"hood_to_rocker_{i}",
            ArticulationType.REVOLUTE,
            parent=hood,
            child=rocker,
            origin=Origin(xyz=(sx, 0.316, 0.035)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-0.26, upper=0.26, effort=0.7, velocity=4.0),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood = object_model.get_part("hood")
    filter_door = object_model.get_part("filter_door")
    door_hinge = object_model.get_articulation("hood_to_filter_door")

    ctx.expect_gap(
        hood,
        filter_door,
        axis="z",
        positive_elem="intake_frame_front",
        negative_elem="filter_panel",
        min_gap=0.0,
        max_gap=0.0015,
        name="closed filter door is flush under intake frame",
    )
    ctx.expect_overlap(
        hood,
        filter_door,
        axes="xy",
        elem_a="intake_shadow",
        elem_b="filter_panel",
        min_overlap=0.25,
        name="filter door covers rectangular intake opening",
    )

    closed_panel = ctx.part_element_world_aabb(filter_door, elem="filter_panel")
    with ctx.pose({door_hinge: 1.18}):
        opened_panel = ctx.part_element_world_aabb(filter_door, elem="filter_panel")
    ctx.check(
        "filter door opens downward from underside hinge",
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[0][2] < closed_panel[0][2] - 0.18,
        details=f"closed={closed_panel}, opened={opened_panel}",
    )

    for i in range(len(ROCKER_XS)):
        joint = object_model.get_articulation(f"hood_to_rocker_{i}")
        rocker = object_model.get_part(f"rocker_{i}")
        limits = joint.motion_limits
        ctx.check(
            f"rocker {i} has transverse pivot limits",
            tuple(round(v, 3) for v in joint.axis) == (1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower < 0.0 < limits.upper
            and limits.upper <= 0.35,
            details=f"axis={joint.axis}, limits={limits}",
        )
        neutral = ctx.part_element_world_aabb(rocker, elem="on_mark")
        with ctx.pose({joint: limits.upper}):
            tilted = ctx.part_element_world_aabb(rocker, elem="on_mark")
        ctx.check(
            f"rocker {i} visibly tilts on its pivot",
            neutral is not None
            and tilted is not None
            and tilted[1][2] > neutral[1][2] + 0.0015,
            details=f"neutral={neutral}, tilted={tilted}",
        )

    return ctx.report()


object_model = build_object_model()
