from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _freezer_cabinet_shell() -> cq.Workplane:
    """Rounded insulated tub with a real top opening and bottom floor."""
    outer_x, outer_y, outer_z = 1.45, 0.75, 0.60
    inner_x, inner_y = 1.23, 0.53
    body = (
        cq.Workplane("XY")
        .box(outer_x, outer_y, outer_z)
        .translate((0.0, 0.0, outer_z / 2.0))
        .edges("|Z")
        .fillet(0.035)
    )
    pocket = (
        cq.Workplane("XY")
        .box(inner_x, inner_y, 0.54)
        .translate((0.0, 0.0, 0.39))
    )
    return body.cut(pocket)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convenience_freezer")

    white = model.material("white_powder_coat", rgba=(0.92, 0.94, 0.93, 1.0))
    dark = model.material("dark_liner", rgba=(0.03, 0.035, 0.04, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.65, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.55, 0.82, 0.95, 0.38))
    black = model.material("black_plastic", rgba=(0.01, 0.012, 0.014, 1.0))
    chrome = model.material("chrome_lock", rgba=(0.80, 0.82, 0.78, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_freezer_cabinet_shell(), "cabinet_shell", tolerance=0.002),
        material=white,
        name="insulated_shell",
    )
    cabinet.visual(
        Box((1.18, 0.48, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
        material=dark,
        name="recessed_liner_floor",
    )

    # Twin long tracks on the top rim: a lower and an upper sliding channel.
    for y, suffix in ((0.312, "front"), (-0.312, "rear")):
        cabinet.visual(
            Box((1.36, 0.040, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.606)),
            material=aluminum,
            name=f"{suffix}_rail_base",
        )
        cabinet.visual(
            Box((1.36, 0.020, 0.044)),
            origin=Origin(xyz=(0.0, y, 0.622)),
            material=aluminum,
            name=f"{suffix}_lower_track_lip",
        )
        cabinet.visual(
            Box((1.36, 0.026, 0.064)),
            origin=Origin(xyz=(0.0, y + (0.035 if y > 0 else -0.035), 0.632)),
            material=aluminum,
            name=f"{suffix}_outer_track_wall",
        )
        cabinet.visual(
            Box((1.36, 0.026, 0.008)),
            origin=Origin(xyz=(0.0, y + (0.011 if y > 0 else -0.011), 0.668)),
            material=aluminum,
            name=f"{suffix}_upper_track_lip",
        )

    # A side-mounted lock barrel and hinge leaf for the rotating dust/lock flap.
    cabinet.visual(
        Cylinder(radius=0.023, length=0.028),
        origin=Origin(xyz=(0.50, 0.389, 0.400), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="key_cylinder",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.50, 0.404, 0.400), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="key_slot",
    )
    cabinet.visual(
        Box((0.205, 0.025, 0.035)),
        origin=Origin(xyz=(0.50, 0.3865, 0.470)),
        material=chrome,
        name="lock_hinge_leaf",
    )

    def add_sliding_lid(
        name: str,
        *,
        closed_x: float,
        center_z: float,
        axis: tuple[float, float, float],
        handle_x: float,
    ):
        lid = model.part(name)
        lid.visual(
            Box((0.610, 0.585, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
            material=glass,
            name="glass_pane",
        )
        lid.visual(
            Box((0.640, 0.020, 0.017)),
            origin=Origin(xyz=(0.0, 0.300, 0.0)),
            material=aluminum,
            name="front_side_frame",
        )
        lid.visual(
            Box((0.640, 0.020, 0.017)),
            origin=Origin(xyz=(0.0, -0.300, 0.0)),
            material=aluminum,
            name="rear_side_frame",
        )
        lid.visual(
            Box((0.020, 0.600, 0.017)),
            origin=Origin(xyz=(0.320, 0.0, 0.0)),
            material=aluminum,
            name="end_frame_0",
        )
        lid.visual(
            Box((0.020, 0.600, 0.017)),
            origin=Origin(xyz=(-0.320, 0.0, 0.0)),
            material=aluminum,
            name="end_frame_1",
        )
        lid.visual(
            Box((0.145, 0.030, 0.018)),
            origin=Origin(xyz=(handle_x, -0.210, 0.0125)),
            material=black,
            name="finger_pull",
        )
        return lid

    lid_0 = add_sliding_lid(
        "top_lid_0",
        closed_x=-0.325,
        center_z=0.6525,
        axis=(1.0, 0.0, 0.0),
        handle_x=0.195,
    )
    lid_1 = add_sliding_lid(
        "top_lid_1",
        closed_x=0.325,
        center_z=0.6805,
        axis=(-1.0, 0.0, 0.0),
        handle_x=-0.195,
    )

    model.articulation(
        "cabinet_to_lid_0",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_0,
        origin=Origin(xyz=(-0.325, 0.0, 0.6525)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.42),
    )
    model.articulation(
        "cabinet_to_lid_1",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_1,
        origin=Origin(xyz=(0.325, 0.0, 0.6805)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.42),
    )

    flap = model.part("lock_flap")
    flap.visual(
        Box((0.160, 0.006, 0.120)),
        origin=Origin(xyz=(0.0, 0.007, -0.062)),
        material=chrome,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.008, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="flap_hinge_barrel",
    )
    flap.visual(
        Box((0.120, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.005, -0.012)),
        material=chrome,
        name="hinge_web",
    )
    flap.visual(
        Box((0.070, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.011, -0.118)),
        material=black,
        name="flap_pull_lip",
    )
    model.articulation(
        "cabinet_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(0.50, 0.407, 0.470)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid_0 = object_model.get_part("top_lid_0")
    lid_1 = object_model.get_part("top_lid_1")
    flap = object_model.get_part("lock_flap")
    lid_0_slide = object_model.get_articulation("cabinet_to_lid_0")
    lid_1_slide = object_model.get_articulation("cabinet_to_lid_1")
    flap_hinge = object_model.get_articulation("cabinet_to_lock_flap")

    ctx.expect_contact(
        lid_0,
        cabinet,
        elem_a="front_side_frame",
        elem_b="front_lower_track_lip",
        contact_tol=0.001,
        name="lower glass lid sits on the lower rail",
    )
    ctx.expect_contact(
        lid_1,
        cabinet,
        elem_a="front_side_frame",
        elem_b="front_upper_track_lip",
        contact_tol=0.001,
        name="upper glass lid sits on the upper rail",
    )
    ctx.expect_gap(
        lid_1,
        lid_0,
        axis="z",
        positive_elem="front_side_frame",
        negative_elem="front_side_frame",
        min_gap=0.008,
        name="stacked sliding lids have vertical track clearance",
    )

    ctx.expect_gap(
        flap,
        cabinet,
        axis="y",
        positive_elem="flap_panel",
        negative_elem="key_cylinder",
        min_gap=0.005,
        max_gap=0.012,
        name="closed lock flap stands just proud of the key cylinder",
    )
    ctx.expect_overlap(
        flap,
        cabinet,
        axes="xz",
        elem_a="flap_panel",
        elem_b="key_cylinder",
        min_overlap=0.040,
        name="closed lock flap covers the key cylinder in side view",
    )
    ctx.expect_contact(
        flap,
        cabinet,
        elem_a="flap_hinge_barrel",
        elem_b="lock_hinge_leaf",
        contact_tol=0.001,
        name="lock flap hinge is mounted on the side leaf",
    )

    rest_lid_0 = ctx.part_world_position(lid_0)
    rest_lid_1 = ctx.part_world_position(lid_1)
    rest_flap_box = ctx.part_element_world_aabb(flap, elem="flap_panel")

    with ctx.pose({lid_0_slide: 0.42, lid_1_slide: 0.42, flap_hinge: 1.20}):
        open_lid_0 = ctx.part_world_position(lid_0)
        open_lid_1 = ctx.part_world_position(lid_1)
        open_flap_box = ctx.part_element_world_aabb(flap, elem="flap_panel")
        ctx.expect_gap(
            lid_1,
            lid_0,
            axis="z",
            positive_elem="front_side_frame",
            negative_elem="front_side_frame",
            min_gap=0.008,
            name="slid-over glass panels remain separated by their rails",
        )

    ctx.check(
        "glass sliders move in opposite directions along their tracks",
        rest_lid_0 is not None
        and rest_lid_1 is not None
        and open_lid_0 is not None
        and open_lid_1 is not None
        and open_lid_0[0] > rest_lid_0[0] + 0.35
        and open_lid_1[0] < rest_lid_1[0] - 0.35,
        details=f"rest=({rest_lid_0}, {rest_lid_1}), open=({open_lid_0}, {open_lid_1})",
    )

    def box_center_yz(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    rest_flap_yz = box_center_yz(rest_flap_box)
    open_flap_yz = box_center_yz(open_flap_box)
    ctx.check(
        "lock flap rotates outward and upward",
        rest_flap_yz is not None
        and open_flap_yz is not None
        and open_flap_yz[0] > rest_flap_yz[0] + 0.040
        and open_flap_yz[1] > rest_flap_yz[1] + 0.030,
        details=f"rest={rest_flap_yz}, open={open_flap_yz}",
    )

    return ctx.report()


object_model = build_object_model()
