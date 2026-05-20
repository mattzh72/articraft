from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)


BASE_LEN = 0.30
BASE_W = 0.09
BASE_T = 0.012

SHAFT_LEN = 0.24
SHAFT_R = 0.010
SHAFT_AXIS_Z = 0.055

SUPPORT_X = 0.103
SUPPORT_T = 0.018
SUPPORT_W = 0.064
SUPPORT_H = 0.078

BEARING_LEN = 0.050
BEARING_W = 0.060
BEARING_BASE_H = 0.020
BEARING_CROWN_R = 0.019

BUSHING_INNER_R = 0.0098
BUSHING_OUTER_R = 0.0152

HUB_LEN = 0.030
HUB_R = 0.018
HUB_X = -0.060

FLANGE_T = 0.008
FLANGE_R = 0.027
FLANGE_X = 0.060

SET_SCREW_SIZE = (0.006, 0.010, 0.006)


def _support_shape() -> cq.Workplane:
    support = (
        cq.Workplane("XY")
        .box(SUPPORT_T, SUPPORT_W, SUPPORT_H)
        .translate((0.0, 0.0, -SHAFT_AXIS_Z + SUPPORT_H / 2.0))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(BUSHING_OUTER_R)
        .extrude(SUPPORT_T * 1.8, both=True)
    )
    return support.cut(bore)


def _bushing_shape(length: float) -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .circle(BUSHING_OUTER_R)
        .extrude(length / 2.0, both=True)
    )
    inner = (
        cq.Workplane("YZ")
        .circle(BUSHING_INNER_R)
        .extrude(length * 0.6, both=True)
    )
    return outer.cut(inner)


def _bearing_block_shape() -> cq.Workplane:
    pedestal = (
        cq.Workplane("XY")
        .box(BEARING_LEN, BEARING_W, BEARING_BASE_H)
        .translate((0.0, 0.0, -SHAFT_AXIS_Z + BEARING_BASE_H / 2.0))
    )
    saddle = (
        cq.Workplane("XY")
        .box(BEARING_LEN, BEARING_W, SHAFT_AXIS_Z - BEARING_CROWN_R + 0.004)
        .translate((0.0, 0.0, (-SHAFT_AXIS_Z + BEARING_CROWN_R - 0.004) / 2.0))
    )
    crown = (
        cq.Workplane("YZ")
        .circle(BEARING_CROWN_R)
        .extrude(BEARING_LEN / 2.0, both=True)
    )
    bore = (
        cq.Workplane("YZ")
        .circle(BUSHING_OUTER_R)
        .extrude(BEARING_LEN * 0.8, both=True)
    )
    return pedestal.union(saddle).union(crown).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_rotary_shaft_module", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.34, 0.36, 0.38, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.40, 0.43, 0.47, 1.0))
    bronze = model.material("bronze", rgba=(0.72, 0.58, 0.32, 1.0))
    oxide = model.material("oxide", rgba=(0.16, 0.17, 0.18, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_LEN, BASE_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, -SHAFT_AXIS_Z + BASE_T / 2.0)),
        material=cast_iron,
        name="base_plate",
    )
    frame.visual(
        mesh_from_cadquery(_support_shape(), "left_support.obj", assets=ASSETS),
        origin=Origin(xyz=(-SUPPORT_X, 0.0, 0.0)),
        material=cast_iron,
        name="left_support",
    )
    frame.visual(
        mesh_from_cadquery(_support_shape(), "right_support.obj", assets=ASSETS),
        origin=Origin(xyz=(SUPPORT_X, 0.0, 0.0)),
        material=cast_iron,
        name="right_support",
    )
    frame.visual(
        mesh_from_cadquery(_bearing_block_shape(), "bearing_block.obj", assets=ASSETS),
        material=cast_iron,
        name="bearing_block",
    )
    frame.visual(
        mesh_from_cadquery(_bushing_shape(SUPPORT_T), "left_bushing.obj", assets=ASSETS),
        origin=Origin(xyz=(-SUPPORT_X, 0.0, 0.0)),
        material=bronze,
        name="left_bushing",
    )
    frame.visual(
        mesh_from_cadquery(_bushing_shape(BEARING_LEN), "center_bushing.obj", assets=ASSETS),
        material=bronze,
        name="center_bushing",
    )
    frame.visual(
        mesh_from_cadquery(_bushing_shape(SUPPORT_T), "right_bushing.obj", assets=ASSETS),
        origin=Origin(xyz=(SUPPORT_X, 0.0, 0.0)),
        material=bronze,
        name="right_bushing",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_W, SUPPORT_H)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, -SHAFT_AXIS_Z + SUPPORT_H / 2.0)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_LEN),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft_body",
    )
    shaft.visual(
        Cylinder(radius=HUB_R, length=HUB_LEN),
        origin=Origin(xyz=(HUB_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    shaft.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_T),
        origin=Origin(xyz=(FLANGE_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="flange",
    )
    shaft.visual(
        Box(SET_SCREW_SIZE),
        origin=Origin(xyz=(HUB_X, HUB_R + SET_SCREW_SIZE[1] / 2.0, 0.0)),
        material=oxide,
        name="set_screw",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAFT_R, length=SHAFT_LEN),
        mass=0.75,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    shaft = object_model.get_part("shaft")
    spin = object_model.get_articulation("frame_to_shaft_spin")

    base_plate = frame.get_visual("base_plate")
    left_support = frame.get_visual("left_support")
    right_support = frame.get_visual("right_support")
    left_bushing = frame.get_visual("left_bushing")
    center_bushing = frame.get_visual("center_bushing")
    right_bushing = frame.get_visual("right_bushing")

    shaft_body = shaft.get_visual("shaft_body")
    hub = shaft.get_visual("hub")
    flange = shaft.get_visual("flange")
    set_screw = shaft.get_visual("set_screw")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        shaft,
        frame,
        elem_a=shaft_body,
        elem_b=left_bushing,
        reason="Left bearing sleeve is an intentional journal-bearing fit around the rotating shaft.",
    )
    ctx.allow_overlap(
        shaft,
        frame,
        elem_a=shaft_body,
        elem_b=center_bushing,
        reason="Center bearing sleeve intentionally nests around the shaft to represent a supported running fit.",
    )
    ctx.allow_overlap(
        shaft,
        frame,
        elem_a=shaft_body,
        elem_b=right_bushing,
        reason="Right bearing sleeve is an intentional journal-bearing fit around the rotating shaft.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=10)

    ctx.expect_origin_distance(
        shaft,
        frame,
        axes="yz",
        max_dist=0.001,
        name="shaft_axis_centered_in_module",
    )
    ctx.expect_gap(
        shaft,
        frame,
        axis="z",
        min_gap=0.032,
        max_gap=0.034,
        positive_elem=shaft_body,
        negative_elem=base_plate,
        name="shaft_carried_above_base_plate",
    )
    ctx.expect_overlap(
        shaft,
        frame,
        axes="yz",
        min_overlap=0.020,
        elem_a=shaft_body,
        elem_b=left_bushing,
        name="left_support_bushing_surrounds_shaft",
    )
    ctx.expect_overlap(
        shaft,
        frame,
        axes="yz",
        min_overlap=0.020,
        elem_a=shaft_body,
        elem_b=center_bushing,
        name="center_bearing_surrounds_shaft",
    )
    ctx.expect_overlap(
        shaft,
        frame,
        axes="yz",
        min_overlap=0.020,
        elem_a=shaft_body,
        elem_b=right_bushing,
        name="right_support_bushing_surrounds_shaft",
    )
    ctx.expect_contact(
        shaft,
        frame,
        elem_a=shaft_body,
        elem_b=left_bushing,
        name="left_bushing_contacts_shaft",
    )
    ctx.expect_contact(
        shaft,
        frame,
        elem_a=shaft_body,
        elem_b=center_bushing,
        name="center_bearing_contacts_shaft",
    )
    ctx.expect_contact(
        shaft,
        frame,
        elem_a=shaft_body,
        elem_b=right_bushing,
        name="right_bushing_contacts_shaft",
    )
    ctx.expect_gap(
        shaft,
        frame,
        axis="x",
        min_gap=0.016,
        max_gap=0.030,
        positive_elem=hub,
        negative_elem=left_support,
        name="hub_clears_left_support",
    )
    ctx.expect_gap(
        frame,
        shaft,
        axis="x",
        min_gap=0.028,
        max_gap=0.032,
        positive_elem=right_support,
        negative_elem=flange,
        name="flange_clears_right_support",
    )

    with ctx.pose({spin: 0.0}):
        ctx.expect_gap(
            shaft,
            frame,
            axis="z",
            min_gap=0.038,
            max_gap=0.045,
            positive_elem=set_screw,
            negative_elem=base_plate,
            name="set_screw_rest_height",
        )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_gap(
            shaft,
            frame,
            axis="z",
            min_gap=0.056,
            max_gap=0.064,
            positive_elem=set_screw,
            negative_elem=base_plate,
            name="set_screw_rises_after_quarter_turn",
        )
        ctx.expect_overlap(
            shaft,
            frame,
            axes="yz",
            min_overlap=0.020,
            elem_a=shaft_body,
            elem_b=center_bushing,
            name="shaft_stays_centered_in_bearing_at_quarter_turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
