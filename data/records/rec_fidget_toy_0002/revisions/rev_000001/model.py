from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BODY_THICKNESS = 0.0074
HOUSING_RADIUS = 0.0135
LOBE_CENTER_RADIUS = 0.028
LOBE_OUTER_RADIUS = 0.016
ARM_LENGTH = 0.024
ARM_WIDTH = 0.0115
RELIEF_CENTER_RADIUS = 0.0195
RELIEF_RADIUS = 0.0105
BEARING_BORE_RADIUS = 0.0056
LOBE_WINDOW_RADIUS = 0.0085
LOBE_RING_OUTER_RADIUS = 0.0132
LOBE_RING_INNER_RADIUS = 0.0082
CENTER_RING_OUTER_RADIUS = 0.0114
CENTER_RING_INNER_RADIUS = 0.0058
EDGE_FILLET = 0.0016

BUTTON_RADIUS = 0.0113
BUTTON_SKIRT_RADIUS = 0.0095
BUTTON_THICKNESS = 0.0030
BUTTON_CLEARANCE = 0.00015
SLEEVE_RADIUS = 0.0039
BUTTON_OFFSET = BODY_THICKNESS / 2.0 + BUTTON_CLEARANCE
SLEEVE_LENGTH = 2.0 * BUTTON_OFFSET
BUTTON_FACE_THICKNESS = 0.0012
ACCENT_THICKNESS = BODY_THICKNESS * 0.94


def _angles() -> tuple[float, float, float]:
    return (0.0, 120.0, 240.0)


def _spinner_frame_shape() -> cq.Workplane:
    body = cq.Workplane("XY").circle(HOUSING_RADIUS).extrude(BODY_THICKNESS)

    for angle in _angles():
        lobe = (
            cq.Workplane("XY")
            .center(LOBE_CENTER_RADIUS, 0.0)
            .circle(LOBE_OUTER_RADIUS)
            .extrude(BODY_THICKNESS)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        arm = (
            cq.Workplane("XY")
            .center((HOUSING_RADIUS + LOBE_CENTER_RADIUS) * 0.5, 0.0)
            .rect(ARM_LENGTH, ARM_WIDTH)
            .extrude(BODY_THICKNESS)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        body = body.union(lobe).union(arm)

    for angle in (60.0, 180.0, 300.0):
        relief = (
            cq.Workplane("XY")
            .center(RELIEF_CENTER_RADIUS, 0.0)
            .circle(RELIEF_RADIUS)
            .extrude(BODY_THICKNESS + 0.002)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        body = body.cut(relief)

    body = body.faces(">Z").workplane().circle(BEARING_BORE_RADIUS).cutThruAll()

    for angle in _angles():
        lobe_window = (
            cq.Workplane("XY")
            .center(LOBE_CENTER_RADIUS, 0.0)
            .circle(LOBE_WINDOW_RADIUS)
            .extrude(BODY_THICKNESS + 0.002)
            .translate((0.0, 0.0, -0.001))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        body = body.cut(lobe_window)

    return body.translate((0.0, 0.0, -BODY_THICKNESS * 0.5))


def _spinner_accent_shape() -> cq.Workplane:
    accents = (
        cq.Workplane("XY")
        .circle(CENTER_RING_OUTER_RADIUS)
        .circle(CENTER_RING_INNER_RADIUS)
        .extrude(ACCENT_THICKNESS)
    )

    for angle in _angles():
        ring = (
            cq.Workplane("XY")
            .center(LOBE_CENTER_RADIUS, 0.0)
            .circle(LOBE_RING_OUTER_RADIUS)
            .circle(LOBE_RING_INNER_RADIUS)
            .extrude(ACCENT_THICKNESS)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        accents = accents.union(ring)

    return accents.translate((0.0, 0.0, -ACCENT_THICKNESS * 0.5))


def _button_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BUTTON_SKIRT_RADIUS).extrude(BUTTON_THICKNESS * 0.58)
    shoulder = (
        cq.Workplane("XY")
        .circle(BUTTON_RADIUS * 0.965)
        .extrude(BUTTON_THICKNESS * 0.22)
        .translate((0.0, 0.0, BUTTON_THICKNESS * 0.58))
    )
    cap = (
        cq.Workplane("XY")
        .circle(BUTTON_RADIUS)
        .extrude(BUTTON_FACE_THICKNESS)
        .translate((0.0, 0.0, BUTTON_THICKNESS - BUTTON_FACE_THICKNESS))
    )
    return base.union(shoulder).union(cap)


def _hub_shape() -> cq.Workplane:
    sleeve = (
        cq.Workplane("XY")
        .circle(SLEEVE_RADIUS)
        .extrude(SLEEVE_LENGTH)
        .translate((0.0, 0.0, -BUTTON_OFFSET))
    )
    top_button = _button_shape().translate((0.0, 0.0, BUTTON_OFFSET))
    bottom_button = (
        _button_shape()
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 180.0)
        .translate((0.0, 0.0, -BUTTON_OFFSET))
    )
    return sleeve.union(top_button).union(bottom_button)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_spinner", assets=ASSETS)

    model.material("frame_black", rgba=(0.11, 0.12, 0.14, 1.0))
    model.material("satin_graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("brushed_steel", rgba=(0.73, 0.74, 0.77, 1.0))

    hub = model.part("hub")
    hub.visual(
        mesh_from_cadquery(_hub_shape(), "hub.obj", assets=ASSETS),
        material="satin_graphite",
    )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=BUTTON_RADIUS, length=0.0138),
        mass=0.028,
    )

    spinner = model.part("spinner")
    spinner.visual(
        mesh_from_cadquery(_spinner_frame_shape(), "spinner_frame.obj", assets=ASSETS),
        material="frame_black",
    )
    spinner.visual(
        mesh_from_cadquery(_spinner_accent_shape(), "spinner_accents.obj", assets=ASSETS),
        material="brushed_steel",
    )
    spinner.inertial = Inertial.from_geometry(
        Cylinder(radius=0.044, length=BODY_THICKNESS),
        mass=0.112,
    )

    model.articulation(
        "hub_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=hub,
        child=spinner,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=60.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "hub",
        "spinner",
        reason="generated collision hulls around the nested bearing races can conservatively intersect",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("hub", "spinner", axes="xy", max_dist=0.001)
    ctx.expect_aabb_overlap("hub", "spinner", axes="xy", min_overlap=0.020)

    with ctx.pose(hub_to_spinner=0.7):
        ctx.expect_origin_distance("hub", "spinner", axes="xy", max_dist=0.001)
        ctx.expect_aabb_overlap("hub", "spinner", axes="xy", min_overlap=0.020)

    with ctx.pose(hub_to_spinner=1.4):
        ctx.expect_origin_distance("hub", "spinner", axes="xy", max_dist=0.001)
        ctx.expect_aabb_overlap("hub", "spinner", axes="xy", min_overlap=0.020)

    with ctx.pose(hub_to_spinner=2.1):
        ctx.expect_origin_distance("hub", "spinner", axes="xy", max_dist=0.001)
        ctx.expect_aabb_overlap("hub", "spinner", axes="xy", min_overlap=0.020)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
