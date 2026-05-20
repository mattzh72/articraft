from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PANEL_SIZE = 0.95
PANEL_THICKNESS = 0.036
HOUSING_SIZE = 0.80
HOUSING_HEIGHT = 0.24
HANGER_RADIUS = 0.010
HANGER_HEIGHT = 0.070
HANGER_OFFSET = 0.285

FEATURE_RECESS_SIZE = 0.70
FEATURE_RECESS_DEPTH = 0.006

GRILLE_POCKET_SIZE = 0.44
GRILLE_THICKNESS = 0.006
CENTER_FRAME_OUTER = 0.60
CENTER_FRAME_STRIP = (CENTER_FRAME_OUTER - GRILLE_POCKET_SIZE) / 2.0
FRAME_BRIDGE_DEPTH = (FEATURE_RECESS_SIZE - CENTER_FRAME_OUTER) / 2.0

LOUVER_INNER_WIDTH = 0.105
LOUVER_OUTER_WIDTH = 0.155
LOUVER_DEPTH = 0.118
LOUVER_THICKNESS = 0.004
LOUVER_OPENING_CLEARANCE = 0.006
LOUVER_HINGE_Z = 0.0048
LOUVER_RADIUS = 0.210
LOUVER_LIMIT = 0.90

HATCH_WIDTH = 0.18
HATCH_DEPTH = 0.060
HATCH_THICKNESS = 0.007
HATCH_CLEARANCE = 0.002
HATCH_HINGE_Y = -0.417
HATCH_HINGE_Z = 0.0045
HATCH_LIMIT = 1.22

LOUVER_LAYOUT = (
    ("corner_louver_0", (LOUVER_RADIUS, LOUVER_RADIUS, LOUVER_HINGE_Z), math.pi / 4.0),
    ("corner_louver_1", (-LOUVER_RADIUS, LOUVER_RADIUS, LOUVER_HINGE_Z), 3.0 * math.pi / 4.0),
    ("corner_louver_2", (-LOUVER_RADIUS, -LOUVER_RADIUS, LOUVER_HINGE_Z), -3.0 * math.pi / 4.0),
    ("corner_louver_3", (LOUVER_RADIUS, -LOUVER_RADIUS, LOUVER_HINGE_Z), -math.pi / 4.0),
)


def _louver_outline(inner_width: float, outer_width: float, depth: float) -> list[tuple[float, float]]:
    return [
        (-inner_width / 2.0, 0.0),
        (inner_width / 2.0, 0.0),
        (outer_width / 2.0, depth),
        (-outer_width / 2.0, depth),
    ]


def _trapezoid_prism(
    outline: list[tuple[float, float]],
    *,
    height: float,
    z_offset: float,
) -> cq.Workplane:
    return cq.Workplane("XY").polyline(outline).close().extrude(height).translate((0.0, 0.0, z_offset))


def _build_louver_shape() -> cq.Workplane:
    blade = _trapezoid_prism(
        _louver_outline(LOUVER_INNER_WIDTH, LOUVER_OUTER_WIDTH, LOUVER_DEPTH),
        height=LOUVER_THICKNESS,
        z_offset=-LOUVER_THICKNESS / 2.0,
    )
    hinge_bead = cq.Workplane("XY").box(LOUVER_INNER_WIDTH * 0.88, 0.006, 0.006).translate((0.0, -0.003, 0.0))
    return blade.union(hinge_bead)


def _build_hatch_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(HATCH_WIDTH, HATCH_DEPTH, HATCH_THICKNESS).translate((0.0, HATCH_DEPTH / 2.0, 0.0))
    hinge_bead = cq.Workplane("XY").box(HATCH_WIDTH * 0.90, 0.007, 0.007).translate((0.0, 0.0035, 0.0))
    finger_relief = (
        cq.Workplane("YZ")
        .circle(0.010)
        .extrude(HATCH_WIDTH * 0.35)
        .translate((-HATCH_WIDTH * 0.175, HATCH_DEPTH - 0.004, 0.0005))
    )
    return panel.union(hinge_bead).cut(finger_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_cassette_air_conditioner")

    model.material("panel_white", rgba=(0.93, 0.94, 0.93, 1.0))
    model.material("grille_white", rgba=(0.89, 0.90, 0.89, 1.0))
    model.material("shadow_black", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("louver_grey", rgba=(0.84, 0.85, 0.84, 1.0))
    model.material("hanger_grey", rgba=(0.61, 0.63, 0.64, 1.0))

    body = model.part("cassette_body")
    body.visual(
        Box((HOUSING_SIZE, HOUSING_SIZE, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_THICKNESS + HOUSING_HEIGHT / 2.0)),
        material="shadow_black",
        name="housing_shell",
    )
    for index, (sx, sy) in enumerate(((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))):
        body.visual(
            Box((0.030, 0.030, 0.012)),
            origin=Origin(
                xyz=(
                    sx * HANGER_OFFSET,
                    sy * HANGER_OFFSET,
                    PANEL_THICKNESS + HOUSING_HEIGHT + 0.006,
                )
            ),
            material="hanger_grey",
            name=f"hanger_plate_{index}",
        )
        body.visual(
            Box((0.016, 0.016, HANGER_HEIGHT)),
            origin=Origin(
                xyz=(
                    sx * HANGER_OFFSET,
                    sy * HANGER_OFFSET,
                    PANEL_THICKNESS + HOUSING_HEIGHT + 0.012 + HANGER_HEIGHT / 2.0,
                )
            ),
            material="hanger_grey",
            name=f"hanger_rod_{index}",
        )

    panel_z = PANEL_THICKNESS / 2.0
    panel_recess_z = FEATURE_RECESS_DEPTH + (PANEL_THICKNESS - FEATURE_RECESS_DEPTH) / 2.0
    panel_recess_height = PANEL_THICKNESS - FEATURE_RECESS_DEPTH
    ring_band = (PANEL_SIZE - FEATURE_RECESS_SIZE) / 2.0
    hatch_total_width = HATCH_WIDTH + 2.0 * HATCH_CLEARANCE
    front_side_width = (FEATURE_RECESS_SIZE - hatch_total_width) / 2.0
    front_side_x = hatch_total_width / 2.0 + front_side_width / 2.0
    front_lip_rear_y = HATCH_HINGE_Y - HATCH_CLEARANCE
    front_lip_depth = front_lip_rear_y - (-PANEL_SIZE / 2.0)
    front_lip_center_y = (-PANEL_SIZE / 2.0 + front_lip_rear_y) / 2.0
    front_inner_depth = (-FEATURE_RECESS_SIZE / 2.0) - front_lip_rear_y
    front_inner_center_y = (front_lip_rear_y + (-FEATURE_RECESS_SIZE / 2.0)) / 2.0

    body.visual(
        Box((ring_band, PANEL_SIZE, PANEL_THICKNESS)),
        origin=Origin(xyz=(-(FEATURE_RECESS_SIZE + ring_band) / 2.0, 0.0, panel_z)),
        material="panel_white",
        name="panel_shell",
    )
    body.visual(
        Box((ring_band, PANEL_SIZE, PANEL_THICKNESS)),
        origin=Origin(xyz=((FEATURE_RECESS_SIZE + ring_band) / 2.0, 0.0, panel_z)),
        material="panel_white",
        name="panel_shell_right",
    )
    body.visual(
        Box((FEATURE_RECESS_SIZE, ring_band, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, (FEATURE_RECESS_SIZE + ring_band) / 2.0, panel_z)),
        material="panel_white",
        name="panel_shell_back",
    )
    body.visual(
        Box((FEATURE_RECESS_SIZE, front_lip_depth, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, front_lip_center_y, panel_z)),
        material="panel_white",
        name="panel_shell_front_lip",
    )
    body.visual(
        Box((front_side_width, front_inner_depth, PANEL_THICKNESS)),
        origin=Origin(xyz=(-front_side_x, front_inner_center_y, panel_z)),
        material="panel_white",
        name="panel_shell_front_left",
    )
    body.visual(
        Box((front_side_width, front_inner_depth, PANEL_THICKNESS)),
        origin=Origin(xyz=(front_side_x, front_inner_center_y, panel_z)),
        material="panel_white",
        name="panel_shell_front_right",
    )

    center_strip_center = GRILLE_POCKET_SIZE / 2.0 + CENTER_FRAME_STRIP / 2.0
    center_strip_span = CENTER_FRAME_OUTER - 2.0 * CENTER_FRAME_STRIP
    bridge_center = CENTER_FRAME_OUTER / 2.0 + FRAME_BRIDGE_DEPTH / 2.0
    bridge_span = CENTER_FRAME_OUTER - 2.0 * CENTER_FRAME_STRIP
    body.visual(
        Box((center_strip_span, CENTER_FRAME_STRIP, panel_recess_height)),
        origin=Origin(xyz=(0.0, center_strip_center, panel_recess_z)),
        material="panel_white",
        name="center_frame_top",
    )
    body.visual(
        Box((center_strip_span, CENTER_FRAME_STRIP, panel_recess_height)),
        origin=Origin(xyz=(0.0, -center_strip_center, panel_recess_z)),
        material="panel_white",
        name="center_frame_bottom",
    )
    body.visual(
        Box((CENTER_FRAME_STRIP, center_strip_span, panel_recess_height)),
        origin=Origin(xyz=(-center_strip_center, 0.0, panel_recess_z)),
        material="panel_white",
        name="center_frame_left",
    )
    body.visual(
        Box((CENTER_FRAME_STRIP, center_strip_span, panel_recess_height)),
        origin=Origin(xyz=(center_strip_center, 0.0, panel_recess_z)),
        material="panel_white",
        name="center_frame_right",
    )
    body.visual(
        Box((bridge_span, FRAME_BRIDGE_DEPTH, panel_recess_height)),
        origin=Origin(xyz=(0.0, bridge_center, panel_recess_z)),
        material="panel_white",
        name="bridge_top",
    )
    body.visual(
        Box((bridge_span, FRAME_BRIDGE_DEPTH, panel_recess_height)),
        origin=Origin(xyz=(0.0, -bridge_center, panel_recess_z)),
        material="panel_white",
        name="bridge_bottom",
    )
    body.visual(
        Box((FRAME_BRIDGE_DEPTH, bridge_span, panel_recess_height)),
        origin=Origin(xyz=(-bridge_center, 0.0, panel_recess_z)),
        material="panel_white",
        name="bridge_left",
    )
    body.visual(
        Box((FRAME_BRIDGE_DEPTH, bridge_span, panel_recess_height)),
        origin=Origin(xyz=(bridge_center, 0.0, panel_recess_z)),
        material="panel_white",
        name="bridge_right",
    )

    grille = model.part("intake_grille")
    grille.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (GRILLE_POCKET_SIZE, GRILLE_POCKET_SIZE),
                GRILLE_THICKNESS,
                slot_size=(0.052, 0.007),
                pitch=(0.068, 0.017),
                frame=0.016,
                corner_radius=0.008,
                stagger=True,
            ),
            "cassette_intake_grille",
        ),
        origin=Origin(xyz=(0.0, 0.0, GRILLE_THICKNESS / 2.0)),
        material="grille_white",
        name="grille_face",
    )
    model.articulation(
        "body_to_intake_grille",
        ArticulationType.FIXED,
        parent=body,
        child=grille,
        origin=Origin(),
    )

    louver_mesh = mesh_from_cadquery(_build_louver_shape(), "corner_louver_blade")
    for part_name, xyz, yaw in LOUVER_LAYOUT:
        louver = model.part(part_name)
        louver.visual(
            louver_mesh,
            material="louver_grey",
            name="blade",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=louver,
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, yaw)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.5,
                lower=0.0,
                upper=LOUVER_LIMIT,
            ),
        )

    hatch = model.part("service_hatch")
    hatch.visual(
        mesh_from_cadquery(_build_hatch_shape(), "service_hatch_panel"),
        origin=Origin(xyz=(0.0, 0.0, HATCH_HINGE_Z)),
        material="panel_white",
        name="hatch_panel",
    )
    model.articulation(
        "body_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(0.0, HATCH_HINGE_Y - HATCH_CLEARANCE, HATCH_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.3,
            lower=0.0,
            upper=HATCH_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cassette_body")
    grille = object_model.get_part("intake_grille")
    hatch = object_model.get_part("service_hatch")
    hatch_joint = object_model.get_articulation("body_to_service_hatch")

    ctx.expect_origin_distance(
        grille,
        body,
        axes="xy",
        max_dist=0.02,
        name="intake grille stays centered in the cassette panel",
    )
    ctx.expect_origin_gap(
        grille,
        hatch,
        axis="y",
        min_gap=0.30,
        name="service hatch sits near the front edge away from the central intake grille",
    )

    closed_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({hatch_joint: HATCH_LIMIT}):
        open_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    ctx.check(
        "service hatch drops downward",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[0][2] < closed_hatch_aabb[0][2] - 0.05,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    for index, (part_name, _, _) in enumerate(LOUVER_LAYOUT):
        louver = object_model.get_part(part_name)
        joint = object_model.get_articulation(f"body_to_{part_name}")
        ctx.expect_origin_distance(
            louver,
            grille,
            axes="xy",
            min_dist=0.24,
            max_dist=0.38,
            name=f"{part_name} sits in a corner discharge zone",
        )
        closed_aabb = ctx.part_element_world_aabb(louver, elem="blade")
        with ctx.pose({joint: LOUVER_LIMIT}):
            open_aabb = ctx.part_element_world_aabb(louver, elem="blade")
        ctx.check(
            f"{part_name} opens downward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][2] < closed_aabb[0][2] - 0.04,
            details=f"louver_index={index}, closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
