from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)

PANEL_SIZE = 0.95
PANEL_THICKNESS = 0.045
PANEL_CORNER_RADIUS = 0.012

CENTRAL_RECESS_SIZE = 0.46
CENTRAL_RECESS_DEPTH = 0.016

SLOT_LONG = 0.50
SLOT_RADIAL = 0.125
SLOT_OUTER_EDGE = PANEL_SIZE * 0.5 - 0.105
SLOT_INNER_EDGE = SLOT_OUTER_EDGE - SLOT_RADIAL
HINGE_OFFSET = 0.012
HINGE_Z = 0.006

FLAP_BLADE_LENGTH = SLOT_LONG - 0.008
FLAP_BLADE_DEPTH = SLOT_RADIAL - 0.006
FLAP_BLADE_THICKNESS = 0.015
FLAP_BARREL_RADIUS = 0.008
FLAP_BARREL_CLEARANCE = 0.012
FLAP_BLADE_Z = -0.0135
FLAP_OPEN_ANGLE = 1.10

GRILLE_SIZE = 0.40
GRILLE_FACE_THICKNESS = 0.004
GRILLE_DEPTH = 0.015

PLENUM_SIZE = 0.66
PLENUM_HEIGHT = 0.17

DIAL_DIAMETER = 0.030
DIAL_HEIGHT = 0.012
DIAL_OFFSET = PANEL_SIZE * 0.5 - 0.118

# Numeric flap names keep the object orientation-neutral while still giving each
# independently articulated perimeter vane a stable identity.
FLAP_SPECS = (
    {
        "part": "flap_0",
        "joint": "panel_to_flap_0",
        "axis_alignment": "x",
        "leaf_sign": -1.0,
        "origin": (0.0, SLOT_OUTER_EDGE - HINGE_OFFSET, HINGE_Z),
        "axis": (1.0, 0.0, 0.0),
    },
    {
        "part": "flap_1",
        "joint": "panel_to_flap_1",
        "axis_alignment": "y",
        "leaf_sign": -1.0,
        "origin": (SLOT_OUTER_EDGE - HINGE_OFFSET, 0.0, HINGE_Z),
        "axis": (0.0, -1.0, 0.0),
    },
    {
        "part": "flap_2",
        "joint": "panel_to_flap_2",
        "axis_alignment": "x",
        "leaf_sign": 1.0,
        "origin": (0.0, -SLOT_OUTER_EDGE + HINGE_OFFSET, HINGE_Z),
        "axis": (-1.0, 0.0, 0.0),
    },
    {
        "part": "flap_3",
        "joint": "panel_to_flap_3",
        "axis_alignment": "y",
        "leaf_sign": 1.0,
        "origin": (-SLOT_OUTER_EDGE + HINGE_OFFSET, 0.0, HINGE_Z),
        "axis": (0.0, 1.0, 0.0),
    },
)


def _slot_cutter(size_x: float, size_y: float, center_x: float, center_y: float) -> cq.Workplane:
    cut_depth = PANEL_THICKNESS + 0.010
    return cq.Workplane("XY").box(size_x, size_y, cut_depth).translate(
        (center_x, center_y, PANEL_THICKNESS * 0.5)
    )


def _build_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(PANEL_SIZE, PANEL_SIZE, PANEL_THICKNESS).translate(
        (0.0, 0.0, PANEL_THICKNESS * 0.5)
    )
    panel = panel.edges("|Z").fillet(PANEL_CORNER_RADIUS)

    recess = cq.Workplane("XY").box(
        CENTRAL_RECESS_SIZE,
        CENTRAL_RECESS_SIZE,
        CENTRAL_RECESS_DEPTH + 0.004,
    ).translate((0.0, 0.0, CENTRAL_RECESS_DEPTH * 0.5 - 0.002))
    panel = panel.cut(recess)

    panel = panel.cut(_slot_cutter(SLOT_LONG, SLOT_RADIAL, 0.0, (SLOT_INNER_EDGE + SLOT_OUTER_EDGE) * 0.5))
    panel = panel.cut(_slot_cutter(SLOT_LONG, SLOT_RADIAL, 0.0, -(SLOT_INNER_EDGE + SLOT_OUTER_EDGE) * 0.5))
    panel = panel.cut(_slot_cutter(SLOT_RADIAL, SLOT_LONG, (SLOT_INNER_EDGE + SLOT_OUTER_EDGE) * 0.5, 0.0))
    panel = panel.cut(_slot_cutter(SLOT_RADIAL, SLOT_LONG, -(SLOT_INNER_EDGE + SLOT_OUTER_EDGE) * 0.5, 0.0))

    return panel


def _add_flap_geometry(model: ArticulatedObject, part_name: str, *, axis_alignment: str, leaf_sign: float) -> None:
    flap = model.part(part_name)

    if axis_alignment == "x":
        flap.visual(
            Cylinder(radius=FLAP_BARREL_RADIUS, length=FLAP_BLADE_LENGTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material="panel_white",
            name="barrel",
        )
        flap.visual(
            Box((FLAP_BLADE_LENGTH, FLAP_BLADE_DEPTH, FLAP_BLADE_THICKNESS)),
            origin=Origin(
                xyz=(
                    0.0,
                    leaf_sign * (FLAP_BLADE_DEPTH * 0.5 - FLAP_BARREL_CLEARANCE),
                    FLAP_BLADE_Z,
                )
            ),
            material="panel_white",
            name="blade",
        )
        rib_size = (0.040, FLAP_BLADE_DEPTH * 0.32, FLAP_BLADE_THICKNESS * 0.65)
        rib_positions = (
            (-FLAP_BLADE_LENGTH * 0.34, leaf_sign * FLAP_BLADE_DEPTH * 0.22, FLAP_BLADE_Z + 0.002),
            (FLAP_BLADE_LENGTH * 0.34, leaf_sign * FLAP_BLADE_DEPTH * 0.22, FLAP_BLADE_Z + 0.002),
        )
    else:
        flap.visual(
            Cylinder(radius=FLAP_BARREL_RADIUS, length=FLAP_BLADE_LENGTH),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="panel_white",
            name="barrel",
        )
        flap.visual(
            Box((FLAP_BLADE_DEPTH, FLAP_BLADE_LENGTH, FLAP_BLADE_THICKNESS)),
            origin=Origin(
                xyz=(
                    leaf_sign * (FLAP_BLADE_DEPTH * 0.5 - FLAP_BARREL_CLEARANCE),
                    0.0,
                    FLAP_BLADE_Z,
                )
            ),
            material="panel_white",
            name="blade",
        )
        rib_size = (FLAP_BLADE_DEPTH * 0.32, 0.040, FLAP_BLADE_THICKNESS * 0.65)
        rib_positions = (
            (leaf_sign * FLAP_BLADE_DEPTH * 0.22, -FLAP_BLADE_LENGTH * 0.34, FLAP_BLADE_Z + 0.002),
            (leaf_sign * FLAP_BLADE_DEPTH * 0.22, FLAP_BLADE_LENGTH * 0.34, FLAP_BLADE_Z + 0.002),
        )

    for index, rib_position in enumerate(rib_positions):
        flap.visual(
            Box(rib_size),
            origin=Origin(xyz=rib_position),
            material="panel_shadow",
            name=f"rib_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cassette_air_conditioner")

    model.material("panel_white", rgba=(0.93, 0.94, 0.92, 1.0))
    model.material("panel_shadow", rgba=(0.73, 0.76, 0.77, 1.0))
    model.material("grille_grey", rgba=(0.66, 0.69, 0.71, 1.0))
    model.material("cavity_dark", rgba=(0.17, 0.19, 0.21, 1.0))
    model.material("dial_grey", rgba=(0.80, 0.82, 0.83, 1.0))
    model.material("dial_mark", rgba=(0.23, 0.28, 0.31, 1.0))

    panel = model.part("panel")
    panel.visual(
        mesh_from_cadquery(_build_panel_shape(), "cassette_panel"),
        material="panel_white",
        name="fascia",
    )
    panel.visual(
        Box((PLENUM_SIZE, PLENUM_SIZE, PLENUM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_THICKNESS + PLENUM_HEIGHT * 0.5)),
        material="cavity_dark",
        name="plenum",
    )

    panel.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (GRILLE_SIZE, GRILLE_SIZE),
                frame=0.014,
                face_thickness=GRILLE_FACE_THICKNESS,
                duct_depth=GRILLE_DEPTH,
                duct_wall=0.003,
                slat_pitch=0.018,
                slat_width=0.008,
                slat_angle_deg=10.0,
                corner_radius=0.008,
                slats=VentGrilleSlats(
                    profile="flat",
                    direction="down",
                    inset=0.002,
                    divider_count=5,
                    divider_width=0.004,
                ),
                frame_profile=VentGrilleFrame(style="radiused", depth=0.0015),
                sleeve=VentGrilleSleeve(style="none"),
                center=False,
            ),
            "cassette_center_grille",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="grille_grey",
        name="grille_face",
    )
    panel.visual(
        Box((GRILLE_SIZE + 0.032, GRILLE_SIZE + 0.032, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, CENTRAL_RECESS_DEPTH - 0.003)),
        material="grille_grey",
        name="mount_frame",
    )

    for flap_spec in FLAP_SPECS:
        _add_flap_geometry(
            model,
            flap_spec["part"],
            axis_alignment=flap_spec["axis_alignment"],
            leaf_sign=flap_spec["leaf_sign"],
        )
        model.articulation(
            flap_spec["joint"],
            ArticulationType.REVOLUTE,
            parent=panel,
            child=flap_spec["part"],
            origin=Origin(xyz=flap_spec["origin"]),
            axis=flap_spec["axis"],
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=1.2,
                lower=0.0,
                upper=FLAP_OPEN_ANGLE,
            ),
        )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                DIAL_DIAMETER,
                DIAL_HEIGHT,
                body_style="cylindrical",
                side_draft_deg=6.0,
                edge_radius=0.0012,
                center=False,
            ),
            "cassette_selector_dial",
        ),
        origin=Origin(rpy=(math.pi, 0.0, 0.0)),
        material="dial_grey",
        name="dial_body",
    )
    model.articulation(
        "panel_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=panel,
        child=selector_dial,
        origin=Origin(xyz=(DIAL_OFFSET, DIAL_OFFSET, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    panel = object_model.get_part("panel")
    selector_dial = object_model.get_part("selector_dial")
    selector_joint = object_model.get_articulation("panel_to_selector_dial")

    panel_aabb = ctx.part_world_aabb(panel)
    if panel_aabb is None:
        ctx.fail("panel_aabb_available", "Expected the cassette panel to have a world-space AABB.")
        return ctx.report()

    panel_min, panel_max = panel_aabb
    panel_dx = float(panel_max[0] - panel_min[0])
    panel_dy = float(panel_max[1] - panel_min[1])
    ctx.check(
        "panel_matches_commercial_ceiling_grid_scale",
        abs(panel_dx - PANEL_SIZE) < 0.01 and abs(panel_dy - PANEL_SIZE) < 0.01,
        details=f"dx={panel_dx:.4f}, dy={panel_dy:.4f}",
    )

    fascia_aabb = ctx.part_element_world_aabb(panel, elem="fascia")
    grille_aabb = ctx.part_element_world_aabb(panel, elem="grille_face")
    grille_centered = False
    grille_within = False
    grille_details = f"fascia={fascia_aabb}, grille={grille_aabb}"
    if fascia_aabb is not None and grille_aabb is not None:
        fascia_min, fascia_max = fascia_aabb
        grille_min, grille_max = grille_aabb
        fascia_center = (
            (fascia_min[0] + fascia_max[0]) * 0.5,
            (fascia_min[1] + fascia_max[1]) * 0.5,
        )
        grille_center = (
            (grille_min[0] + grille_max[0]) * 0.5,
            (grille_min[1] + grille_max[1]) * 0.5,
        )
        grille_centered = (
            abs(grille_center[0] - fascia_center[0]) < 0.001
            and abs(grille_center[1] - fascia_center[1]) < 0.001
        )
        grille_within = (
            grille_min[0] >= fascia_min[0] + 0.03
            and grille_max[0] <= fascia_max[0] - 0.03
            and grille_min[1] >= fascia_min[1] + 0.03
            and grille_max[1] <= fascia_max[1] - 0.03
        )
        grille_details = (
            f"fascia_center={fascia_center}, grille_center={grille_center}, "
            f"fascia={fascia_aabb}, grille={grille_aabb}"
        )
    ctx.check("center_grille_is_centered_in_panel", grille_centered, details=grille_details)
    ctx.check("center_grille_stays_inside_fascia_footprint", grille_within, details=grille_details)

    ctx.expect_overlap(
        selector_dial,
        panel,
        axes="xy",
        min_overlap=0.020,
        elem_a="dial_body",
        elem_b="fascia",
        name="selector_dial_overlaps_corner_of_panel",
    )
    ctx.expect_gap(
        panel,
        selector_dial,
        axis="z",
        positive_elem="fascia",
        negative_elem="dial_body",
        max_gap=0.002,
        max_penetration=0.0001,
        name="selector_dial_mounts_flush_to_panel_underside",
    )
    ctx.check(
        "selector_dial_uses_continuous_joint",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and selector_joint.motion_limits is not None
        and selector_joint.motion_limits.lower is None
        and selector_joint.motion_limits.upper is None,
        details=f"joint_type={selector_joint.articulation_type}, limits={selector_joint.motion_limits}",
    )

    for flap_spec in FLAP_SPECS:
        flap = object_model.get_part(flap_spec["part"])
        flap_joint = object_model.get_articulation(flap_spec["joint"])

        ctx.expect_gap(
            panel,
            flap,
            axis="z",
            positive_elem="fascia",
            negative_elem="blade",
            max_gap=0.003,
            max_penetration=0.0001,
            name=f"{flap_spec['part']}_sits_flush_to_slot_edge",
        )

        rest_aabb = ctx.part_element_world_aabb(flap, elem="blade")
        with ctx.pose({flap_joint: FLAP_OPEN_ANGLE}):
            opened_aabb = ctx.part_element_world_aabb(flap, elem="blade")

        opens_downward = (
            rest_aabb is not None
            and opened_aabb is not None
            and opened_aabb[0][2] < rest_aabb[0][2] - 0.045
        )
        ctx.check(
            f"{flap_spec['part']}_opens_downward",
            opens_downward,
            details=f"rest={rest_aabb}, opened={opened_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
