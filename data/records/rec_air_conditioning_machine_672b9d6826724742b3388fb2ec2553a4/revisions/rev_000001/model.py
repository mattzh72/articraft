from __future__ import annotations

import cadquery as cq

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


OUTER_PANEL = 0.95
PANEL_THICKNESS = 0.022
OUTER_CORNER_RADIUS = 0.030

GRILLE_SIZE = 0.58
GRILLE_RECESS = 0.006
GRILLE_BORDER = 0.030
GRILLE_SLOT_LENGTH = GRILLE_SIZE - 2.0 * GRILLE_BORDER
GRILLE_SLOT_WIDTH = 0.014
GRILLE_SLOT_PITCH = 0.026

DISCHARGE_SLOT_WIDTH = 0.095
DISCHARGE_SLOT_LENGTH = 0.480
SLOT_OUTER_EDGE = GRILLE_SIZE / 2.0 + DISCHARGE_SLOT_WIDTH
SLOT_INNER_EDGE = GRILLE_SIZE / 2.0

FLAP_THICKNESS = 0.006
FLAP_WIDTH = 0.086
FLAP_LENGTH = 0.468
FLAP_OPEN_ANGLE = 0.95

INSPECTION_SIZE_X = 0.054
INSPECTION_SIZE_Y = 0.112
INSPECTION_THICKNESS = 0.004
INSPECTION_POCKET_DEPTH = 0.0046
INSPECTION_GAP = 0.0
INSPECTION_CENTER_X = 0.428
INSPECTION_CENTER_Y = -0.150
INSPECTION_OPEN_ANGLE = 1.10
HINGE_BARREL_RADIUS = 0.0022
HINGE_BARREL_LENGTH = 0.042


def _rect_prism(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _build_panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(OUTER_PANEL, OUTER_PANEL, PANEL_THICKNESS)
        .edges("|Z")
        .fillet(OUTER_CORNER_RADIUS)
        .translate((0.0, 0.0, -PANEL_THICKNESS / 2.0))
    )

    recessed_center = _rect_prism(
        (GRILLE_SIZE, GRILLE_SIZE, GRILLE_RECESS + 0.001),
        (0.0, 0.0, -(GRILLE_RECESS + 0.001) / 2.0),
    )
    panel = panel.cut(recessed_center)

    discharge_cuts = (
        (DISCHARGE_SLOT_LENGTH, DISCHARGE_SLOT_WIDTH, PANEL_THICKNESS + 0.004, 0.0, SLOT_OUTER_EDGE - DISCHARGE_SLOT_WIDTH / 2.0),
        (DISCHARGE_SLOT_LENGTH, DISCHARGE_SLOT_WIDTH, PANEL_THICKNESS + 0.004, 0.0, -SLOT_OUTER_EDGE + DISCHARGE_SLOT_WIDTH / 2.0),
        (DISCHARGE_SLOT_WIDTH, DISCHARGE_SLOT_LENGTH, PANEL_THICKNESS + 0.004, SLOT_OUTER_EDGE - DISCHARGE_SLOT_WIDTH / 2.0, 0.0),
        (DISCHARGE_SLOT_WIDTH, DISCHARGE_SLOT_LENGTH, PANEL_THICKNESS + 0.004, -SLOT_OUTER_EDGE + DISCHARGE_SLOT_WIDTH / 2.0, 0.0),
    )
    for sx, sy, sz, cx, cy in discharge_cuts:
        panel = panel.cut(_rect_prism((sx, sy, sz), (cx, cy, -PANEL_THICKNESS / 2.0)))

    grille_slot_extent = GRILLE_SIZE - 2.0 * GRILLE_BORDER
    x = -grille_slot_extent / 2.0 + GRILLE_SLOT_PITCH / 2.0
    x_limit = grille_slot_extent / 2.0 - GRILLE_SLOT_PITCH / 2.0
    while x <= x_limit + 1e-9:
        panel = panel.cut(
            _rect_prism(
                (GRILLE_SLOT_WIDTH, GRILLE_SLOT_LENGTH, PANEL_THICKNESS + 0.004),
                (x, 0.0, -PANEL_THICKNESS / 2.0),
            )
        )
        x += GRILLE_SLOT_PITCH

    panel = panel.cut(
        _rect_prism(
            (
                INSPECTION_SIZE_X + INSPECTION_GAP,
                INSPECTION_SIZE_Y + INSPECTION_GAP,
                INSPECTION_POCKET_DEPTH,
            ),
            (
                INSPECTION_CENTER_X,
                INSPECTION_CENTER_Y,
                -INSPECTION_POCKET_DEPTH / 2.0,
            ),
        )
    )

    return panel


def _add_flap(
    model: ArticulatedObject,
    *,
    name: str,
    hinge_origin: tuple[float, float, float],
    axis: tuple[float, float, float],
    size: tuple[float, float, float],
    visual_origin: Origin,
) -> None:
    flap = model.part(name)
    flap.visual(
        Box(size),
        origin=visual_origin,
        material="louver_white",
        name="flap_surface",
    )
    model.articulation(
        f"panel_to_{name}",
        ArticulationType.REVOLUTE,
        parent="panel",
        child=flap,
        origin=Origin(xyz=hinge_origin),
        axis=axis,
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=FLAP_OPEN_ANGLE,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cassette_air_conditioner")

    model.material("panel_white", rgba=(0.95, 0.95, 0.93, 1.0))
    model.material("louver_white", rgba=(0.88, 0.88, 0.85, 1.0))
    model.material("louver_shadow", rgba=(0.70, 0.70, 0.67, 1.0))

    panel = model.part("panel")
    panel.visual(
        mesh_from_cadquery(_build_panel_shape(), "cassette_panel"),
        material="panel_white",
        name="panel_shell",
    )

    _add_flap(
        model,
        name="north_flap",
        hinge_origin=(0.0, SLOT_OUTER_EDGE, FLAP_THICKNESS),
        axis=(-1.0, 0.0, 0.0),
        size=(FLAP_LENGTH, FLAP_WIDTH, FLAP_THICKNESS),
        visual_origin=Origin(xyz=(0.0, -FLAP_WIDTH / 2.0, -FLAP_THICKNESS / 2.0)),
    )
    _add_flap(
        model,
        name="south_flap",
        hinge_origin=(0.0, -SLOT_OUTER_EDGE, FLAP_THICKNESS),
        axis=(1.0, 0.0, 0.0),
        size=(FLAP_LENGTH, FLAP_WIDTH, FLAP_THICKNESS),
        visual_origin=Origin(xyz=(0.0, FLAP_WIDTH / 2.0, -FLAP_THICKNESS / 2.0)),
    )
    _add_flap(
        model,
        name="east_flap",
        hinge_origin=(SLOT_OUTER_EDGE, 0.0, FLAP_THICKNESS),
        axis=(0.0, 1.0, 0.0),
        size=(FLAP_WIDTH, FLAP_LENGTH, FLAP_THICKNESS),
        visual_origin=Origin(xyz=(-FLAP_WIDTH / 2.0, 0.0, -FLAP_THICKNESS / 2.0)),
    )
    _add_flap(
        model,
        name="west_flap",
        hinge_origin=(-SLOT_OUTER_EDGE, 0.0, FLAP_THICKNESS),
        axis=(0.0, -1.0, 0.0),
        size=(FLAP_WIDTH, FLAP_LENGTH, FLAP_THICKNESS),
        visual_origin=Origin(xyz=(FLAP_WIDTH / 2.0, 0.0, -FLAP_THICKNESS / 2.0)),
    )

    inspection_panel = model.part("inspection_panel")
    inspection_panel.visual(
        Box((INSPECTION_SIZE_X, INSPECTION_SIZE_Y, INSPECTION_THICKNESS)),
        origin=Origin(xyz=(0.0, -INSPECTION_SIZE_Y / 2.0, -INSPECTION_THICKNESS / 2.0)),
        material="panel_white",
        name="door_leaf",
    )
    inspection_panel.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(0.0, -HINGE_BARREL_RADIUS, -HINGE_BARREL_RADIUS),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material="louver_shadow",
        name="hinge_barrel",
    )
    inspection_panel.visual(
        Box((INSPECTION_SIZE_X * 0.35, 0.010, 0.0016)),
        origin=Origin(
            xyz=(
                0.0,
                -INSPECTION_SIZE_Y + 0.008,
                0.0008,
            )
        ),
        material="louver_shadow",
        name="latch_pull",
    )
    model.articulation(
        "panel_to_inspection_panel",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=inspection_panel,
        origin=Origin(
            xyz=(
                INSPECTION_CENTER_X,
                INSPECTION_CENTER_Y + INSPECTION_SIZE_Y / 2.0,
                0.0,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=INSPECTION_OPEN_ANGLE,
        ),
    )

    return model


def _max_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return float(aabb[1][2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    flap_joint_names = (
        "panel_to_north_flap",
        "panel_to_south_flap",
        "panel_to_east_flap",
        "panel_to_west_flap",
    )
    flap_part_names = ("north_flap", "south_flap", "east_flap", "west_flap")

    rest_max_z: dict[str, float | None] = {}
    for part_name in flap_part_names:
        rest_aabb = ctx.part_element_world_aabb(part_name, elem="flap_surface")
        rest_max_z[part_name] = _max_z(rest_aabb)
        ctx.check(
            f"{part_name}_rest_aabb_present",
            rest_max_z[part_name] is not None,
            details=f"aabb={rest_aabb}",
        )

    open_pose = {
        object_model.get_articulation(joint_name): FLAP_OPEN_ANGLE * 0.82
        for joint_name in flap_joint_names
    }
    with ctx.pose(open_pose):
        for part_name in flap_part_names:
            opened_aabb = ctx.part_element_world_aabb(part_name, elem="flap_surface")
            opened_max_z = _max_z(opened_aabb)
            rest_z = rest_max_z[part_name]
            ctx.check(
                f"{part_name}_opens_downward",
                rest_z is not None and opened_max_z is not None and opened_max_z > rest_z + 0.045,
                details=f"rest_max_z={rest_z}, opened_max_z={opened_max_z}",
            )

    inspection_rest = ctx.part_element_world_aabb("inspection_panel", elem="door_leaf")
    inspection_rest_max_z = _max_z(inspection_rest)
    with ctx.pose(panel_to_inspection_panel=INSPECTION_OPEN_ANGLE * 0.82):
        inspection_open = ctx.part_element_world_aabb("inspection_panel", elem="door_leaf")
        inspection_open_max_z = _max_z(inspection_open)
        ctx.check(
            "inspection_panel_opens_downward",
            inspection_rest_max_z is not None
            and inspection_open_max_z is not None
            and inspection_open_max_z > inspection_rest_max_z + 0.035,
            details=f"rest_max_z={inspection_rest_max_z}, open_max_z={inspection_open_max_z}",
        )

    return ctx.report()


object_model = build_object_model()
