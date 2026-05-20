from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CANOPY_HEIGHT = 0.42
BOTTOM_FRONT_Y = -0.34
TOP_FRONT_Y = -0.13
PANEL_ROLL = math.atan2(BOTTOM_FRONT_Y - TOP_FRONT_Y, CANOPY_HEIGHT)
PANEL_UP = (0.0, math.sin(-PANEL_ROLL), math.cos(PANEL_ROLL))
PANEL_IN = (0.0, math.cos(PANEL_ROLL), math.sin(PANEL_ROLL))

PLATE_THICKNESS = 0.010
BUTTON_TRAVEL = 0.008
BUTTON_X = 0.095
BUTTON_CENTER_Z = 0.245
BUTTON_CENTER_Y = BOTTOM_FRONT_Y + (TOP_FRONT_Y - BOTTOM_FRONT_Y) * (
    BUTTON_CENTER_Z / CANOPY_HEIGHT
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _canopy_shell() -> MeshGeometry:
    """Open-bottom four-panel frustum for the faceted hood canopy."""
    geom = MeshGeometry()

    bottom_w = 0.94
    top_w = 0.30
    bottom_rear_y = 0.24
    top_rear_y = 0.11
    bottom_z = 0.035
    top_z = CANOPY_HEIGHT

    points = [
        (-bottom_w / 2.0, BOTTOM_FRONT_Y, bottom_z),
        (bottom_w / 2.0, BOTTOM_FRONT_Y, bottom_z),
        (bottom_w / 2.0, bottom_rear_y, bottom_z),
        (-bottom_w / 2.0, bottom_rear_y, bottom_z),
        (-top_w / 2.0, TOP_FRONT_Y, top_z),
        (top_w / 2.0, TOP_FRONT_Y, top_z),
        (top_w / 2.0, top_rear_y, top_z),
        (-top_w / 2.0, top_rear_y, top_z),
    ]
    ids = [geom.add_vertex(*p) for p in points]
    b0, b1, b2, b3, t0, t1, t2, t3 = ids

    _add_quad(geom, b0, b1, t1, t0)  # front sloped panel
    _add_quad(geom, b1, b2, t2, t1)  # right facet
    _add_quad(geom, b2, b3, t3, t2)  # rear facet
    _add_quad(geom, b3, b0, t0, t3)  # left facet
    _add_quad(geom, t0, t1, t2, t3)  # small top deck under chimney
    return geom


def _button_cap() -> MeshGeometry:
    """A large rounded rectangular push button whose back face lies at local Y=0."""
    thickness = 0.018
    geom = ExtrudeGeometry(
        rounded_rect_profile(0.150, 0.070, 0.010, corner_segments=8),
        thickness,
        cap=True,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(0.0, -thickness / 2.0, 0.0)
    return geom


def _front_panel_point(x: float) -> tuple[float, float, float]:
    return (x, BUTTON_CENTER_Y, BUTTON_CENTER_Z)


def _panel_local_point(local_x: float, local_y: float, local_z: float) -> tuple[float, float, float]:
    return (
        local_x,
        BUTTON_CENTER_Y + local_y * math.cos(PANEL_ROLL) - local_z * math.sin(PANEL_ROLL),
        BUTTON_CENTER_Z + local_y * math.sin(PANEL_ROLL) + local_z * math.cos(PANEL_ROLL),
    )


def _plate_front_point(x: float) -> tuple[float, float, float]:
    # Local -Y is outward from the sloped front panel.
    return _panel_local_point(x, -PLATE_THICKNESS, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.73, 0.70, 1.0))
    shadow = model.material("dark_recess", rgba=(0.035, 0.038, 0.040, 1.0))
    filter_metal = model.material("dark_filter_metal", rgba=(0.22, 0.23, 0.23, 1.0))
    button_mat = model.material("satin_push_button", rgba=(0.86, 0.86, 0.82, 1.0))

    body = model.part("hood_body")
    body.visual(
        mesh_from_geometry(_canopy_shell(), "faceted_canopy"),
        material=stainless,
        name="faceted_canopy",
    )
    body.visual(
        Box((0.25, 0.20, 0.69)),
        origin=Origin(xyz=(0.0, -0.01, 0.735)),
        material=stainless,
        name="plain_chimney",
    )
    body.visual(
        Box((0.98, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, -0.348, 0.0275)),
        material=stainless,
        name="front_lower_lip",
    )
    body.visual(
        Box((0.045, 0.58, 0.055)),
        origin=Origin(xyz=(-0.49, -0.05, 0.0275)),
        material=stainless,
        name="side_lip_0",
    )
    body.visual(
        Box((0.045, 0.58, 0.055)),
        origin=Origin(xyz=(0.49, -0.05, 0.0275)),
        material=stainless,
        name="side_lip_1",
    )
    body.visual(
        Box((0.92, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.235, 0.020)),
        material=stainless,
        name="rear_lower_lip",
    )
    body.visual(
        Box((0.50, 0.30, 0.010)),
        origin=Origin(xyz=(0.0, -0.055, -0.005)),
        material=filter_metal,
        name="underside_filter",
    )
    body.visual(
        Box((0.035, 0.600, 0.012)),
        origin=Origin(xyz=(-0.25, -0.055, -0.004)),
        material=filter_metal,
        name="filter_rail_0",
    )
    body.visual(
        Box((0.035, 0.600, 0.012)),
        origin=Origin(xyz=(0.25, -0.055, -0.004)),
        material=filter_metal,
        name="filter_rail_1",
    )
    for index, x in enumerate((-0.20, -0.12, -0.04, 0.04, 0.12, 0.20)):
        body.visual(
            Box((0.014, 0.280, 0.004)),
            origin=Origin(xyz=(x, -0.055, -0.009)),
            material=shadow,
            name=f"filter_slat_{index}",
        )
    body.visual(
        Box((0.44, PLATE_THICKNESS, 0.024)),
        origin=Origin(
            xyz=_panel_local_point(0.0, -PLATE_THICKNESS / 2.0, 0.058),
            rpy=(PANEL_ROLL, 0.0, 0.0),
        ),
        material=shadow,
        name="button_frame_top",
    )
    body.visual(
        Box((0.44, PLATE_THICKNESS, 0.024)),
        origin=Origin(
            xyz=_panel_local_point(0.0, -PLATE_THICKNESS / 2.0, -0.058),
            rpy=(PANEL_ROLL, 0.0, 0.0),
        ),
        material=shadow,
        name="button_frame_bottom",
    )
    body.visual(
        Box((0.030, PLATE_THICKNESS, 0.140)),
        origin=Origin(
            xyz=_panel_local_point(-0.205, -PLATE_THICKNESS / 2.0, 0.0),
            rpy=(PANEL_ROLL, 0.0, 0.0),
        ),
        material=shadow,
        name="button_frame_side_0",
    )
    body.visual(
        Box((0.030, PLATE_THICKNESS, 0.140)),
        origin=Origin(
            xyz=_panel_local_point(0.205, -PLATE_THICKNESS / 2.0, 0.0),
            rpy=(PANEL_ROLL, 0.0, 0.0),
        ),
        material=shadow,
        name="button_frame_side_1",
    )
    body.visual(
        Box((0.022, PLATE_THICKNESS, 0.140)),
        origin=Origin(
            xyz=_panel_local_point(0.0, -PLATE_THICKNESS / 2.0, 0.0),
            rpy=(PANEL_ROLL, 0.0, 0.0),
        ),
        material=shadow,
        name="button_frame_divider",
    )

    button_mesh = mesh_from_geometry(_button_cap(), "button_cap")
    for index, x in enumerate((-BUTTON_X, BUTTON_X)):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.075, 0.010, 0.028)),
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
            material=shadow,
            name="button_stem",
        )
        model.articulation(
            f"button_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=_plate_front_point(x), rpy=(PANEL_ROLL, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("hood_body")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    slide_0 = object_model.get_articulation("button_0_slide")
    slide_1 = object_model.get_articulation("button_1_slide")

    ctx.check(
        "only the two buttons articulate",
        len(object_model.articulations) == 2
        and {str(a.child) for a in object_model.articulations} == {"button_0", "button_1"}
        and all(str(a.articulation_type).endswith("PRISMATIC") for a in object_model.articulations),
        details=f"articulations={[a.name for a in object_model.articulations]}",
    )
    for button in (button_0, button_1):
        ctx.allow_overlap(
            button,
            body,
            elem_a="button_stem",
            elem_b="faceted_canopy",
            reason="The hidden push-button stem is intentionally captured through the simplified front panel skin.",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="xz",
            min_overlap=0.025,
            elem_a="button_stem",
            elem_b="faceted_canopy",
            name=f"{button.name} stem remains captured through the front panel",
        )
    ctx.expect_origin_distance(
        button_0,
        button_1,
        axes="x",
        min_dist=0.18,
        max_dist=0.20,
        name="buttons are side by side across the front",
    )
    ctx.expect_overlap(
        button_0,
        button_1,
        axes="z",
        min_overlap=0.060,
        elem_a="button_cap",
        elem_b="button_cap",
        name="buttons share the same front-panel height",
    )
    ctx.expect_gap(
        button_0,
        body,
        axis="x",
        min_gap=0.008,
        positive_elem="button_cap",
        negative_elem="button_frame_side_0",
        name="button 0 clears the outer frame edge",
    )
    ctx.expect_gap(
        body,
        button_0,
        axis="x",
        min_gap=0.006,
        positive_elem="button_frame_divider",
        negative_elem="button_cap",
        name="button 0 clears the center divider",
    )
    ctx.expect_gap(
        button_1,
        body,
        axis="x",
        min_gap=0.006,
        positive_elem="button_cap",
        negative_elem="button_frame_divider",
        name="button 1 clears the center divider",
    )
    ctx.expect_gap(
        body,
        button_1,
        axis="x",
        min_gap=0.008,
        positive_elem="button_frame_side_1",
        negative_elem="button_cap",
        name="button 1 clears the outer frame edge",
    )
    ctx.expect_overlap(
        button_0,
        body,
        axes="z",
        min_overlap=0.060,
        elem_a="button_cap",
        elem_b="button_frame_divider",
        name="button caps align vertically with the center divider",
    )

    rest_pos = ctx.part_world_position(button_0)
    with ctx.pose({slide_0: BUTTON_TRAVEL, slide_1: BUTTON_TRAVEL}):
        pressed_pos = ctx.part_world_position(button_0)
    if rest_pos is not None and pressed_pos is not None:
        delta = tuple(pressed_pos[i] - rest_pos[i] for i in range(3))
        inward_travel = sum(delta[i] * PANEL_IN[i] for i in range(3))
        lateral_drift = abs(delta[0])
    else:
        inward_travel = 0.0
        lateral_drift = 1.0
    ctx.check(
        "button travel is inward normal to the sloped panel",
        inward_travel > BUTTON_TRAVEL * 0.95 and lateral_drift < 0.001,
        details=f"rest={rest_pos}, pressed={pressed_pos}, inward={inward_travel}",
    )

    return ctx.report()


object_model = build_object_model()
