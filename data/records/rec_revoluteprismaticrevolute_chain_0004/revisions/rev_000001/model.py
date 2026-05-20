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

BASE_L = 0.18
BASE_W = 0.14
BASE_H = 0.028
COLUMN_R = 0.028
COLUMN_H = 0.052
TOP_PLATE_R = 0.042
TOP_PLATE_T = 0.012
ROOT_Z = BASE_H + COLUMN_H + TOP_PLATE_T

CARRIER_PIVOT_R = 0.042
CARRIER_PIVOT_T = 0.012
RAIL_BACK = 0.008
RAIL_L = 0.185
RAIL_OUTER_W = 0.056
RAIL_BOTTOM_T = 0.012
RAIL_GUIDE_T = 0.006
RAIL_GUIDE_H = 0.010

SLIDER_L = 0.13
SLIDER_W = 0.034
SLIDER_H = 0.016
SLIDER_RIB_L = 0.072
SLIDER_RIB_W = 0.020
SLIDER_RIB_H = 0.010
SLIDE_START_X = 0.040
SLIDE_STROKE = 0.10

CLEVIS_LEN = 0.016
CLEVIS_THICK = 0.0045
CLEVIS_GAP = 0.013
CLEVIS_H = 0.040
PIN_Z = 0.024
FORK_PIN_X = SLIDER_L + CLEVIS_LEN / 2.0

FORK_TAB_L = 0.010
FORK_TAB_W = 0.011
FORK_TAB_H = 0.006
FORK_BASE_L = 0.016
FORK_BASE_W = 0.024
FORK_BASE_H = 0.006
FORK_TINE_L = 0.028
FORK_TINE_W = 0.005
FORK_TINE_H = 0.006
FORK_TINE_OFFSET = 0.008
FORK_HUB_R = 0.005


def _carrier_plate_shape() -> cq.Workplane:
    pivot = cq.Workplane("XY").circle(CARRIER_PIVOT_R).extrude(CARRIER_PIVOT_T)
    rail_bottom = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_OUTER_W, RAIL_BOTTOM_T)
        .translate((RAIL_BACK + RAIL_L / 2.0, 0.0, RAIL_BOTTOM_T / 2.0))
    )
    return pivot.union(rail_bottom)


def _carrier_guides_shape() -> cq.Workplane:
    side_z = RAIL_BOTTOM_T + RAIL_GUIDE_H / 2.0
    rail_side_pos = SLIDER_W / 2.0 + 0.004 + RAIL_GUIDE_T / 2.0
    left_wall = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_GUIDE_T, RAIL_GUIDE_H)
        .translate((RAIL_BACK + RAIL_L / 2.0, rail_side_pos, side_z))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_GUIDE_T, RAIL_GUIDE_H)
        .translate((RAIL_BACK + RAIL_L / 2.0, -rail_side_pos, side_z))
    )
    return left_wall.union(right_wall)


def _slider_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(SLIDER_L, SLIDER_W, SLIDER_H)
        .translate((SLIDER_L / 2.0, 0.0, 0.0))
    )
    top_rib = (
        cq.Workplane("XY")
        .box(SLIDER_RIB_L, SLIDER_RIB_W, SLIDER_RIB_H)
        .translate(
            (
                SLIDER_RIB_L / 2.0 + 0.016,
                0.0,
                SLIDER_H / 2.0 + SLIDER_RIB_H / 2.0,
            )
        )
    )
    return body.union(top_rib)


def _slider_clevis_shape() -> cq.Workplane:
    cheek_y = CLEVIS_GAP / 2.0 + CLEVIS_THICK / 2.0
    cheek_z = CLEVIS_H / 2.0 - SLIDER_H / 2.0
    left_cheek = (
        cq.Workplane("XY")
        .box(CLEVIS_LEN, CLEVIS_THICK, CLEVIS_H)
        .translate((SLIDER_L + CLEVIS_LEN / 2.0, cheek_y, cheek_z))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(CLEVIS_LEN, CLEVIS_THICK, CLEVIS_H)
        .translate((SLIDER_L + CLEVIS_LEN / 2.0, -cheek_y, cheek_z))
    )
    return left_cheek.union(right_cheek)


def _fork_hub_shape() -> cq.Workplane:
    return cq.Workplane("XZ").circle(FORK_HUB_R).extrude(CLEVIS_GAP)


def _fork_shape() -> cq.Workplane:
    hub = _fork_hub_shape().translate((0.0, -CLEVIS_GAP / 2.0, 0.0))
    arm = (
        cq.Workplane("XY")
        .box(0.020, 0.008, 0.008)
        .translate((0.010, 0.0, 0.0))
    )
    base = (
        cq.Workplane("XY")
        .box(FORK_BASE_L, FORK_BASE_W, FORK_BASE_H)
        .translate((0.020 + FORK_BASE_L / 2.0, 0.0, 0.0))
    )
    tine_center_x = 0.020 + FORK_BASE_L + FORK_TINE_L / 2.0
    left_tine = (
        cq.Workplane("XY")
        .box(FORK_TINE_L, FORK_TINE_W, FORK_TINE_H)
        .translate((tine_center_x, FORK_TINE_OFFSET, 0.0))
    )
    right_tine = (
        cq.Workplane("XY")
        .box(FORK_TINE_L, FORK_TINE_W, FORK_TINE_H)
        .translate((tine_center_x, -FORK_TINE_OFFSET, 0.0))
    )
    return hub.union(arm).union(base).union(left_tine).union(right_tine)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinge_slide_fork_chain", assets=ASSETS)

    dark_paint = model.material("dark_paint", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))
    orange = model.material("tool_orange", rgba=(0.86, 0.44, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_L, BASE_W, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=dark_paint,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + COLUMN_H / 2.0)),
        material=steel,
        name="column",
    )
    base.visual(
        Cylinder(radius=TOP_PLATE_R, length=TOP_PLATE_T),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + COLUMN_H + TOP_PLATE_T / 2.0)),
        material=steel,
        name="top_plate",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, ROOT_Z)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, ROOT_Z / 2.0)),
    )

    carrier = model.part("carrier")
    carrier.visual(
        mesh_from_cadquery(_carrier_plate_shape(), "carrier_plate.obj", assets=ASSETS),
        material=steel,
        name="carrier_plate",
    )
    carrier.visual(
        mesh_from_cadquery(_carrier_guides_shape(), "carrier_guides.obj", assets=ASSETS),
        material=steel,
        name="carrier_guides",
    )
    carrier.inertial = Inertial.from_geometry(
        Box((RAIL_BACK + RAIL_L, RAIL_OUTER_W, RAIL_BOTTOM_T + RAIL_GUIDE_H)),
        mass=1.3,
        origin=Origin(
            xyz=(
                (RAIL_BACK + RAIL_L) / 2.0,
                0.0,
                (RAIL_BOTTOM_T + RAIL_GUIDE_H) / 2.0,
            )
        ),
    )

    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(_slider_body_shape(), "slider_body.obj", assets=ASSETS),
        material=orange,
        name="slider_body",
    )
    slider.visual(
        mesh_from_cadquery(_slider_clevis_shape(), "slider_clevis.obj", assets=ASSETS),
        material=steel,
        name="slider_clevis",
    )
    slider.inertial = Inertial.from_geometry(
        Box((SLIDER_L + CLEVIS_LEN, SLIDER_W + 0.01, CLEVIS_H)),
        mass=0.7,
        origin=Origin(xyz=((SLIDER_L + CLEVIS_LEN) / 2.0, 0.0, 0.0)),
    )

    fork = model.part("fork")
    fork.visual(
        mesh_from_cadquery(_fork_shape(), "fork_shell.obj", assets=ASSETS),
        material=dark_paint,
        name="fork_shell",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.07, FORK_BASE_W, 0.02)),
        mass=0.22,
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_carrier",
        ArticulationType.REVOLUTE,
        parent=base,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, ROOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-math.radians(120.0),
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "carrier_to_slider",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=slider,
        origin=Origin(xyz=(SLIDE_START_X, 0.0, RAIL_BOTTOM_T + SLIDER_H / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_STROKE,
        ),
    )
    model.articulation(
        "slider_to_fork",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=fork,
        origin=Origin(xyz=(FORK_PIN_X, 0.0, PIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-math.radians(75.0),
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    carrier = object_model.get_part("carrier")
    slider = object_model.get_part("slider")
    fork = object_model.get_part("fork")

    base_to_carrier = object_model.get_articulation("base_to_carrier")
    carrier_to_slider = object_model.get_articulation("carrier_to_slider")
    slider_to_fork = object_model.get_articulation("slider_to_fork")

    top_plate = base.get_visual("top_plate")
    carrier_plate = carrier.get_visual("carrier_plate")
    carrier_guides = carrier.get_visual("carrier_guides")
    slider_body = slider.get_visual("slider_body")
    slider_clevis = slider.get_visual("slider_clevis")
    fork_shell = fork.get_visual("fork_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        fork,
        slider,
        elem_a=fork_shell,
        elem_b=slider_clevis,
        reason="The distal fork uses an enclosed clevis-pin style knuckle seated between the slider cheeks, so the hinge capture is modeled as a tight nested fit.",
    )

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
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_origin_gap(
        carrier,
        base,
        axis="z",
        min_gap=ROOT_Z - 0.001,
        max_gap=ROOT_Z + 0.001,
        name="carrier hinge origin sits atop the base column",
    )
    ctx.expect_contact(
        carrier,
        base,
        elem_a=carrier_plate,
        elem_b=top_plate,
        name="carrier thrust plate contacts the base top plate",
    )
    ctx.expect_overlap(
        carrier,
        base,
        axes="xy",
        min_overlap=0.07,
        elem_a=carrier_plate,
        elem_b=top_plate,
        name="carrier pivot footprint stays seated over the base plate",
    )

    ctx.expect_origin_gap(
        slider,
        carrier,
        axis="x",
        min_gap=SLIDE_START_X - 0.001,
        max_gap=SLIDE_START_X + 0.001,
        name="slider starts near the rear of the carrier rail",
    )
    ctx.expect_overlap(
        slider,
        carrier,
        axes="y",
        elem_a=slider_body,
        elem_b=carrier_guides,
        min_overlap=0.02,
        name="slider remains laterally aligned inside the carrier guide",
    )
    ctx.expect_contact(
        slider,
        carrier,
        elem_a=slider_body,
        elem_b=carrier_plate,
        name="slider body rides directly on the carrier bed",
    )

    ctx.expect_origin_gap(
        fork,
        slider,
        axis="x",
        min_gap=FORK_PIN_X - 0.001,
        max_gap=FORK_PIN_X + 0.001,
        name="fork joint sits at the nose of the slider",
    )
    ctx.expect_contact(
        fork,
        slider,
        elem_a=fork_shell,
        elem_b=slider_clevis,
        name="fork hub is captured between the slider clevis cheeks",
    )

    with ctx.pose({base_to_carrier: math.radians(120.0)}):
        ctx.expect_origin_distance(
            carrier,
            base,
            axes="xy",
            max_dist=0.001,
            name="carrier hinge keeps the rotating stage centered at positive root limit",
        )
        ctx.expect_contact(
            carrier,
            base,
            elem_a=carrier_plate,
            elem_b=top_plate,
            name="carrier remains seated on the base at positive root limit",
        )

    with ctx.pose({base_to_carrier: -math.radians(120.0)}):
        ctx.expect_origin_distance(
            carrier,
            base,
            axes="xy",
            max_dist=0.001,
            name="carrier hinge keeps the rotating stage centered at negative root limit",
        )

    with ctx.pose({carrier_to_slider: SLIDE_STROKE}):
        ctx.expect_origin_gap(
            slider,
            carrier,
            axis="x",
            min_gap=SLIDE_START_X + SLIDE_STROKE - 0.001,
            max_gap=SLIDE_START_X + SLIDE_STROKE + 0.001,
            name="slider reaches the full 100 mm extension",
        )
        ctx.expect_contact(
            slider,
            carrier,
            elem_a=slider_body,
            elem_b=carrier_plate,
            name="slider stays supported on the carrier at full extension",
        )

    with ctx.pose({slider_to_fork: math.radians(75.0)}):
        ctx.expect_contact(
            fork,
            slider,
            elem_a=fork_shell,
            elem_b=slider_clevis,
            name="fork remains pinned in the clevis at positive tip limit",
        )

    with ctx.pose({slider_to_fork: -math.radians(75.0)}):
        ctx.expect_contact(
            fork,
            slider,
            elem_a=fork_shell,
            elem_b=slider_clevis,
            name="fork remains pinned in the clevis at negative tip limit",
        )

    with ctx.pose(
        {
            base_to_carrier: math.radians(120.0),
            carrier_to_slider: SLIDE_STROKE,
            slider_to_fork: -math.radians(75.0),
        }
    ):
        ctx.expect_contact(
            carrier,
            base,
            elem_a=carrier_plate,
            elem_b=top_plate,
            name="root hinge stays seated in a combined extreme pose",
        )
        ctx.expect_contact(
            slider,
            carrier,
            elem_a=slider_body,
            elem_b=carrier_plate,
            name="slider stays guided in a combined extreme pose",
        )
        ctx.expect_contact(
            fork,
            slider,
            elem_a=fork_shell,
            elem_b=slider_clevis,
            name="fork stays mounted in a combined extreme pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
