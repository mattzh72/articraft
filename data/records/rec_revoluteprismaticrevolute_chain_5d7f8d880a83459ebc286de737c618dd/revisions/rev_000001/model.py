from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LENGTH = 0.080
BASE_WIDTH = 0.050
BASE_THICKNESS = 0.010

PIVOT_X = -0.022
PIVOT_Z = 0.028
EAR_LENGTH = 0.018
EAR_THICKNESS = 0.010
EAR_HEIGHT = 0.038
CLEVIS_GAP = 0.020
PEDESTAL_LENGTH = 0.028
PEDESTAL_WIDTH = 0.024
PEDESTAL_HEIGHT = 0.013

ROOT_BOSS_RADIUS = 0.0075
CARRIER_LENGTH = 0.110
CARRIER_RAIL_START = 0.020
RAIL_WIDTH = 0.010
RAIL_HEIGHT = 0.010
PIVOT_BLOCK_LENGTH = 0.020
PIVOT_BLOCK_HEIGHT = 0.016

SLIDER_HOME_X = 0.048
SLIDER_LENGTH = 0.036
SLIDER_WIDTH = 0.026
SLIDER_HEIGHT = 0.028
SLIDER_STROKE = 0.040
DISTAL_FORK_LENGTH = 0.012
DISTAL_FORK_THICKNESS = 0.006
DISTAL_FORK_HEIGHT = 0.018
TAB_GAP = 0.010
SLIDER_CHEEK_HEIGHT = 0.018
SLIDER_CHEEK_CENTER_Z = -0.004
SLIDER_BRIDGE_THICKNESS = 0.008
SLIDER_BRIDGE_CENTER_Z = 0.009
DISTAL_FORK_START_X = SLIDER_LENGTH / 2.0
DISTAL_FORK_BASE_Z = -0.003
TAB_JOINT_Z = 0.014

TAB_BOSS_RADIUS = 0.0045
TAB_BLADE_LENGTH = 0.022
TAB_BLADE_THICKNESS = 0.007
TAB_BLADE_HEIGHT = 0.014
TAB_JOINT_X = 0.024
ROOT_BOSS_LENGTH = CLEVIS_GAP
TAB_BOSS_LENGTH = TAB_GAP


def make_base_clevis() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )

    ear_y = CLEVIS_GAP / 2.0 + EAR_THICKNESS / 2.0
    ear_z = BASE_THICKNESS
    left_ear = (
        cq.Workplane("XY")
        .box(
            EAR_LENGTH,
            EAR_THICKNESS,
            EAR_HEIGHT,
            centered=(True, True, False),
        )
        .translate((PIVOT_X, ear_y, ear_z))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(
            EAR_LENGTH,
            EAR_THICKNESS,
            EAR_HEIGHT,
            centered=(True, True, False),
        )
        .translate((PIVOT_X, -ear_y, ear_z))
    )

    left_buttress = (
        cq.Workplane("XY")
        .box(
            0.018,
            EAR_THICKNESS,
            0.014,
            centered=(True, True, False),
        )
        .translate((PIVOT_X - 0.010, ear_y, BASE_THICKNESS))
    )
    right_buttress = (
        cq.Workplane("XY")
        .box(
            0.018,
            EAR_THICKNESS,
            0.014,
            centered=(True, True, False),
        )
        .translate((PIVOT_X - 0.010, -ear_y, BASE_THICKNESS))
    )

    return plate.union(left_ear).union(right_ear).union(left_buttress).union(right_buttress)


def make_carrier() -> cq.Workplane:
    pivot_boss = cq.Workplane("XZ").circle(ROOT_BOSS_RADIUS).extrude(ROOT_BOSS_LENGTH / 2.0, both=True)

    neck = (
        cq.Workplane("XY")
        .box(
            CARRIER_RAIL_START + 0.002,
            0.010,
            0.010,
            centered=(False, True, True),
        )
        .translate((0.0, 0.0, 0.0))
    )

    rail = (
        cq.Workplane("XY")
        .box(
            CARRIER_LENGTH - CARRIER_RAIL_START,
            RAIL_WIDTH,
            RAIL_HEIGHT,
            centered=(False, True, True),
        )
        .translate((CARRIER_RAIL_START, 0.0, 0.0))
    )

    nose = (
        cq.Workplane("XZ")
        .circle(RAIL_HEIGHT / 2.0)
        .extrude(RAIL_WIDTH / 2.0, both=True)
        .translate((CARRIER_LENGTH, 0.0, 0.0))
    )

    return pivot_boss.union(neck).union(rail).union(nose)


def make_slider_body() -> cq.Workplane:
    cheek_thickness = (SLIDER_WIDTH - RAIL_WIDTH) / 2.0
    cheek_y = RAIL_WIDTH / 2.0 + cheek_thickness / 2.0

    left_cheek = (
        cq.Workplane("XY")
        .box(
            SLIDER_LENGTH,
            cheek_thickness,
            SLIDER_CHEEK_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, cheek_y, SLIDER_CHEEK_CENTER_Z))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(
            SLIDER_LENGTH,
            cheek_thickness,
            SLIDER_CHEEK_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, -cheek_y, SLIDER_CHEEK_CENTER_Z))
    )

    top_bridge = (
        cq.Workplane("XY")
        .box(
            SLIDER_LENGTH,
            SLIDER_WIDTH,
            SLIDER_BRIDGE_THICKNESS,
            centered=(True, True, True),
        )
        .translate((0.0, 0.0, SLIDER_BRIDGE_CENTER_Z))
    )

    fork_y = TAB_GAP / 2.0 + DISTAL_FORK_THICKNESS / 2.0
    left_fork = (
        cq.Workplane("XY")
        .box(
            DISTAL_FORK_LENGTH,
            DISTAL_FORK_THICKNESS,
            DISTAL_FORK_HEIGHT,
            centered=(False, True, False),
        )
        .translate((DISTAL_FORK_START_X, fork_y, DISTAL_FORK_BASE_Z))
    )
    right_fork = (
        cq.Workplane("XY")
        .box(
            DISTAL_FORK_LENGTH,
            DISTAL_FORK_THICKNESS,
            DISTAL_FORK_HEIGHT,
            centered=(False, True, False),
        )
        .translate((DISTAL_FORK_START_X, -fork_y, DISTAL_FORK_BASE_Z))
    )

    fork_root = (
        cq.Workplane("XY")
        .box(
            0.010,
            TAB_GAP + 2.0 * DISTAL_FORK_THICKNESS,
            0.006,
            centered=(False, True, False),
        )
        .translate((DISTAL_FORK_START_X - 0.004, 0.0, DISTAL_FORK_BASE_Z))
    )

    return left_cheek.union(right_cheek).union(top_bridge).union(left_fork).union(right_fork).union(fork_root)


def make_output_tab() -> cq.Workplane:
    boss = cq.Workplane("XZ").circle(TAB_BOSS_RADIUS).extrude(TAB_BOSS_LENGTH / 2.0, both=True)

    blade = (
        cq.Workplane("XY")
        .box(
            TAB_BLADE_LENGTH,
            TAB_BLADE_THICKNESS,
            TAB_BLADE_HEIGHT,
            centered=(False, True, True),
        )
        .translate((0.0, 0.0, 0.0))
    )

    nose = (
        cq.Workplane("XZ")
        .circle(TAB_BLADE_HEIGHT / 2.0)
        .extrude(TAB_BLADE_THICKNESS / 2.0, both=True)
        .translate((TAB_BLADE_LENGTH, 0.0, 0.0))
    )

    return boss.union(blade).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_hinge_slide_hinge")

    model.material("base_metal", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("carrier_metal", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("slider_metal", rgba=(0.46, 0.48, 0.52, 1.0))
    model.material("tab_accent", rgba=(0.82, 0.44, 0.16, 1.0))

    base = model.part("base_clevis")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="base_metal",
        name="base_plate",
    )
    ear_y = CLEVIS_GAP / 2.0 + EAR_THICKNESS / 2.0
    base.visual(
        Box((EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT)),
        origin=Origin(xyz=(PIVOT_X, ear_y, BASE_THICKNESS + EAR_HEIGHT / 2.0)),
        material="base_metal",
        name="left_ear",
    )
    base.visual(
        Box((EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT)),
        origin=Origin(xyz=(PIVOT_X, -ear_y, BASE_THICKNESS + EAR_HEIGHT / 2.0)),
        material="base_metal",
        name="right_ear",
    )
    base.visual(
        Box((0.018, EAR_THICKNESS, 0.014)),
        origin=Origin(xyz=(PIVOT_X - 0.010, ear_y, BASE_THICKNESS + 0.007)),
        material="base_metal",
        name="left_buttress",
    )
    base.visual(
        Box((0.018, EAR_THICKNESS, 0.014)),
        origin=Origin(xyz=(PIVOT_X - 0.010, -ear_y, BASE_THICKNESS + 0.007)),
        material="base_metal",
        name="right_buttress",
    )

    carrier = model.part("carrier")
    carrier.visual(
        Cylinder(radius=ROOT_BOSS_RADIUS, length=ROOT_BOSS_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-1.57079632679, 0.0, 0.0)),
        material="carrier_metal",
        name="root_boss",
    )
    carrier.visual(
        Box((0.016, 0.008, 0.008)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material="carrier_metal",
        name="neck",
    )
    carrier.visual(
        Box((0.090, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material="carrier_metal",
        name="rail",
    )

    slider = model.part("slider_body")
    cheek_thickness = (SLIDER_WIDTH - RAIL_WIDTH) / 2.0
    cheek_center_y = RAIL_WIDTH / 2.0 + cheek_thickness / 2.0
    slider.visual(
        Box((SLIDER_LENGTH, SLIDER_WIDTH, SLIDER_BRIDGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, SLIDER_BRIDGE_CENTER_Z)),
        material="slider_metal",
        name="top_bridge",
    )
    slider.visual(
        Box((SLIDER_LENGTH, cheek_thickness, SLIDER_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, cheek_center_y, SLIDER_CHEEK_CENTER_Z)),
        material="slider_metal",
        name="left_cheek",
    )
    slider.visual(
        Box((SLIDER_LENGTH, cheek_thickness, SLIDER_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, -cheek_center_y, SLIDER_CHEEK_CENTER_Z)),
        material="slider_metal",
        name="right_cheek",
    )
    slider.visual(
        Box((DISTAL_FORK_LENGTH, DISTAL_FORK_THICKNESS, DISTAL_FORK_HEIGHT)),
        origin=Origin(
            xyz=(
                TAB_JOINT_X,
                TAB_GAP / 2.0 + DISTAL_FORK_THICKNESS / 2.0,
                TAB_JOINT_Z,
            )
        ),
        material="slider_metal",
        name="left_fork",
    )
    slider.visual(
        Box((DISTAL_FORK_LENGTH, DISTAL_FORK_THICKNESS, DISTAL_FORK_HEIGHT)),
        origin=Origin(
            xyz=(
                TAB_JOINT_X,
                -(TAB_GAP / 2.0 + DISTAL_FORK_THICKNESS / 2.0),
                TAB_JOINT_Z,
            )
        ),
        material="slider_metal",
        name="right_fork",
    )

    output_tab = model.part("output_tab")
    output_tab.visual(
        Cylinder(radius=TAB_BOSS_RADIUS, length=TAB_BOSS_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-1.57079632679, 0.0, 0.0)),
        material="tab_accent",
        name="tab_boss",
    )
    output_tab.visual(
        Box((TAB_BLADE_LENGTH, TAB_BLADE_THICKNESS, TAB_BLADE_HEIGHT)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material="tab_accent",
        name="tab_blade",
    )

    model.articulation(
        "base_to_carrier",
        ArticulationType.REVOLUTE,
        parent=base,
        child=carrier,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    model.articulation(
        "carrier_to_slider",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=slider,
        origin=Origin(xyz=(SLIDER_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.15,
            lower=0.0,
            upper=SLIDER_STROKE,
        ),
    )

    model.articulation(
        "slider_to_output_tab",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=output_tab,
        origin=Origin(xyz=(TAB_JOINT_X, 0.0, TAB_JOINT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=1.00,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_clevis")
    carrier = object_model.get_part("carrier")
    slider = object_model.get_part("slider_body")
    output_tab = object_model.get_part("output_tab")
    root_hinge = object_model.get_articulation("base_to_carrier")
    slider_stage = object_model.get_articulation("carrier_to_slider")
    distal_hinge = object_model.get_articulation("slider_to_output_tab")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.expect_contact(base, carrier, name="base clevis carries the root hinge boss")
    ctx.expect_contact(carrier, slider, name="slider body rides on the carrier rail")
    ctx.expect_contact(slider, output_tab, name="output tab is pinned in the distal fork")

    with ctx.pose({root_hinge: 0.80}):
        ctx.expect_contact(base, carrier, name="root hinge stays supported when raised")

    with ctx.pose({slider_stage: SLIDER_STROKE}):
        ctx.expect_contact(carrier, slider, name="slider stays guided at full extension")

    with ctx.pose({distal_hinge: 0.80}):
        ctx.expect_contact(slider, output_tab, name="distal hinge stays supported when actuated")

    with ctx.pose({root_hinge: 0.0, slider_stage: 0.0, distal_hinge: 0.0}):
        home_slider_position = ctx.part_world_position(slider)
        home_tab_aabb = ctx.part_world_aabb(output_tab)

    with ctx.pose({root_hinge: 0.80, slider_stage: 0.0, distal_hinge: 0.0}):
        raised_slider_position = ctx.part_world_position(slider)

    with ctx.pose({root_hinge: 0.0, slider_stage: SLIDER_STROKE, distal_hinge: 0.0}):
        extended_slider_position = ctx.part_world_position(slider)

    with ctx.pose({root_hinge: 0.0, slider_stage: SLIDER_STROKE, distal_hinge: 0.80}):
        raised_tab_aabb = ctx.part_world_aabb(output_tab)

    ctx.check(
        "root revolute lifts the carried stage",
        home_slider_position is not None
        and raised_slider_position is not None
        and raised_slider_position[2] > home_slider_position[2] + 0.030,
        details=(
            f"home slider position={home_slider_position}, "
            f"raised slider position={raised_slider_position}"
        ),
    )
    ctx.check(
        "prismatic stage extends the slider outward",
        home_slider_position is not None
        and extended_slider_position is not None
        and extended_slider_position[0] > home_slider_position[0] + 0.030,
        details=(
            f"home slider position={home_slider_position}, "
            f"extended slider position={extended_slider_position}"
        ),
    )
    ctx.check(
        "distal revolute lifts the output tab",
        home_tab_aabb is not None
        and raised_tab_aabb is not None
        and raised_tab_aabb[1][2] > home_tab_aabb[1][2] + 0.010,
        details=(
            f"home tab aabb={home_tab_aabb}, "
            f"raised tab aabb={raised_tab_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
