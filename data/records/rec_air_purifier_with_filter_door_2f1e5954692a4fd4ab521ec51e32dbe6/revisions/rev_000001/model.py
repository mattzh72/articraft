from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_W = 0.230
BODY_D = 0.180
BODY_H = 0.500
WALL_T = 0.008
BASE_T = 0.014

SERVICE_OPENING_W = 0.166
SERVICE_OPENING_H = 0.205
SERVICE_OPENING_BOTTOM = 0.055
SERVICE_JAMB_W = (BODY_W - (2.0 * WALL_T) - SERVICE_OPENING_W) / 2.0
UPPER_FASCIA_H = BODY_H - WALL_T - SERVICE_OPENING_BOTTOM - SERVICE_OPENING_H

FLAP_W = SERVICE_OPENING_W - 0.004
FLAP_H = SERVICE_OPENING_H - 0.004
FLAP_T = 0.007
FLAP_RETURN_D = 0.010
FLAP_OPEN_ANGLE = 1.25

FILTER_W = 0.150
FILTER_H = 0.186
FILTER_D = 0.112
FILTER_FRONT_Y = -0.049
FILTER_TRAVEL = 0.045

GUIDE_W = 0.032
GUIDE_H = 0.008
GUIDE_L = 0.080
GUIDE_Y = -0.015
GUIDE_Z = SERVICE_OPENING_BOTTOM + 0.012
GUIDE_X = (BODY_W / 2.0) - WALL_T - (GUIDE_W / 2.0)

KNOB_SHAFT_L = 0.008
KNOB_SHAFT_R = 0.004
KNOB_Z = 0.350
KNOB_Y = 0.008


def _add_box(part, size, xyz, material, name: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_cylinder(part, radius, length, xyz, rpy, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_floor_purifier")

    body_finish = model.material("body_finish", rgba=(0.92, 0.93, 0.94, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.77, 0.79, 0.82, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.26, 0.29, 0.32, 1.0))
    filter_frame_finish = model.material("filter_frame_finish", rgba=(0.95, 0.96, 0.97, 1.0))
    filter_media_finish = model.material("filter_media_finish", rgba=(0.73, 0.81, 0.86, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    shaft_finish = model.material("shaft_finish", rgba=(0.55, 0.57, 0.60, 1.0))

    housing = model.part("housing")
    _add_box(
        housing,
        (BODY_W, BODY_D, BASE_T),
        (0.0, 0.0, BASE_T / 2.0),
        body_finish,
        "base_plinth",
    )
    _add_box(
        housing,
        (BODY_W, WALL_T, BODY_H),
        (0.0, (BODY_D / 2.0) - (WALL_T / 2.0), BODY_H / 2.0),
        body_finish,
        "back_panel",
    )
    _add_box(
        housing,
        (WALL_T, BODY_D, BODY_H),
        (-(BODY_W / 2.0) + (WALL_T / 2.0), 0.0, BODY_H / 2.0),
        body_finish,
        "side_shell_0",
    )
    _add_box(
        housing,
        (WALL_T, BODY_D, BODY_H),
        ((BODY_W / 2.0) - (WALL_T / 2.0), 0.0, BODY_H / 2.0),
        body_finish,
        "side_shell_1",
    )
    _add_box(
        housing,
        (BODY_W, BODY_D, WALL_T),
        (0.0, 0.0, BODY_H - (WALL_T / 2.0)),
        body_finish,
        "top_cover",
    )
    _add_box(
        housing,
        (BODY_W - (2.0 * WALL_T), WALL_T, UPPER_FASCIA_H),
        (
            0.0,
            -(BODY_D / 2.0) + (WALL_T / 2.0),
            SERVICE_OPENING_BOTTOM + SERVICE_OPENING_H + (UPPER_FASCIA_H / 2.0),
        ),
        body_finish,
        "upper_fascia",
    )
    _add_box(
        housing,
        (0.138, 0.004, 0.116),
        (0.0, -(BODY_D / 2.0) + 0.002, 0.345),
        grille_finish,
        "front_grille",
    )
    _add_box(
        housing,
        (SERVICE_JAMB_W, WALL_T, SERVICE_OPENING_H),
        (
            -((SERVICE_OPENING_W / 2.0) + (SERVICE_JAMB_W / 2.0)),
            -(BODY_D / 2.0) + (WALL_T / 2.0),
            SERVICE_OPENING_BOTTOM + (SERVICE_OPENING_H / 2.0),
        ),
        trim_finish,
        "front_jamb_0",
    )
    _add_box(
        housing,
        (SERVICE_JAMB_W, WALL_T, SERVICE_OPENING_H),
        (
            (SERVICE_OPENING_W / 2.0) + (SERVICE_JAMB_W / 2.0),
            -(BODY_D / 2.0) + (WALL_T / 2.0),
            SERVICE_OPENING_BOTTOM + (SERVICE_OPENING_H / 2.0),
        ),
        trim_finish,
        "front_jamb_1",
    )
    _add_box(
        housing,
        (BODY_W - (2.0 * WALL_T), WALL_T, SERVICE_OPENING_BOTTOM),
        (
            0.0,
            -(BODY_D / 2.0) + (WALL_T / 2.0),
            SERVICE_OPENING_BOTTOM / 2.0,
        ),
        trim_finish,
        "front_sill",
    )
    _add_box(
        housing,
        (GUIDE_W, GUIDE_L, GUIDE_H),
        (-GUIDE_X, GUIDE_Y, GUIDE_Z),
        trim_finish,
        "guide_0",
    )
    _add_box(
        housing,
        (GUIDE_W, GUIDE_L, GUIDE_H),
        (GUIDE_X, GUIDE_Y, GUIDE_Z),
        trim_finish,
        "guide_1",
    )

    flap = model.part("service_flap")
    _add_box(
        flap,
        (FLAP_W, FLAP_T, FLAP_H),
        (0.0, -(FLAP_T / 2.0), FLAP_H / 2.0),
        body_finish,
        "panel",
    )
    _add_box(
        flap,
        (0.070, 0.005, 0.012),
        (0.0, -(FLAP_T + 0.0025), FLAP_H - 0.026),
        trim_finish,
        "pull_ridge",
    )
    _add_box(
        flap,
        (0.008, FLAP_RETURN_D, FLAP_H - 0.020),
        (-(FLAP_W / 2.0) + 0.004, FLAP_RETURN_D / 2.0, (FLAP_H - 0.020) / 2.0),
        trim_finish,
        "side_return_0",
    )
    _add_box(
        flap,
        (0.008, FLAP_RETURN_D, FLAP_H - 0.020),
        ((FLAP_W / 2.0) - 0.004, FLAP_RETURN_D / 2.0, (FLAP_H - 0.020) / 2.0),
        trim_finish,
        "side_return_1",
    )

    cassette = model.part("filter_cassette")
    _add_box(
        cassette,
        (FILTER_W - 0.016, FILTER_D - 0.012, FILTER_H - 0.016),
        (0.0, (FILTER_D / 2.0) + 0.002, 0.0),
        filter_media_finish,
        "filter_media",
    )
    _add_box(
        cassette,
        (0.008, FILTER_D, FILTER_H),
        (-(FILTER_W / 2.0) + 0.004, FILTER_D / 2.0, 0.0),
        filter_frame_finish,
        "frame_0",
    )
    _add_box(
        cassette,
        (0.008, FILTER_D, FILTER_H),
        ((FILTER_W / 2.0) - 0.004, FILTER_D / 2.0, 0.0),
        filter_frame_finish,
        "frame_1",
    )
    _add_box(
        cassette,
        (FILTER_W - 0.016, FILTER_D, 0.008),
        (0.0, FILTER_D / 2.0, (FILTER_H / 2.0) - 0.004),
        filter_frame_finish,
        "frame_2",
    )
    _add_box(
        cassette,
        (FILTER_W - 0.016, FILTER_D, 0.008),
        (0.0, FILTER_D / 2.0, -((FILTER_H / 2.0) - 0.004)),
        filter_frame_finish,
        "frame_3",
    )
    _add_box(
        cassette,
        (0.052, 0.012, 0.020),
        (0.0, -0.006, (FILTER_H / 2.0) - 0.028),
        trim_finish,
        "pull_tab",
    )
    _add_box(
        cassette,
        (0.020, 0.010, 0.050),
        (0.0, 0.005, (FILTER_H / 2.0) - 0.048),
        trim_finish,
        "tab_bridge",
    )

    knob = model.part("control_knob")
    _add_cylinder(
        knob,
        radius=KNOB_SHAFT_R,
        length=KNOB_SHAFT_L,
        xyz=(KNOB_SHAFT_L / 2.0, 0.0, 0.0),
        rpy=(0.0, pi / 2.0, 0.0),
        material=shaft_finish,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.018,
                body_style="skirted",
                top_diameter=0.026,
                skirt=KnobSkirt(0.038, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=14, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
                center=False,
            ),
            "purifier_side_knob",
        ),
        origin=Origin(xyz=(KNOB_SHAFT_L, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=knob_finish,
        name="knob_cap",
    )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_D / 2.0) + FLAP_T,
                SERVICE_OPENING_BOTTOM,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "housing_to_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=cassette,
        origin=Origin(
            xyz=(
                0.0,
                FILTER_FRONT_Y,
                SERVICE_OPENING_BOTTOM + (FILTER_H / 2.0),
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.18,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=((BODY_W / 2.0), KNOB_Y, KNOB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    flap = object_model.get_part("service_flap")
    cassette = object_model.get_part("filter_cassette")
    knob = object_model.get_part("control_knob")

    flap_hinge = object_model.get_articulation("housing_to_flap")
    filter_slide = object_model.get_articulation("housing_to_filter")
    knob_spin = object_model.get_articulation("housing_to_knob")

    ctx.expect_gap(
        cassette,
        flap,
        axis="y",
        min_gap=0.011,
        max_gap=0.020,
        name="filter rests clearly behind the closed flap",
    )
    ctx.expect_within(
        cassette,
        housing,
        axes="xz",
        margin=0.010,
        name="filter cassette stays centered in the housing opening",
    )
    ctx.expect_overlap(
        cassette,
        housing,
        axes="y",
        elem_b="guide_0",
        min_overlap=0.050,
        name="collapsed filter still overlaps the guide rail",
    )
    ctx.expect_gap(
        knob,
        housing,
        axis="x",
        min_gap=0.0,
        max_gap=0.010,
        name="side knob mounts flush against the right wall",
    )
    ctx.expect_overlap(
        knob,
        housing,
        axes="yz",
        min_overlap=0.020,
        name="side knob is mounted on the right sidewall footprint",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(flap, elem="panel")
    closed_filter_pos = ctx.part_world_position(cassette)

    with ctx.pose({flap_hinge: FLAP_OPEN_ANGLE, filter_slide: FILTER_TRAVEL, knob_spin: 1.2}):
        ctx.expect_overlap(
            cassette,
            housing,
            axes="y",
            elem_b="guide_0",
            min_overlap=0.030,
            name="extended filter remains retained by the guide rail",
        )
        open_panel_aabb = ctx.part_element_world_aabb(flap, elem="panel")
        open_filter_pos = ctx.part_world_position(cassette)

    flap_rotates_down = False
    flap_details = f"closed={closed_panel_aabb!r}, open={open_panel_aabb!r}"
    if closed_panel_aabb is not None and open_panel_aabb is not None:
        closed_min, closed_max = closed_panel_aabb
        open_min, open_max = open_panel_aabb
        flap_rotates_down = (
            open_max[2] < closed_max[2] - 0.050
            and open_min[1] < closed_min[1] - 0.055
        )
    ctx.check(
        "service flap rotates downward and outward",
        flap_rotates_down,
        details=flap_details,
    )

    filter_extends_out = False
    filter_details = f"closed={closed_filter_pos!r}, open={open_filter_pos!r}"
    if closed_filter_pos is not None and open_filter_pos is not None:
        filter_extends_out = open_filter_pos[1] < closed_filter_pos[1] - 0.030
    ctx.check(
        "filter cassette slides outward",
        filter_extends_out,
        details=filter_details,
    )

    knob_limits = knob_spin.motion_limits
    ctx.check(
        "side knob uses continuous rotation",
        knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
        details=f"limits={knob_limits!r}",
    )

    return ctx.report()


object_model = build_object_model()
