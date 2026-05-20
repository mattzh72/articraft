from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PANEL_LENGTH = 0.445
PANEL_DEPTH = 0.170
PANEL_THICKNESS = 0.026
PANEL_CENTER = (0.235, -0.096, -0.036)
SLEEVE_CENTER = (0.310, -0.096, -0.036)
SLEEVE_LENGTH = 0.214
SLEEVE_DEPTH = 0.108
EXTENDER_LENGTH = 0.190
EXTENDER_DEPTH = 0.094
EXTENDER_THICKNESS = 0.012
EXTENDER_TRAVEL = 0.120


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _visor_panel_mesh():
    outer = rounded_rect_profile(
        PANEL_LENGTH,
        PANEL_DEPTH,
        radius=0.018,
        corner_segments=10,
    )
    sleeve = _shift_profile(
        rounded_rect_profile(
            SLEEVE_LENGTH,
            SLEEVE_DEPTH,
            radius=0.010,
            corner_segments=8,
        ),
        dx=SLEEVE_CENTER[0] - PANEL_CENTER[0],
        dy=SLEEVE_CENTER[1] - PANEL_CENTER[1],
    )
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [sleeve],
            PANEL_THICKNESS,
            center=True,
        ),
        "visor_padded_panel",
    )


def _extender_mesh():
    profile = rounded_rect_profile(
        EXTENDER_LENGTH,
        EXTENDER_DEPTH,
        radius=0.008,
        corner_segments=8,
    )
    return mesh_from_geometry(
        ExtrudeGeometry(profile, EXTENDER_THICKNESS, center=True),
        "visor_sliding_extender",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="driver_side_sun_visor")

    headliner = model.material("warm_grey_headliner", rgba=(0.72, 0.69, 0.63, 1.0))
    padded_fabric = model.material("molded_beige_fabric", rgba=(0.78, 0.72, 0.62, 1.0))
    extender_fabric = model.material("slightly_lighter_extender", rgba=(0.84, 0.79, 0.69, 1.0))
    grey_plastic = model.material("grey_plastic_hardware", rgba=(0.47, 0.47, 0.45, 1.0))
    dark_pivot = model.material("dark_hinge_pin", rgba=(0.08, 0.08, 0.08, 1.0))
    shadow = model.material("recess_shadow", rgba=(0.10, 0.09, 0.08, 1.0))

    roof_hardware = model.part("roof_hardware")
    roof_hardware.visual(
        Box((0.570, 0.070, 0.012)),
        origin=Origin(xyz=(0.005, 0.018, 0.006)),
        material=headliner,
        name="headliner_patch",
    )
    roof_hardware.visual(
        Box((0.070, 0.052, 0.014)),
        origin=Origin(xyz=(-0.225, 0.004, -0.004)),
        material=grey_plastic,
        name="bracket_base",
    )
    roof_hardware.visual(
        Box((0.038, 0.006, 0.038)),
        origin=Origin(xyz=(-0.225, -0.020, -0.025)),
        material=grey_plastic,
        name="bracket_cheek_0",
    )
    roof_hardware.visual(
        Box((0.038, 0.006, 0.038)),
        origin=Origin(xyz=(-0.225, 0.020, -0.025)),
        material=grey_plastic,
        name="bracket_cheek_1",
    )
    roof_hardware.visual(
        Cylinder(radius=0.010, length=0.044),
        origin=Origin(xyz=(-0.225, 0.000, -0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_pivot,
        name="bracket_socket",
    )
    roof_hardware.visual(
        Box((0.046, 0.046, 0.012)),
        origin=Origin(xyz=(0.225, 0.000, -0.004)),
        material=grey_plastic,
        name="clip_base",
    )
    roof_hardware.visual(
        Box((0.024, 0.006, 0.028)),
        origin=Origin(xyz=(0.225, -0.018, -0.022)),
        material=grey_plastic,
        name="clip_jaw_0",
    )
    roof_hardware.visual(
        Box((0.024, 0.006, 0.028)),
        origin=Origin(xyz=(0.225, 0.018, -0.022)),
        material=grey_plastic,
        name="clip_jaw_1",
    )
    roof_hardware.visual(
        Box((0.024, 0.006, 0.010)),
        origin=Origin(xyz=(0.225, -0.012, -0.036)),
        material=grey_plastic,
        name="clip_lip_0",
    )
    roof_hardware.visual(
        Box((0.024, 0.006, 0.010)),
        origin=Origin(xyz=(0.225, 0.012, -0.036)),
        material=grey_plastic,
        name="clip_lip_1",
    )
    roof_hardware.inertial = Inertial.from_geometry(
        Box((0.570, 0.080, 0.060)),
        mass=0.22,
        origin=Origin(xyz=(0.005, 0.010, -0.010)),
    )

    hinge_rod = model.part("hinge_rod")
    hinge_rod.visual(
        Cylinder(radius=0.0052, length=0.462),
        origin=Origin(xyz=(0.231, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_pivot,
        name="steel_rod",
    )
    hinge_rod.visual(
        Cylinder(radius=0.0085, length=0.018),
        origin=Origin(xyz=(0.454, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_plastic,
        name="clip_pin",
    )
    hinge_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.006, length=0.462),
        mass=0.06,
        origin=Origin(xyz=(0.231, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        _visor_panel_mesh(),
        origin=Origin(xyz=PANEL_CENTER),
        material=padded_fabric,
        name="panel_body",
    )
    visor_panel.visual(
        Box((0.226, 0.004, 0.016)),
        origin=Origin(xyz=(SLEEVE_CENTER[0], SLEEVE_CENTER[1] + SLEEVE_DEPTH / 2.0 + 0.002, -0.036)),
        material=shadow,
        name="sleeve_upper_shadow",
    )
    visor_panel.visual(
        Box((0.226, 0.004, 0.016)),
        origin=Origin(xyz=(SLEEVE_CENTER[0], SLEEVE_CENTER[1] - SLEEVE_DEPTH / 2.0 - 0.002, -0.036)),
        material=shadow,
        name="sleeve_lower_shadow",
    )
    visor_panel.visual(
        Box((0.085, 0.014, 0.018)),
        origin=Origin(xyz=(0.058, -0.010, -0.035)),
        material=grey_plastic,
        name="hinge_leaf_0",
    )
    visor_panel.visual(
        Box((0.085, 0.014, 0.018)),
        origin=Origin(xyz=(0.172, -0.010, -0.035)),
        material=grey_plastic,
        name="hinge_leaf_1",
    )
    visor_panel.visual(
        Box((0.020, 0.012, 0.018)),
        origin=Origin(xyz=(0.452, -0.010, -0.035)),
        material=grey_plastic,
        name="free_end_keeper",
    )
    visor_panel.inertial = Inertial.from_geometry(
        Box((PANEL_LENGTH, PANEL_DEPTH, PANEL_THICKNESS)),
        mass=0.34,
        origin=Origin(xyz=PANEL_CENTER),
    )

    extender = model.part("extender")
    extender.visual(
        _extender_mesh(),
        origin=Origin(),
        material=extender_fabric,
        name="extender_panel",
    )
    extender.visual(
        Box((0.184, 0.006, 0.010)),
        origin=Origin(xyz=(0.000, SLEEVE_DEPTH / 2.0 - 0.004, 0.000)),
        material=grey_plastic,
        name="upper_guide_rail",
    )
    extender.visual(
        Box((0.184, 0.006, 0.010)),
        origin=Origin(xyz=(0.000, -SLEEVE_DEPTH / 2.0 + 0.004, 0.000)),
        material=grey_plastic,
        name="lower_guide_rail",
    )
    extender.visual(
        Box((0.006, 0.002, 0.004)),
        origin=Origin(xyz=(0.000, SLEEVE_DEPTH / 2.0, 0.000)),
        material=grey_plastic,
        name="upper_rubbing_pad",
    )
    extender.visual(
        Box((0.006, 0.002, 0.004)),
        origin=Origin(xyz=(0.000, -SLEEVE_DEPTH / 2.0, 0.000)),
        material=grey_plastic,
        name="lower_rubbing_pad",
    )
    extender.visual(
        Box((0.018, 0.060, 0.006)),
        origin=Origin(xyz=(EXTENDER_LENGTH / 2.0 - 0.011, 0.000, -0.009)),
        material=grey_plastic,
        name="pull_grip",
    )
    extender.inertial = Inertial.from_geometry(
        Box((EXTENDER_LENGTH, EXTENDER_DEPTH, EXTENDER_THICKNESS)),
        mass=0.09,
    )

    model.articulation(
        "bracket_pivot",
        ArticulationType.REVOLUTE,
        parent=roof_hardware,
        child=hinge_rod,
        origin=Origin(xyz=(-0.225, 0.000, -0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "main_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_rod,
        child=visor_panel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "extender_slide",
        ArticulationType.PRISMATIC,
        parent=visor_panel,
        child=extender,
        origin=Origin(xyz=SLEEVE_CENTER),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=EXTENDER_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_hardware")
    panel = object_model.get_part("visor_panel")
    extender = object_model.get_part("extender")
    rod = object_model.get_part("hinge_rod")
    hinge = object_model.get_articulation("main_hinge")
    pivot = object_model.get_articulation("bracket_pivot")
    slide = object_model.get_articulation("extender_slide")

    ctx.allow_overlap(
        rod,
        roof,
        elem_a="steel_rod",
        elem_b="bracket_socket",
        reason="The hinge rod is intentionally captured inside the molded bracket socket at the secondary pivot.",
    )
    ctx.expect_within(
        rod,
        roof,
        axes="yz",
        inner_elem="steel_rod",
        outer_elem="bracket_socket",
        margin=0.001,
        name="hinge rod is centered in bracket socket",
    )
    ctx.expect_overlap(
        rod,
        roof,
        axes="x",
        elem_a="steel_rod",
        elem_b="bracket_socket",
        min_overlap=0.009,
        name="bracket socket surrounds hinge rod pivot zone",
    )
    ctx.expect_overlap(
        extender,
        panel,
        axes="x",
        elem_a="upper_guide_rail",
        elem_b="panel_body",
        min_overlap=0.160,
        name="upper guide rail is retained by visor sleeve",
    )
    ctx.expect_overlap(
        extender,
        panel,
        axes="x",
        elem_a="lower_guide_rail",
        elem_b="panel_body",
        min_overlap=0.160,
        name="lower guide rail is retained by visor sleeve",
    )

    ctx.expect_within(
        extender,
        panel,
        axes="yz",
        inner_elem="extender_panel",
        outer_elem="panel_body",
        margin=0.003,
        name="extender nests inside visor sleeve cross-section",
    )
    ctx.expect_overlap(
        extender,
        panel,
        axes="x",
        elem_a="extender_panel",
        elem_b="panel_body",
        min_overlap=0.180,
        name="retracted extender remains mostly inside visor body",
    )
    ctx.expect_overlap(
        roof,
        panel,
        axes="x",
        elem_a="clip_base",
        elem_b="free_end_keeper",
        min_overlap=0.010,
        name="free end keeper aligns with side clip",
    )

    rest_panel_aabb = ctx.part_world_aabb(panel)
    rest_extender_aabb = ctx.part_world_aabb(extender)
    with ctx.pose({slide: EXTENDER_TRAVEL}):
        ctx.expect_within(
            extender,
            panel,
            axes="y",
            inner_elem="extender_panel",
            outer_elem="panel_body",
            margin=0.003,
            name="extended visor extender stays centered in sleeve",
        )
        ctx.expect_overlap(
            extender,
            panel,
            axes="x",
            elem_a="extender_panel",
            elem_b="panel_body",
            min_overlap=0.065,
            name="extended visor extender retains insertion",
        )
        extended_extender_aabb = ctx.part_world_aabb(extender)

    ctx.check(
        "extender slides outboard",
        rest_extender_aabb is not None
        and extended_extender_aabb is not None
        and float(extended_extender_aabb[1][0] - rest_extender_aabb[1][0]) > EXTENDER_TRAVEL - 0.010,
        details=f"rest={rest_extender_aabb}, extended={extended_extender_aabb}",
    )

    with ctx.pose({hinge: 1.15}):
        lowered_panel_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "main hinge lowers visor panel",
        rest_panel_aabb is not None
        and lowered_panel_aabb is not None
        and float(lowered_panel_aabb[0][2]) < float(rest_panel_aabb[0][2]) - 0.080,
        details=f"rest={rest_panel_aabb}, lowered={lowered_panel_aabb}",
    )

    with ctx.pose({pivot: 0.85}):
        swung_panel_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "secondary pivot swings visor sideways",
        rest_panel_aabb is not None
        and swung_panel_aabb is not None
        and abs(
            (float(swung_panel_aabb[0][1]) + float(swung_panel_aabb[1][1])) * 0.5
            - (float(rest_panel_aabb[0][1]) + float(rest_panel_aabb[1][1])) * 0.5
        )
        > 0.090,
        details=f"rest={rest_panel_aabb}, swung={swung_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
