from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="car_sun_visor")

    headliner = model.material("headliner", rgba=(0.82, 0.80, 0.75, 1.0))
    bracket_plastic = model.material("bracket_plastic", rgba=(0.18, 0.18, 0.19, 1.0))
    arm_trim = model.material("arm_trim", rgba=(0.22, 0.22, 0.23, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    visor_vinyl = model.material("visor_vinyl", rgba=(0.76, 0.72, 0.65, 1.0))

    panel_width = 0.34
    panel_height = 0.16
    panel_thickness = 0.018

    panel_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(panel_width, panel_height, radius=0.022, corner_segments=10),
            panel_thickness,
            cap=True,
            closed=True,
        ),
        "sun_visor_panel",
    )
    arm_rod_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, -0.035),
                (0.010, 0.0, -0.038),
                (0.046, 0.0, -0.038),
                (0.082, 0.0, -0.031),
            ],
            radius=0.0046,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "sun_visor_pivot_rod",
    )

    roof_bracket = model.part("roof_bracket")
    roof_bracket.visual(
        Box((0.170, 0.090, 0.012)),
        origin=Origin(xyz=(0.055, 0.0, 0.020)),
        material=headliner,
        name="roof_pad",
    )
    roof_bracket.visual(
        Box((0.052, 0.038, 0.020)),
        origin=Origin(xyz=(0.000, 0.0, 0.004)),
        material=bracket_plastic,
        name="bracket_housing",
    )
    roof_bracket.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.000, 0.0, -0.006)),
        material=bracket_plastic,
        name="bearing_disc",
    )
    roof_bracket.visual(
        Box((0.038, 0.012, 0.012)),
        origin=Origin(xyz=(0.030, 0.0, -0.004)),
        material=bracket_plastic,
        name="housing_tail",
    )
    roof_bracket.inertial = Inertial.from_geometry(
        Box((0.170, 0.090, 0.038)),
        mass=0.40,
        origin=Origin(xyz=(0.055, 0.0, 0.007)),
    )

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.000, 0.0, -0.015)),
        material=arm_trim,
        name="swivel_cap",
    )
    pivot_arm.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.000, 0.0, -0.025)),
        material=steel,
        name="vertical_stem",
    )
    pivot_arm.visual(
        arm_rod_mesh,
        material=steel,
        name="pivot_rod",
    )
    pivot_arm.visual(
        Box((0.030, 0.022, 0.016)),
        origin=Origin(xyz=(0.091, 0.0, -0.029)),
        material=arm_trim,
        name="hinge_block",
    )
    pivot_arm.visual(
        Box((0.006, 0.018, 0.022)),
        origin=Origin(xyz=(0.109, 0.0, -0.029)),
        material=arm_trim,
        name="fork_left_cheek",
    )
    pivot_arm.visual(
        Box((0.006, 0.018, 0.022)),
        origin=Origin(xyz=(0.143, 0.0, -0.029)),
        material=arm_trim,
        name="fork_right_cheek",
    )
    pivot_arm.visual(
        Box((0.040, 0.018, 0.004)),
        origin=Origin(xyz=(0.126, 0.0, -0.016)),
        material=arm_trim,
        name="fork_bridge",
    )
    pivot_arm.inertial = Inertial.from_geometry(
        Box((0.150, 0.040, 0.050)),
        mass=0.22,
        origin=Origin(xyz=(0.075, 0.0, -0.025)),
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        panel_mesh,
        origin=Origin(xyz=(0.174, -0.092, 0.0)),
        material=visor_vinyl,
        name="panel_body",
    )
    visor_panel.visual(
        Box((0.024, 0.026, 0.018)),
        origin=Origin(xyz=(0.016, -0.013, 0.0)),
        material=visor_vinyl,
        name="hinge_web",
    )
    visor_panel.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_trim,
        name="hinge_sleeve",
    )
    visor_panel.inertial = Inertial.from_geometry(
        Box((panel_width, panel_height, panel_thickness)),
        mass=0.65,
        origin=Origin(xyz=(0.174, -0.092, 0.0)),
    )

    model.articulation(
        "side_swing",
        ArticulationType.REVOLUTE,
        parent=roof_bracket,
        child=pivot_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.60,
        ),
    )
    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=pivot_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.112, 0.0, -0.029)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    roof_bracket = object_model.get_part("roof_bracket")
    pivot_arm = object_model.get_part("pivot_arm")
    visor_panel = object_model.get_part("visor_panel")
    side_swing = object_model.get_articulation("side_swing")
    flip_hinge = object_model.get_articulation("flip_hinge")

    ctx.expect_contact(
        roof_bracket,
        pivot_arm,
        elem_a="bearing_disc",
        elem_b="swivel_cap",
        contact_tol=0.0005,
        name="swivel cap stays seated on the roof bracket bearing",
    )

    with ctx.pose({side_swing: 0.0, flip_hinge: 0.0}):
        ctx.expect_gap(
            roof_bracket,
            visor_panel,
            axis="z",
            min_gap=0.004,
            max_gap=0.020,
            name="stowed visor sits just below the roof bracket",
        )

        panel_aabb = ctx.part_element_world_aabb(visor_panel, elem="panel_body")
        roof_aabb = ctx.part_element_world_aabb(roof_bracket, elem="roof_pad")
        panel_long_and_thin = False
        details = "missing AABB"
        if panel_aabb is not None and roof_aabb is not None:
            panel_size = tuple(high - low for low, high in zip(panel_aabb[0], panel_aabb[1]))
            roof_size = tuple(high - low for low, high in zip(roof_aabb[0], roof_aabb[1]))
            panel_long_and_thin = (
                panel_size[0] > roof_size[0] * 1.9
                and panel_size[0] > panel_size[1] * 1.8
                and panel_size[2] < panel_size[1] * 0.18
            )
            details = f"panel_size={panel_size}, roof_size={roof_size}"
        ctx.check(
            "visor panel is long and thin relative to the bracket",
            panel_long_and_thin,
            details=details,
        )

        stowed_panel_aabb = ctx.part_element_world_aabb(visor_panel, elem="panel_body")
        stowed_position = (
            tuple((low + high) * 0.5 for low, high in zip(stowed_panel_aabb[0], stowed_panel_aabb[1]))
            if stowed_panel_aabb is not None
            else None
        )

    with ctx.pose({side_swing: 0.0, flip_hinge: 1.20}):
        flipped_panel_aabb = ctx.part_element_world_aabb(visor_panel, elem="panel_body")
        flipped_position = (
            tuple((low + high) * 0.5 for low, high in zip(flipped_panel_aabb[0], flipped_panel_aabb[1]))
            if flipped_panel_aabb is not None
            else None
        )
        ctx.check(
            "visor flips downward from the roof",
            stowed_position is not None
            and flipped_position is not None
            and flipped_position[2] < stowed_position[2] - 0.060,
            details=f"stowed={stowed_position}, flipped={flipped_position}",
        )

    with ctx.pose({side_swing: 1.45, flip_hinge: 0.0}):
        swung_panel_aabb = ctx.part_element_world_aabb(visor_panel, elem="panel_body")
        swung_position = (
            tuple((low + high) * 0.5 for low, high in zip(swung_panel_aabb[0], swung_panel_aabb[1]))
            if swung_panel_aabb is not None
            else None
        )
        ctx.check(
            "visor swings laterally toward the side window position",
            stowed_position is not None
            and swung_position is not None
            and swung_position[1] > stowed_position[1] + 0.12,
            details=f"stowed={stowed_position}, swung={swung_position}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
