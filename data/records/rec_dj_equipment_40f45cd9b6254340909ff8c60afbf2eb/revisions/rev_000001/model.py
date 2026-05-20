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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_headphone_monitor_mixer")

    shell_charcoal = model.material("shell_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    panel_black = model.material("panel_black", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_rubber = model.material("knob_rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    marker_light = model.material("marker_light", rgba=(0.84, 0.86, 0.88, 1.0))
    fader_cap = model.material("fader_cap", rgba=(0.90, 0.91, 0.92, 1.0))
    socket_metal = model.material("socket_metal", rgba=(0.40, 0.42, 0.45, 1.0))

    housing_width = 0.224
    housing_depth = 0.130
    housing_height = 0.045
    wall_thickness = 0.004
    bottom_thickness = 0.003
    top_thickness = 0.003
    wall_height = housing_height - bottom_thickness - top_thickness

    housing = model.part("housing")
    housing.visual(
        Box((housing_width, housing_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness * 0.5)),
        material=shell_charcoal,
        name="bottom_plate",
    )
    housing.visual(
        Box((wall_thickness, housing_depth, wall_height)),
        origin=Origin(
            xyz=(
                -housing_width * 0.5 + wall_thickness * 0.5,
                0.0,
                bottom_thickness + wall_height * 0.5,
            )
        ),
        material=shell_charcoal,
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, housing_depth, wall_height)),
        origin=Origin(
            xyz=(
                housing_width * 0.5 - wall_thickness * 0.5,
                0.0,
                bottom_thickness + wall_height * 0.5,
            )
        ),
        material=shell_charcoal,
        name="right_wall",
    )
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                -housing_depth * 0.5 + wall_thickness * 0.5,
                bottom_thickness + wall_height * 0.5,
            )
        ),
        material=panel_black,
        name="front_panel",
    )
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - wall_thickness * 0.5,
                bottom_thickness + wall_height * 0.5,
            )
        ),
        material=shell_charcoal,
        name="rear_panel",
    )

    top_panel_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(
            housing_width - 2.0 * wall_thickness,
            housing_depth - 2.0 * wall_thickness,
            radius=0.010,
            corner_segments=8,
        ),
        [
            rounded_rect_profile(
                0.070,
                0.006,
                radius=0.003,
                corner_segments=6,
            )
        ],
        top_thickness,
        center=True,
        cap=True,
        closed=True,
    )
    housing.visual(
        mesh_from_geometry(top_panel_geom, "monitor_mixer_top_panel"),
        origin=Origin(xyz=(0.0, 0.0, housing_height - top_thickness * 0.5)),
        material=panel_black,
        name="top_panel",
    )
    housing.visual(
        Box((0.092, 0.016, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, housing_height - top_thickness - 0.001)),
        material=shell_charcoal,
        name="fader_track_floor",
    )
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, 0.006, 0.006)),
        origin=Origin(
            xyz=(
                0.0,
                -housing_depth * 0.5 + wall_thickness + 0.003,
                0.018,
            )
        ),
        material=shell_charcoal,
        name="front_trim",
    )

    knob_x_positions = (-0.078, -0.026, 0.026, 0.078)
    knob_z = 0.027
    jack_z = 0.012
    for index, knob_x in enumerate(knob_x_positions, start=1):
        housing.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(
                xyz=(knob_x, -housing_depth * 0.5 - 0.002, jack_z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=socket_metal,
            name=f"jack_bezel_{index}",
        )
        housing.visual(
            Cylinder(radius=0.0046, length=0.005),
            origin=Origin(
                xyz=(knob_x, -housing_depth * 0.5 - 0.0025, jack_z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=panel_black,
            name=f"jack_socket_{index}",
        )
        housing.visual(
            Box((0.010, 0.0014, 0.004)),
            origin=Origin(
                xyz=(knob_x, -housing_depth * 0.5 + 0.0033, 0.036),
            ),
            material=marker_light,
            name=f"knob_label_{index}",
        )

    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, housing_height * 0.5)),
    )

    for index, knob_x in enumerate(knob_x_positions, start=1):
        knob = model.part(f"headphone_knob_{index}")
        knob.visual(
            Cylinder(radius=0.0035, length=0.010),
            origin=Origin(
                xyz=(0.0, -0.005, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=socket_metal,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.016, length=0.017),
            origin=Origin(
                xyz=(0.0, -0.0185, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=knob_rubber,
            name="body",
        )
        knob.visual(
            Cylinder(radius=0.0135, length=0.006),
            origin=Origin(
                xyz=(0.0, -0.030, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=knob_rubber,
            name="face",
        )
        knob.visual(
            Box((0.003, 0.002, 0.010)),
            origin=Origin(xyz=(0.0, -0.034, 0.008)),
            material=marker_light,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.032, 0.036, 0.032)),
            mass=0.045,
            origin=Origin(xyz=(0.0, -0.019, 0.0)),
        )
        model.articulation(
            f"housing_to_headphone_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(knob_x, -housing_depth * 0.5, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.18,
                velocity=6.0,
                lower=-2.5,
                upper=2.5,
            ),
        )

    mix_fader = model.part("mix_fader")
    mix_fader.visual(
        Box((0.014, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=socket_metal,
        name="carriage",
    )
    mix_fader.visual(
        Box((0.004, 0.0034, 0.013)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=socket_metal,
        name="stem",
    )
    mix_fader.visual(
        Box((0.018, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=fader_cap,
        name="cap",
    )
    mix_fader.inertial = Inertial.from_geometry(
        Box((0.018, 0.012, 0.024)),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    model.articulation(
        "housing_to_mix_fader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=mix_fader,
        origin=Origin(xyz=(0.0, 0.0, housing_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.10,
            lower=-0.026,
            upper=0.026,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    housing = object_model.get_part("housing")
    mix_fader = object_model.get_part("mix_fader")
    mix_fader_joint = object_model.get_articulation("housing_to_mix_fader")

    knob_parts = [
        object_model.get_part(f"headphone_knob_{index}")
        for index in range(1, 5)
    ]
    knob_joints = [
        object_model.get_articulation(f"housing_to_headphone_knob_{index}")
        for index in range(1, 5)
    ]

    for index, (knob_part, knob_joint) in enumerate(zip(knob_parts, knob_joints), start=1):
        ctx.expect_contact(knob_part, housing, name=f"knob_{index}_mounted")
        ctx.expect_origin_gap(
            housing,
            knob_part,
            axis="y",
            min_gap=0.062,
            max_gap=0.066,
            name=f"knob_{index}_on_front_face",
        )
        ctx.check(
            f"knob_{index}_axis",
            tuple(round(value, 3) for value in knob_joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={knob_joint.axis}",
        )
        knob_limits = knob_joint.motion_limits
        ctx.check(
            f"knob_{index}_limits",
            knob_limits is not None
            and knob_limits.lower is not None
            and knob_limits.upper is not None
            and knob_limits.lower < -2.0
            and knob_limits.upper > 2.0,
            details=f"limits={knob_limits}",
        )

    ctx.expect_contact(mix_fader, housing, name="mix_fader_mounted")
    ctx.expect_origin_gap(
        mix_fader,
        housing,
        axis="z",
        min_gap=0.044,
        max_gap=0.046,
        name="mix_fader_on_top_centerline",
    )
    ctx.check(
        "mix_fader_axis",
        tuple(round(value, 3) for value in mix_fader_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={mix_fader_joint.axis}",
    )
    fader_limits = mix_fader_joint.motion_limits
    ctx.check(
        "mix_fader_limits",
        fader_limits is not None
        and fader_limits.lower is not None
        and fader_limits.upper is not None
        and fader_limits.lower < 0.0
        and fader_limits.upper > 0.0,
        details=f"limits={fader_limits}",
    )

    knob_indicator_rest = ctx.part_element_world_aabb(knob_parts[0], elem="indicator")
    with ctx.pose({knob_joints[0]: 1.4}):
        ctx.expect_contact(knob_parts[0], housing, name="knob_1_contact_when_turned")
        knob_indicator_turned = ctx.part_element_world_aabb(knob_parts[0], elem="indicator")

    indicator_shift = None
    if knob_indicator_rest is not None and knob_indicator_turned is not None:
        indicator_shift = abs(knob_indicator_turned[1][0] - knob_indicator_rest[1][0])
    ctx.check(
        "knob_1_indicator_moves_with_rotation",
        indicator_shift is not None and indicator_shift > 0.004,
        details=f"indicator_shift={indicator_shift}",
    )

    fader_rest = ctx.part_world_position(mix_fader)
    with ctx.pose({mix_fader_joint: 0.026}):
        ctx.expect_contact(mix_fader, housing, name="mix_fader_contact_high")
        fader_high = ctx.part_world_position(mix_fader)
    with ctx.pose({mix_fader_joint: -0.026}):
        ctx.expect_contact(mix_fader, housing, name="mix_fader_contact_low")
        fader_low = ctx.part_world_position(mix_fader)

    fader_travel = None
    fader_y_stable = None
    fader_z_stable = None
    if fader_high is not None and fader_low is not None:
        fader_travel = fader_high[0] - fader_low[0]
    if fader_rest is not None and fader_high is not None and fader_low is not None:
        fader_y_stable = max(
            abs(fader_high[1] - fader_rest[1]),
            abs(fader_low[1] - fader_rest[1]),
        )
        fader_z_stable = max(
            abs(fader_high[2] - fader_rest[2]),
            abs(fader_low[2] - fader_rest[2]),
        )
    ctx.check(
        "mix_fader_travel_is_central_and_linear",
        fader_travel is not None
        and fader_travel > 0.050
        and fader_y_stable is not None
        and fader_y_stable < 1e-6
        and fader_z_stable is not None
        and fader_z_stable < 1e-6,
        details=(
            f"travel={fader_travel}, "
            f"y_stable={fader_y_stable}, "
            f"z_stable={fader_z_stable}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
