from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.760
BODY_DEPTH = 0.500
BODY_HEIGHT = 0.085
SHELL_THICKNESS = 0.012
CONTROL_STRIP_THICKNESS = 0.004
CONTROL_STRIP_HEIGHT = 0.032
CONTROL_STRIP_CENTER_Z = 0.040
CONTROL_STRIP_FACE_Y = (BODY_DEPTH * 0.5) + CONTROL_STRIP_THICKNESS
FRONT_RETURN_DEPTH = 0.030
FRONT_RETURN_HEIGHT = 0.006

SMALL_KNOB_RADIUS = 0.014
MEDIUM_KNOB_RADIUS = 0.017
LARGE_KNOB_RADIUS = 0.020
KNOB_DEPTH = 0.024
KNOB_CENTER_XS = (0.258, 0.301, 0.344)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_width = width * 0.5
    half_depth = depth * 0.5
    return [
        (-half_width, -half_depth),
        (half_width, -half_depth),
        (half_width, half_depth),
        (-half_width, half_depth),
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_intake_panel_mesh():
    panel_width = BODY_WIDTH - (2.0 * SHELL_THICKNESS) - 0.044
    panel_depth = BODY_DEPTH - SHELL_THICKNESS - FRONT_RETURN_DEPTH - 0.048
    outer_profile = _rect_profile(panel_width, panel_depth)

    slot_width = 0.055
    slot_depth = 0.330
    slot_radius = 0.004
    slot_shape = rounded_rect_profile(slot_width, slot_depth, slot_radius, corner_segments=5)
    hole_profiles: list[list[tuple[float, float]]] = []
    for center_x in (-0.302, -0.236, -0.170, -0.104, -0.038, 0.038, 0.104, 0.170, 0.236, 0.302):
        hole_profiles.append(_translate_profile(slot_shape, center_x, 0.0))

    return ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=0.004,
        center=True,
    )


def _add_knob_part(
    model: ArticulatedObject,
    *,
    part_name: str,
    articulation_name: str,
    knob_radius: float,
    center_x: float,
    knob_material,
    indicator_material,
):
    knob = model.part(part_name)
    knob.visual(
        Cylinder(radius=knob_radius, length=KNOB_DEPTH),
        origin=Origin(
            xyz=(0.0, KNOB_DEPTH * 0.5, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=knob_material,
        name="knob_body",
    )
    knob.visual(
        Box((knob_radius * 0.95, 0.0022, max(0.0030, knob_radius * 0.16))),
        origin=Origin(
            xyz=(
                0.0,
                KNOB_DEPTH - 0.0011,
                knob_radius * 0.55,
            )
        ),
        material=indicator_material,
        name="indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=knob_radius, length=KNOB_DEPTH),
        mass=0.065,
        origin=Origin(
            xyz=(0.0, KNOB_DEPTH * 0.5, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
    )

    model.articulation(
        articulation_name,
        ArticulationType.CONTINUOUS,
        parent="hood_body",
        child=knob,
        origin=Origin(xyz=(center_x, CONTROL_STRIP_FACE_Y, CONTROL_STRIP_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=10.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.20, 0.21, 0.23, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.10, 0.11, 1.0))
    filter_aluminum = model.material("filter_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    lamp_lens = model.material("lamp_lens", rgba=(0.94, 0.96, 0.88, 0.55))
    marker = model.material("marker", rgba=(0.82, 0.84, 0.86, 1.0))

    hood = model.part("hood_body")
    hood.visual(
        Box((BODY_WIDTH, BODY_DEPTH, SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - (SHELL_THICKNESS * 0.5))),
        material=stainless,
        name="top_shell",
    )
    hood.visual(
        Box((SHELL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH * 0.5 + SHELL_THICKNESS * 0.5,
                0.0,
                BODY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="left_side",
    )
    hood.visual(
        Box((SHELL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                BODY_WIDTH * 0.5 - SHELL_THICKNESS * 0.5,
                0.0,
                BODY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="right_side",
    )
    hood.visual(
        Box((BODY_WIDTH - (2.0 * SHELL_THICKNESS), SHELL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_DEPTH * 0.5 + SHELL_THICKNESS * 0.5,
                BODY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="back_panel",
    )
    hood.visual(
        Box((BODY_WIDTH, SHELL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 - SHELL_THICKNESS * 0.5,
                BODY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="front_fascia",
    )
    hood.visual(
        Box((BODY_WIDTH, FRONT_RETURN_DEPTH, FRONT_RETURN_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 - FRONT_RETURN_DEPTH * 0.5,
                FRONT_RETURN_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="front_return",
    )
    hood.visual(
        Box((BODY_WIDTH - 0.024, CONTROL_STRIP_THICKNESS, CONTROL_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 + CONTROL_STRIP_THICKNESS * 0.5,
                CONTROL_STRIP_CENTER_Z,
            )
        ),
        material=dark_trim,
        name="control_strip",
    )
    hood.visual(
        _save_mesh(_build_intake_panel_mesh(), "range_hood_intake_panel.obj"),
        origin=Origin(xyz=(0.0, -0.018, 0.014)),
        material=filter_aluminum,
        name="intake_panel",
    )
    hood.visual(
        Box((0.022, 0.405, 0.006)),
        origin=Origin(xyz=(-0.357, -0.018, 0.019)),
        material=dark_trim,
        name="left_filter_rail",
    )
    hood.visual(
        Box((0.022, 0.405, 0.006)),
        origin=Origin(xyz=(0.357, -0.018, 0.019)),
        material=dark_trim,
        name="right_filter_rail",
    )
    hood.visual(
        Box((0.692, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, -0.231, 0.019)),
        material=dark_trim,
        name="rear_filter_rail",
    )
    hood.visual(
        Box((0.692, 0.034, 0.006)),
        origin=Origin(xyz=(0.0, 0.203, 0.019)),
        material=dark_trim,
        name="front_filter_rail",
    )
    hood.visual(
        Box((0.124, 0.040, 0.004)),
        origin=Origin(xyz=(-0.255, 0.196, 0.014)),
        material=dark_trim,
        name="lamp_bezel",
    )
    hood.visual(
        Box((0.110, 0.034, 0.004)),
        origin=Origin(xyz=(-0.255, 0.196, 0.010)),
        material=lamp_lens,
        name="lamp_lens",
    )
    hood.visual(
        Box((0.180, 0.170, 0.055)),
        origin=Origin(xyz=(0.0, 0.010, 0.0515)),
        material=charcoal,
        name="blower_housing",
    )
    hood.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    _add_knob_part(
        model,
        part_name="small_knob",
        articulation_name="small_knob_spin",
        knob_radius=SMALL_KNOB_RADIUS,
        center_x=KNOB_CENTER_XS[0],
        knob_material=charcoal,
        indicator_material=marker,
    )
    _add_knob_part(
        model,
        part_name="medium_knob",
        articulation_name="medium_knob_spin",
        knob_radius=MEDIUM_KNOB_RADIUS,
        center_x=KNOB_CENTER_XS[1],
        knob_material=charcoal,
        indicator_material=marker,
    )
    _add_knob_part(
        model,
        part_name="large_knob",
        articulation_name="large_knob_spin",
        knob_radius=LARGE_KNOB_RADIUS,
        center_x=KNOB_CENTER_XS[2],
        knob_material=charcoal,
        indicator_material=marker,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood = object_model.get_part("hood_body")
    control_strip = hood.get_visual("control_strip")
    intake_panel = hood.get_visual("intake_panel")

    small_knob = object_model.get_part("small_knob")
    medium_knob = object_model.get_part("medium_knob")
    large_knob = object_model.get_part("large_knob")

    small_joint = object_model.get_articulation("small_knob_spin")
    medium_joint = object_model.get_articulation("medium_knob_spin")
    large_joint = object_model.get_articulation("large_knob_spin")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    hood_aabb = ctx.part_world_aabb(hood)
    ctx.check(
        "hood_body_dimensions",
        hood_aabb is not None
        and 0.72 <= hood_aabb[1][0] - hood_aabb[0][0] <= 0.79
        and 0.47 <= hood_aabb[1][1] - hood_aabb[0][1] <= 0.52
        and 0.07 <= hood_aabb[1][2] - hood_aabb[0][2] <= 0.10,
        details=f"hood aabb={hood_aabb}",
    )
    ctx.check(
        "key_visuals_present",
        control_strip is not None and intake_panel is not None,
        details="Expected control strip and intake panel visuals on hood body.",
    )

    knob_positions = [
        ctx.part_world_position(small_knob),
        ctx.part_world_position(medium_knob),
        ctx.part_world_position(large_knob),
    ]
    ctx.check(
        "knob_cluster_far_right",
        hood_aabb is not None
        and all(position is not None for position in knob_positions)
        and knob_positions[0][0] > (hood_aabb[1][0] - 0.17)
        and knob_positions[2][0] > (hood_aabb[1][0] - 0.05),
        details=f"hood aabb={hood_aabb}, knob_positions={knob_positions}",
    )
    ctx.check(
        "knob_cluster_spacing",
        all(position is not None for position in knob_positions)
        and 0.032 <= knob_positions[1][0] - knob_positions[0][0] <= 0.050
        and 0.032 <= knob_positions[2][0] - knob_positions[1][0] <= 0.050
        and abs(knob_positions[0][2] - knob_positions[1][2]) <= 0.002
        and abs(knob_positions[1][2] - knob_positions[2][2]) <= 0.002,
        details=f"knob_positions={knob_positions}",
    )

    small_radius = small_knob.get_visual("knob_body").geometry.radius
    medium_radius = medium_knob.get_visual("knob_body").geometry.radius
    large_radius = large_knob.get_visual("knob_body").geometry.radius
    ctx.check(
        "knob_diameters_increase_left_to_right",
        small_radius < medium_radius < large_radius,
        details=f"radii={(small_radius, medium_radius, large_radius)}",
    )

    for joint_name, joint_obj in (
        ("small_knob_spin", small_joint),
        ("medium_knob_spin", medium_joint),
        ("large_knob_spin", large_joint),
    ):
        limits = joint_obj.motion_limits
        ctx.check(
            f"{joint_name}_continuous_axis",
            joint_obj.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint_obj.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and limits.velocity >= 5.0,
            details=(
                f"type={joint_obj.articulation_type}, axis={joint_obj.axis}, "
                f"limits={limits}"
            ),
        )

    for knob_name, knob_part in (
        ("small_knob", small_knob),
        ("medium_knob", medium_knob),
        ("large_knob", large_knob),
    ):
        ctx.expect_contact(
            knob_part,
            hood,
            elem_b="control_strip",
            contact_tol=0.0015,
            name=f"{knob_name}_touches_control_strip",
        )
        ctx.expect_gap(
            knob_part,
            hood,
            axis="y",
            max_gap=0.0015,
            max_penetration=0.0,
            name=f"{knob_name}_seats_on_front_face",
        )
        ctx.expect_overlap(
            knob_part,
            hood,
            axes="xz",
            min_overlap=0.020,
            name=f"{knob_name}_overlaps_hood_face_projection",
        )

    with ctx.pose(
        {
            small_joint: math.pi * 0.5,
            medium_joint: math.pi,
            large_joint: math.pi * 1.5,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="rotated_knobs_no_floating")
        ctx.expect_contact(
            small_knob,
            hood,
            elem_b="control_strip",
            contact_tol=0.0015,
            name="small_knob_rotated_contact",
        )
        ctx.expect_contact(
            medium_knob,
            hood,
            elem_b="control_strip",
            contact_tol=0.0015,
            name="medium_knob_rotated_contact",
        )
        ctx.expect_contact(
            large_knob,
            hood,
            elem_b="control_strip",
            contact_tol=0.0015,
            name="large_knob_rotated_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
