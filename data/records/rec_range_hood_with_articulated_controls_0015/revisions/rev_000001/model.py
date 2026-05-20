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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.80, 0.81, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    brushed_black = model.material("brushed_black", rgba=(0.10, 0.11, 0.12, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.50
    canopy_height = 0.28
    shell_t = 0.015

    front_rail_depth = 0.065
    back_rail_depth = 0.045
    side_rail_width = 0.060
    filter_depth = canopy_depth - front_rail_depth - back_rail_depth + 0.010
    filter_width = (canopy_width - 2.0 * side_rail_width - 0.030) / 2.0 + 0.005

    lower_chimney_width = 0.320
    lower_chimney_depth = 0.260
    chimney_t = 0.012
    lower_chimney_height = 0.400
    lower_chimney_bottom = canopy_height - 0.010
    lower_chimney_center_z = lower_chimney_bottom + lower_chimney_height * 0.5

    upper_chimney_width = lower_chimney_width - 2.0 * chimney_t
    upper_chimney_depth = lower_chimney_depth - 2.0 * chimney_t
    upper_chimney_height = 0.560
    upper_chimney_bottom = 0.470
    upper_chimney_center_z = upper_chimney_bottom + upper_chimney_height * 0.5

    hood_body = model.part("hood_body")

    hood_body.visual(
        Box((canopy_width, front_rail_depth, shell_t)),
        origin=Origin(
            xyz=(0.0, canopy_depth * 0.5 - front_rail_depth * 0.5, shell_t * 0.5)
        ),
        material=stainless,
        name="underside_front_rail",
    )
    hood_body.visual(
        Box((canopy_width, back_rail_depth, shell_t)),
        origin=Origin(
            xyz=(0.0, -canopy_depth * 0.5 + back_rail_depth * 0.5, shell_t * 0.5)
        ),
        material=stainless,
        name="underside_back_rail",
    )
    side_rail_center_y = (
        (-canopy_depth * 0.5 + back_rail_depth)
        + (canopy_depth * 0.5 - front_rail_depth)
    ) * 0.5
    for sign, name in ((-1.0, "underside_left_rail"), (1.0, "underside_right_rail")):
        hood_body.visual(
            Box((side_rail_width, canopy_depth - front_rail_depth - back_rail_depth, shell_t)),
            origin=Origin(
                xyz=(
                    sign * (canopy_width * 0.5 - side_rail_width * 0.5),
                    side_rail_center_y,
                    shell_t * 0.5,
                )
            ),
            material=stainless,
            name=name,
        )

    hood_body.visual(
        Box((filter_width, filter_depth, 0.008)),
        origin=Origin(xyz=(-0.2025, -0.010, 0.019)),
        material=dark_steel,
        name="filter_left",
    )
    hood_body.visual(
        Box((filter_width, filter_depth, 0.008)),
        origin=Origin(xyz=(0.2025, -0.010, 0.019)),
        material=dark_steel,
        name="filter_right",
    )
    hood_body.visual(
        Box((0.030, filter_depth, 0.012)),
        origin=Origin(xyz=(0.0, -0.010, 0.016)),
        material=charcoal,
        name="filter_center_strip",
    )

    hood_body.visual(
        Box((canopy_width, 0.030, 0.190)),
        origin=Origin(xyz=(0.0, canopy_depth * 0.5 - 0.015, 0.095)),
        material=stainless,
        name="canopy_front_face",
    )
    hood_body.visual(
        Box((canopy_width, 0.020, canopy_height)),
        origin=Origin(xyz=(0.0, -canopy_depth * 0.5 + 0.010, canopy_height * 0.5)),
        material=stainless,
        name="canopy_back_face",
    )
    for sign, name in ((-1.0, "canopy_left_wall"), (1.0, "canopy_right_wall")):
        hood_body.visual(
            Box((shell_t, canopy_depth, canopy_height)),
            origin=Origin(
                xyz=(sign * (canopy_width * 0.5 - shell_t * 0.5), 0.0, canopy_height * 0.5)
            ),
            material=stainless,
            name=name,
        )

    hood_body.visual(
        Box((0.620, 0.400, shell_t)),
        origin=Origin(xyz=(0.0, -0.050, canopy_height - shell_t * 0.5)),
        material=stainless,
        name="canopy_top_deck",
    )
    hood_body.visual(
        Box((canopy_width, 0.195, shell_t)),
        origin=Origin(
            xyz=(0.0, 0.135, 0.235),
            rpy=(-0.486, 0.0, 0.0),
        ),
        material=stainless,
        name="canopy_crown_panel",
    )

    hood_body.visual(
        Box((lower_chimney_width, chimney_t, lower_chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                lower_chimney_depth * 0.5 - chimney_t * 0.5,
                lower_chimney_center_z,
            )
        ),
        material=stainless,
        name="lower_chimney_front",
    )
    hood_body.visual(
        Box((lower_chimney_width, chimney_t, lower_chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                -lower_chimney_depth * 0.5 + chimney_t * 0.5,
                lower_chimney_center_z,
            )
        ),
        material=stainless,
        name="lower_chimney_back",
    )
    for sign, name in (
        (-1.0, "lower_chimney_left"),
        (1.0, "lower_chimney_right"),
    ):
        hood_body.visual(
            Box((chimney_t, lower_chimney_depth - 2.0 * chimney_t, lower_chimney_height)),
            origin=Origin(
                xyz=(
                    sign * (lower_chimney_width * 0.5 - chimney_t * 0.5),
                    0.0,
                    lower_chimney_center_z,
                )
            ),
            material=stainless,
            name=name,
        )

    hood_body.visual(
        Box((upper_chimney_width, chimney_t, upper_chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                upper_chimney_depth * 0.5 - chimney_t * 0.5,
                upper_chimney_center_z,
            )
        ),
        material=stainless,
        name="upper_chimney_front",
    )
    hood_body.visual(
        Box((upper_chimney_width, chimney_t, upper_chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                -upper_chimney_depth * 0.5 + chimney_t * 0.5,
                upper_chimney_center_z,
            )
        ),
        material=stainless,
        name="upper_chimney_back",
    )
    for sign, name in (
        (-1.0, "upper_chimney_left"),
        (1.0, "upper_chimney_right"),
    ):
        hood_body.visual(
            Box((chimney_t, upper_chimney_depth - 2.0 * chimney_t, upper_chimney_height)),
            origin=Origin(
                xyz=(
                    sign * (upper_chimney_width * 0.5 - chimney_t * 0.5),
                    0.0,
                    upper_chimney_center_z,
                )
            ),
            material=stainless,
            name=name,
        )

    hood_body.inertial = Inertial.from_geometry(
        Box((canopy_width, canopy_depth, upper_chimney_bottom + upper_chimney_height)),
        mass=18.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (upper_chimney_bottom + upper_chimney_height) * 0.5,
            )
        ),
    )

    knob_x_positions = (-0.180, -0.060, 0.060, 0.180)
    knob_y = canopy_depth * 0.5 - front_rail_depth * 0.5
    for index, knob_x in enumerate(knob_x_positions, start=1):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.017, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=charcoal,
            name="collar",
        )
        knob.visual(
            Cylinder(radius=0.020, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.013)),
            material=brushed_black,
            name="body",
        )
        knob.visual(
            Box((0.004, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, 0.015, -0.013)),
            material=stainless,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.040, 0.040, 0.022)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, -0.011)),
        )
        model.articulation(
            f"hood_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(knob_x, knob_y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
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

    hood = object_model.get_part("hood_body")
    canopy_front = hood.get_visual("canopy_front_face")
    lower_front = hood.get_visual("lower_chimney_front")
    upper_front = hood.get_visual("upper_chimney_front")
    left_filter = hood.get_visual("filter_left")
    right_filter = hood.get_visual("filter_right")

    knob_parts = [object_model.get_part(f"knob_{index}") for index in range(1, 5)]
    knob_joints = [
        object_model.get_articulation(f"hood_to_knob_{index}") for index in range(1, 5)
    ]

    def _dims_from_aabb(aabb):
        return (
            aabb[1][0] - aabb[0][0],
            aabb[1][1] - aabb[0][1],
            aabb[1][2] - aabb[0][2],
        )

    def _center_from_aabb(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    hood_aabb = ctx.part_world_aabb(hood)
    ctx.check("hood_body_has_aabb", hood_aabb is not None, "hood body AABB missing")
    if hood_aabb is not None:
        hood_dims = _dims_from_aabb(hood_aabb)
        ctx.check(
            "hood_realistic_width",
            0.88 <= hood_dims[0] <= 0.92,
            f"expected ~0.90 m width, got {hood_dims[0]:.3f} m",
        )
        ctx.check(
            "hood_realistic_depth",
            0.49 <= hood_dims[1] <= 0.51,
            f"expected ~0.50 m depth, got {hood_dims[1]:.3f} m",
        )
        ctx.check(
            "hood_realistic_height",
            1.00 <= hood_dims[2] <= 1.05,
            f"expected ~1.03 m height, got {hood_dims[2]:.3f} m",
        )

    canopy_front_aabb = ctx.part_element_world_aabb(hood, elem=canopy_front)
    lower_front_aabb = ctx.part_element_world_aabb(hood, elem=lower_front)
    upper_front_aabb = ctx.part_element_world_aabb(hood, elem=upper_front)
    filter_left_aabb = ctx.part_element_world_aabb(hood, elem=left_filter)
    filter_right_aabb = ctx.part_element_world_aabb(hood, elem=right_filter)

    ctx.check(
        "hood_visuals_resolve",
        all(aabb is not None for aabb in (canopy_front_aabb, lower_front_aabb, upper_front_aabb)),
        "front canopy or chimney visuals are missing world AABBs",
    )
    if (
        canopy_front_aabb is not None
        and lower_front_aabb is not None
        and upper_front_aabb is not None
    ):
        canopy_width = canopy_front_aabb[1][0] - canopy_front_aabb[0][0]
        lower_width = lower_front_aabb[1][0] - lower_front_aabb[0][0]
        upper_width = upper_front_aabb[1][0] - upper_front_aabb[0][0]
        ctx.check(
            "broad_canopy_face",
            canopy_width > 0.82 and canopy_width > lower_width * 2.5,
            (
                "front canopy face should read much broader than chimney: "
                f"canopy={canopy_width:.3f}, lower_chimney={lower_width:.3f}"
            ),
        )
        lower_top_z = lower_front_aabb[1][2]
        upper_bottom_z = upper_front_aabb[0][2]
        upper_top_z = upper_front_aabb[1][2]
        ctx.check(
            "telescoping_upper_shell_overlap",
            upper_width < lower_width and lower_top_z > upper_bottom_z,
            (
                "upper chimney should telescope within lower shell: "
                f"upper_width={upper_width:.3f}, lower_width={lower_width:.3f}, "
                f"lower_top={lower_top_z:.3f}, upper_bottom={upper_bottom_z:.3f}"
            ),
        )
        ctx.check(
            "upper_chimney_extends_above_lower",
            upper_top_z > lower_top_z + 0.30,
            f"upper shell should continue above lower shell, got {upper_top_z - lower_top_z:.3f} m",
        )

    ctx.check(
        "underside_filters_present",
        filter_left_aabb is not None and filter_right_aabb is not None,
        "underside filter panels should be present inside the open intake cavity",
    )
    if filter_left_aabb is not None and filter_right_aabb is not None:
        left_center = _center_from_aabb(filter_left_aabb)
        right_center = _center_from_aabb(filter_right_aabb)
        ctx.check(
            "filters_split_opening_evenly",
            left_center[0] < -0.10 and right_center[0] > 0.10,
            f"filters should flank the center strip, got x={left_center[0]:.3f}, {right_center[0]:.3f}",
        )

    articulation_count = len(getattr(object_model, "articulations", ()))
    ctx.check(
        "only_four_articulations",
        articulation_count == 4,
        f"expected exactly four articulations for the four knobs, got {articulation_count}",
    )

    knob_positions = []
    for index, (knob, joint) in enumerate(zip(knob_parts, knob_joints, strict=True), start=1):
        ctx.expect_contact(knob, hood, name=f"knob_{index}_contacts_hood")
        ctx.expect_gap(
            hood,
            knob,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            name=f"knob_{index}_seated_under_hood",
        )
        ctx.expect_overlap(
            knob,
            hood,
            axes="xy",
            min_overlap=0.020,
            name=f"knob_{index}_mounted_in_front_rail_footprint",
        )
        knob_position = ctx.part_world_position(knob)
        ctx.check(
            f"knob_{index}_position_available",
            knob_position is not None,
            f"missing world position for knob_{index}",
        )
        if knob_position is not None:
            knob_positions.append(knob_position)

        limits = joint.motion_limits
        axis = joint.axis
        ctx.check(
            f"knob_{index}_continuous_joint",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"knob_{index} should use a continuous rotary joint, got {joint.articulation_type}",
        )
        ctx.check(
            f"knob_{index}_vertical_axis",
            axis == (0.0, 0.0, 1.0),
            f"knob_{index} axis should be vertical +Z, got {axis}",
        )
        ctx.check(
            f"knob_{index}_unbounded_limits",
            limits is not None and limits.lower is None and limits.upper is None,
            f"continuous knob_{index} should not have lower/upper bounds, got {limits}",
        )

    if len(knob_positions) == 4:
        xs = [position[0] for position in knob_positions]
        ys = [position[1] for position in knob_positions]
        zs = [position[2] for position in knob_positions]
        spacings = [xs[i + 1] - xs[i] for i in range(3)]
        ctx.check(
            "knobs_form_straight_row",
            max(abs(y - ys[0]) for y in ys) < 1e-9 and max(abs(z) for z in zs) < 1e-9,
            f"knob origins should share one front-edge row, got ys={ys}, zs={zs}",
        )
        ctx.check(
            "knobs_evenly_spaced",
            max(spacings) - min(spacings) < 1e-9 and 0.11 <= spacings[0] <= 0.13,
            f"expected ~0.12 m even spacing, got {spacings}",
        )
        ctx.check(
            "knobs_near_front_edge",
            ys[0] > 0.18,
            f"knob row should sit under the front edge, got y={ys[0]:.3f}",
        )

    indicator_rest = ctx.part_element_world_aabb(knob_parts[1], elem="indicator")
    ctx.check(
        "knob_indicator_available",
        indicator_rest is not None,
        "indicator visual on knob_2 should resolve for articulation checks",
    )
    if indicator_rest is not None:
        knob_origin = ctx.part_world_position(knob_parts[1])
        indicator_rest_center = _center_from_aabb(indicator_rest)
        with ctx.pose({knob_joints[1]: math.pi / 2.0}):
            indicator_turn = ctx.part_element_world_aabb(knob_parts[1], elem="indicator")
            ctx.expect_contact(knob_parts[1], hood, name="knob_2_contact_at_quarter_turn")
            ctx.fail_if_parts_overlap_in_current_pose(name="knob_2_quarter_turn_no_overlap")
            if knob_origin is not None and indicator_turn is not None:
                indicator_turn_center = _center_from_aabb(indicator_turn)
                ctx.check(
                    "knob_indicator_starts_facing_forward",
                    indicator_rest_center[1] > knob_origin[1] + 0.008,
                    (
                        "knob indicator should begin on the front side of the knob: "
                        f"indicator_y={indicator_rest_center[1]:.3f}, knob_y={knob_origin[1]:.3f}"
                    ),
                )
                ctx.check(
                    "knob_indicator_rotates_about_vertical_axis",
                    indicator_turn_center[0] < knob_origin[0] - 0.008,
                    (
                        "after quarter turn the indicator should swing laterally around the vertical axis: "
                        f"indicator_x={indicator_turn_center[0]:.3f}, knob_x={knob_origin[0]:.3f}"
                    ),
                )

    with ctx.pose(
        {
            knob_joints[0]: math.pi / 2.0,
            knob_joints[1]: math.pi,
            knob_joints[2]: -math.pi / 2.0,
            knob_joints[3]: 1.5 * math.pi,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_knobs_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="all_knobs_rotated_no_floating")
        for index, knob in enumerate(knob_parts, start=1):
            ctx.expect_contact(knob, hood, name=f"knob_{index}_contact_when_rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
