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
    wire_from_points,
)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def _box_rail_mesh(
    *,
    name: str,
    outer_width: float,
    outer_depth: float,
    inner_width: float,
    inner_depth: float,
    length: float,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(outer_width, outer_depth),
            [_rect_profile(inner_width, inner_depth)],
            height=length,
            center=True,
        ),
        name,
    )


def _guide_ring_mesh(*, name: str, half_x: float, half_y: float, wire_radius: float):
    return mesh_from_geometry(
        wire_from_points(
            [
                (-half_x, -half_y, 0.0),
                (half_x, -half_y, 0.0),
                (half_x, half_y, 0.0),
                (-half_x, half_y, 0.0),
            ],
            radius=wire_radius,
            radial_segments=14,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=min(half_x, half_y) * 0.45,
            corner_segments=8,
        ),
        name,
    )


def _add_rungs(
    part,
    *,
    rung_positions: list[float],
    rung_length: float,
    rung_radius: float,
    y_offset: float,
    material,
    name_prefix: str,
) -> None:
    for index, z in enumerate(rung_positions):
        part.visual(
            Cylinder(radius=rung_radius, length=rung_length),
            origin=Origin(xyz=(0.0, y_offset, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=f"{name_prefix}_rung_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fibreglass_extension_ladder")

    frp_orange = model.material("frp_orange", rgba=(0.93, 0.48, 0.11, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    galvanized = model.material("galvanized", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    base_length = 4.20
    fly_length = 3.95

    base_rail_outer_x = 0.034
    base_rail_outer_y = 0.092
    base_rail_inner_x = 0.020
    base_rail_inner_y = 0.068
    fly_rail_outer_x = 0.030
    fly_rail_outer_y = 0.074
    fly_rail_inner_x = 0.018
    fly_rail_inner_y = 0.052

    base_rail_center_x = 0.185
    fly_rail_center_x = 0.170
    fly_section_y = 0.103

    base_rung_length = (base_rail_center_x * 2.0) - base_rail_outer_x
    fly_rung_length = (fly_rail_center_x * 2.0) - fly_rail_outer_x
    rung_radius = 0.016

    base_rung_positions = [0.34 + 0.305 * index for index in range(13)]
    fly_rung_positions = [0.28 + 0.305 * index for index in range(12)]
    guide_heights = [0.88, 2.10, 3.32]

    base_rail_mesh = _box_rail_mesh(
        name="base_rail_shell",
        outer_width=base_rail_outer_x,
        outer_depth=base_rail_outer_y,
        inner_width=base_rail_inner_x,
        inner_depth=base_rail_inner_y,
        length=base_length,
    )
    fly_rail_mesh = _box_rail_mesh(
        name="fly_rail_shell",
        outer_width=fly_rail_outer_x,
        outer_depth=fly_rail_outer_y,
        inner_width=fly_rail_inner_x,
        inner_depth=fly_rail_inner_y,
        length=fly_length,
    )
    guide_ring_mesh = _guide_ring_mesh(
        name="guide_ring",
        half_x=0.022,
        half_y=0.045,
        wire_radius=0.004,
    )

    base_section = model.part("base_section")
    base_section.visual(
        base_rail_mesh,
        origin=Origin(xyz=(-base_rail_center_x, 0.0, base_length * 0.5)),
        material=frp_orange,
        name="left_base_rail",
    )
    base_section.visual(
        base_rail_mesh,
        origin=Origin(xyz=(base_rail_center_x, 0.0, base_length * 0.5)),
        material=frp_orange,
        name="right_base_rail",
    )
    base_section.visual(
        Box((base_rail_outer_x, base_rail_outer_y, 0.010)),
        origin=Origin(xyz=(-base_rail_center_x, 0.0, base_length - 0.005)),
        material=dark_steel,
        name="left_top_cap",
    )
    base_section.visual(
        Box((base_rail_outer_x, base_rail_outer_y, 0.010)),
        origin=Origin(xyz=(base_rail_center_x, 0.0, base_length - 0.005)),
        material=dark_steel,
        name="right_top_cap",
    )
    _add_rungs(
        base_section,
        rung_positions=base_rung_positions,
        rung_length=base_rung_length,
        rung_radius=rung_radius,
        y_offset=0.0,
        material=aluminum,
        name_prefix="base",
    )
    for index, z in enumerate(guide_heights):
        for side_name, sign in (("left", -1.0), ("right", 1.0)):
            x = sign * fly_rail_center_x
            base_section.visual(
                Box((0.030, 0.022, 0.080)),
                origin=Origin(xyz=(x, 0.055, z)),
                material=galvanized,
                name=f"{side_name}_guide_mount_{index}",
            )
            base_section.visual(
                guide_ring_mesh,
                origin=Origin(xyz=(x, fly_section_y, z)),
                material=galvanized,
                name=f"{side_name}_guide_ring_{index}",
            )
    base_section.inertial = Inertial.from_geometry(
        Box((0.44, 0.16, base_length)),
        mass=21.0,
        origin=Origin(xyz=(0.0, 0.04, base_length * 0.5)),
    )

    fly_section = model.part("fly_section")
    fly_section.visual(
        fly_rail_mesh,
        origin=Origin(xyz=(-fly_rail_center_x, fly_section_y, fly_length * 0.5)),
        material=frp_orange,
        name="left_fly_rail",
    )
    fly_section.visual(
        fly_rail_mesh,
        origin=Origin(xyz=(fly_rail_center_x, fly_section_y, fly_length * 0.5)),
        material=frp_orange,
        name="right_fly_rail",
    )
    fly_section.visual(
        Box((fly_rail_outer_x, fly_rail_outer_y, 0.010)),
        origin=Origin(xyz=(-fly_rail_center_x, fly_section_y, fly_length - 0.005)),
        material=dark_steel,
        name="left_fly_cap",
    )
    fly_section.visual(
        Box((fly_rail_outer_x, fly_rail_outer_y, 0.010)),
        origin=Origin(xyz=(fly_rail_center_x, fly_section_y, fly_length - 0.005)),
        material=dark_steel,
        name="right_fly_cap",
    )
    _add_rungs(
        fly_section,
        rung_positions=fly_rung_positions,
        rung_length=fly_rung_length,
        rung_radius=rung_radius,
        y_offset=fly_section_y,
        material=aluminum,
        name_prefix="fly",
    )
    fly_section.inertial = Inertial.from_geometry(
        Box((0.40, 0.13, fly_length)),
        mass=17.0,
        origin=Origin(xyz=(0.0, fly_section_y, fly_length * 0.5)),
    )

    left_foot = model.part("left_foot")
    left_foot.visual(
        Box((0.030, 0.046, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=dark_steel,
        name="left_hanger",
    )
    left_foot.visual(
        Box((0.072, 0.094, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, -0.040)),
        material=rubber,
        name="left_pad",
    )
    left_foot.inertial = Inertial.from_geometry(
        Box((0.08, 0.10, 0.06)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.01, -0.03)),
    )

    right_foot = model.part("right_foot")
    right_foot.visual(
        Box((0.030, 0.046, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=dark_steel,
        name="right_hanger",
    )
    right_foot.visual(
        Box((0.072, 0.094, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, -0.040)),
        material=rubber,
        name="right_pad",
    )
    right_foot.inertial = Inertial.from_geometry(
        Box((0.08, 0.10, 0.06)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.01, -0.03)),
    )

    model.articulation(
        "base_to_fly_slide",
        ArticulationType.PRISMATIC,
        parent=base_section,
        child=fly_section,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.6, lower=0.0, upper=1.50),
    )
    model.articulation(
        "left_foot_swivel",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child=left_foot,
        origin=Origin(xyz=(-base_rail_center_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.5, lower=-1.10, upper=0.40),
    )
    model.articulation(
        "right_foot_swivel",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child=right_foot,
        origin=Origin(xyz=(base_rail_center_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.5, lower=-1.10, upper=0.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_section = object_model.get_part("base_section")
    fly_section = object_model.get_part("fly_section")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")
    fly_slide = object_model.get_articulation("base_to_fly_slide")
    left_swivel = object_model.get_articulation("left_foot_swivel")
    right_swivel = object_model.get_articulation("right_foot_swivel")

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

    ctx.check(
        "ladder_parts_present",
        all(
            part is not None
            for part in (base_section, fly_section, left_foot, right_foot)
        ),
        "Base section, fly section, and both swivel feet should exist.",
    )
    ctx.check(
        "primary_articulations_match_prompt",
        fly_slide.articulation_type == ArticulationType.PRISMATIC
        and left_swivel.articulation_type == ArticulationType.REVOLUTE
        and right_swivel.articulation_type == ArticulationType.REVOLUTE,
        "Expected one prismatic fly slide and two revolute swivel-foot joints.",
    )
    ctx.check(
        "articulation_axes_are_realistic",
        fly_slide.axis == (0.0, 0.0, 1.0)
        and left_swivel.axis == (1.0, 0.0, 0.0)
        and right_swivel.axis == (1.0, 0.0, 0.0),
        (
            f"Got axes fly={fly_slide.axis}, "
            f"left={left_swivel.axis}, right={right_swivel.axis}."
        ),
    )

    ctx.expect_contact(
        left_foot,
        base_section,
        elem_a="left_hanger",
        name="left_foot_hanger_contacts_base_rail",
    )
    ctx.expect_contact(
        right_foot,
        base_section,
        elem_a="right_hanger",
        name="right_foot_hanger_contacts_base_rail",
    )
    ctx.expect_gap(
        fly_section,
        base_section,
        axis="y",
        positive_elem="left_fly_rail",
        negative_elem="left_base_rail",
        min_gap=0.017,
        max_gap=0.023,
        name="left_fly_rail_runs_forward_of_base_rail",
    )
    ctx.expect_gap(
        fly_section,
        base_section,
        axis="y",
        positive_elem="right_fly_rail",
        negative_elem="right_base_rail",
        min_gap=0.017,
        max_gap=0.023,
        name="right_fly_rail_runs_forward_of_base_rail",
    )
    ctx.expect_within(
        fly_section,
        base_section,
        axes="x",
        margin=0.0,
        name="fly_section_stays_within_base_width",
    )
    ctx.expect_overlap(
        fly_section,
        base_section,
        axes="x",
        min_overlap=0.36,
        name="fly_section_remains_centered_between_stiles",
    )
    ctx.expect_contact(
        base_section,
        fly_section,
        elem_a="left_guide_mount_0",
        elem_b="left_fly_rail",
        name="left_d_ring_guide_bracket_contacts_fly_rail",
    )
    ctx.expect_contact(
        base_section,
        fly_section,
        elem_a="right_guide_mount_0",
        elem_b="right_fly_rail",
        name="right_d_ring_guide_bracket_contacts_fly_rail",
    )

    rest_fly_pos = ctx.part_world_position(fly_section)
    with ctx.pose({fly_slide: 1.50}):
        extended_fly_pos = ctx.part_world_position(fly_section)
        ctx.expect_within(
            fly_section,
            base_section,
            axes="x",
            margin=0.0,
            name="extended_fly_section_stays_within_base_width",
        )
        ctx.check(
            "fly_section_extends_upward",
            rest_fly_pos is not None
            and extended_fly_pos is not None
            and 1.49 <= (extended_fly_pos[2] - rest_fly_pos[2]) <= 1.51,
            (
                f"Expected about 1.50 m upward slide, got "
                f"{None if rest_fly_pos is None or extended_fly_pos is None else extended_fly_pos[2] - rest_fly_pos[2]}."
            ),
        )

    rest_left_pad = ctx.part_element_world_aabb(left_foot, elem="left_pad")
    with ctx.pose({left_swivel: -0.90}):
        swung_left_pad = ctx.part_element_world_aabb(left_foot, elem="left_pad")
        ctx.check(
            "left_swivel_rotates_rubber_pad",
            rest_left_pad is not None
            and swung_left_pad is not None
            and abs(swung_left_pad[0][2] - rest_left_pad[0][2]) > 0.010
            and abs(swung_left_pad[1][1] - rest_left_pad[1][1]) > 0.010,
            "The left rubber foot pad should visibly change pose when the swivel joint turns.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
