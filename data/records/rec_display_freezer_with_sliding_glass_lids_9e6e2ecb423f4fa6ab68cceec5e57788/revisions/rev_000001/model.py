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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _curved_lid_mesh(
    *,
    mesh_name: str,
    length: float,
    span: float,
    rise: float,
    thickness: float,
    edge_z: float,
    samples: int = 20,
):
    half_span = span * 0.5
    radius = (rise * rise + half_span * half_span) / (2.0 * rise)
    center_z = edge_z - (radius - rise)
    theta_edge = math.asin(half_span / radius)
    inner_radius = radius - thickness

    outer_arc: list[tuple[float, float]] = []
    inner_arc: list[tuple[float, float]] = []
    for index in range(samples + 1):
        theta = -theta_edge + (2.0 * theta_edge * index / samples)
        outer_arc.append(
            (
                radius * math.sin(theta),
                center_z + radius * math.cos(theta),
            )
        )
    for index in range(samples, -1, -1):
        theta = -theta_edge + (2.0 * theta_edge * index / samples)
        inner_arc.append(
            (
                inner_radius * math.sin(theta),
                center_z + inner_radius * math.cos(theta),
            )
        )

    profile = outer_arc + inner_arc
    half_length = length * 0.5
    section_a = [(-half_length, y, z) for y, z in profile]
    section_b = [(half_length, y, z) for y, z in profile]
    return mesh_from_geometry(section_loft([section_a, section_b]), mesh_name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[i] + maximum[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    liner_white = model.material("liner_white", rgba=(0.97, 0.97, 0.96, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.24, 0.27, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    glass = model.material("curved_glass", rgba=(0.67, 0.82, 0.89, 0.32))
    panel_grey = model.material("panel_grey", rgba=(0.82, 0.84, 0.86, 1.0))
    control_black = model.material("control_black", rgba=(0.12, 0.13, 0.15, 1.0))

    body_length = 1.40
    body_depth = 0.82
    base_height = 0.10
    wall_height = 0.70
    wall_thickness = 0.05
    tub_top = base_height + wall_height + 0.04
    opening_length = 1.18
    opening_depth = 0.64

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((1.36, 0.78, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=charcoal,
        name="base_skirt",
    )
    for sx in (-0.56, 0.56):
        for sy in (-0.29, 0.29):
            cabinet.visual(
                Box((0.10, 0.10, 0.02)),
                origin=Origin(xyz=(sx, sy, 0.01)),
                material=rubber,
                name=f"foot_{'l' if sx < 0 else 'r'}_{'f' if sy < 0 else 'b'}",
            )

    cabinet.visual(
        Box((1.28, 0.70, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=cabinet_white,
        name="base_insulation",
    )
    cabinet.visual(
        Box((wall_thickness, body_depth, wall_height)),
        origin=Origin(xyz=(-(body_length - wall_thickness) * 0.5, 0.0, 0.45)),
        material=cabinet_white,
        name="left_outer_wall",
    )
    cabinet.visual(
        Box((wall_thickness, body_depth, wall_height)),
        origin=Origin(xyz=((body_length - wall_thickness) * 0.5, 0.0, 0.45)),
        material=cabinet_white,
        name="right_outer_wall",
    )
    cabinet.visual(
        Box((1.30, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, (body_depth - wall_thickness) * 0.5, 0.45)),
        material=cabinet_white,
        name="rear_outer_wall",
    )
    cabinet.visual(
        Box((0.18, wall_thickness, 0.38)),
        origin=Origin(xyz=(-0.56, -(body_depth - wall_thickness) * 0.5, 0.29)),
        material=cabinet_white,
        name="front_left_stile",
    )
    cabinet.visual(
        Box((0.18, wall_thickness, 0.38)),
        origin=Origin(xyz=(0.56, -(body_depth - wall_thickness) * 0.5, 0.29)),
        material=cabinet_white,
        name="front_right_stile",
    )
    cabinet.visual(
        Box((1.14, wall_thickness, 0.18)),
        origin=Origin(xyz=(0.0, -(body_depth - wall_thickness) * 0.5, 0.09)),
        material=cabinet_white,
        name="front_lower_sill",
    )
    cabinet.visual(
        Box((0.98, 0.03, 0.34)),
        origin=Origin(xyz=(0.0, -0.365, 0.35)),
        material=panel_grey,
        name="trim_backer",
    )
    cabinet.visual(
        Box((1.06, wall_thickness, 0.06)),
        origin=Origin(xyz=(0.0, -(body_depth - wall_thickness) * 0.5, 0.55)),
        material=cabinet_white,
        name="trim_header",
    )
    cabinet.visual(
        Box((1.24, 0.09, 0.28)),
        origin=Origin(xyz=(0.0, -0.365, 0.69)),
        material=cabinet_white,
        name="control_housing",
    )
    cabinet.visual(
        Box((0.40, 0.01, 0.08)),
        origin=Origin(xyz=(0.0, -0.411, 0.70)),
        material=control_black,
        name="control_face",
    )

    cabinet.visual(
        Box((1.10, 0.56, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=liner_white,
        name="tub_floor",
    )
    cabinet.visual(
        Box((0.025, 0.60, 0.60)),
        origin=Origin(xyz=(-0.5375, 0.0, 0.42)),
        material=liner_white,
        name="left_liner_wall",
    )
    cabinet.visual(
        Box((0.025, 0.60, 0.60)),
        origin=Origin(xyz=(0.5375, 0.0, 0.42)),
        material=liner_white,
        name="right_liner_wall",
    )
    cabinet.visual(
        Box((1.05, 0.025, 0.60)),
        origin=Origin(xyz=(0.0, 0.2875, 0.42)),
        material=liner_white,
        name="rear_liner_wall",
    )
    cabinet.visual(
        Box((1.05, 0.025, 0.60)),
        origin=Origin(xyz=(0.0, -0.2875, 0.42)),
        material=liner_white,
        name="front_liner_wall",
    )

    cabinet.visual(
        Box((0.09, 0.74, 0.12)),
        origin=Origin(xyz=(-0.605, 0.0, 0.78)),
        material=cabinet_white,
        name="left_top_bridge",
    )
    cabinet.visual(
        Box((0.09, 0.74, 0.12)),
        origin=Origin(xyz=(0.605, 0.0, 0.78)),
        material=cabinet_white,
        name="right_top_bridge",
    )
    cabinet.visual(
        Box((1.30, 0.11, 0.12)),
        origin=Origin(xyz=(0.0, 0.355, 0.78)),
        material=cabinet_white,
        name="rear_top_bridge",
    )
    cabinet.visual(
        Box((1.30, 0.15, 0.12)),
        origin=Origin(xyz=(0.0, -0.335, 0.78)),
        material=cabinet_white,
        name="front_top_bridge",
    )

    track_length = 1.12
    lower_track_y = 0.307
    upper_track_top = tub_top + 0.032
    lower_track_top = tub_top + 0.008

    cabinet.visual(
        Box((track_length, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, -lower_track_y, lower_track_top - 0.003)),
        material=aluminum,
        name="lower_front_track",
    )
    cabinet.visual(
        Box((track_length, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, lower_track_y, lower_track_top - 0.003)),
        material=aluminum,
        name="lower_rear_track",
    )
    cabinet.visual(
        Box((track_length, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, -lower_track_y, upper_track_top - 0.003)),
        material=aluminum,
        name="upper_front_track",
    )
    cabinet.visual(
        Box((track_length, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, lower_track_y, upper_track_top - 0.003)),
        material=aluminum,
        name="upper_rear_track",
    )
    cabinet.visual(
        Box((track_length, 0.012, upper_track_top - (lower_track_top - 0.016))),
        origin=Origin(xyz=(0.0, -0.325, (upper_track_top + lower_track_top - 0.016) * 0.5)),
        material=aluminum,
        name="front_track_outer_web",
    )
    cabinet.visual(
        Box((track_length, 0.012, upper_track_top - (lower_track_top - 0.016))),
        origin=Origin(xyz=(0.0, 0.325, (upper_track_top + lower_track_top - 0.016) * 0.5)),
        material=aluminum,
        name="rear_track_outer_web",
    )
    cabinet.visual(
        Box((0.03, 0.60, 0.12)),
        origin=Origin(xyz=(-0.615, 0.0, 0.84)),
        material=aluminum,
        name="left_lid_side_guide",
    )
    cabinet.visual(
        Box((0.03, 0.60, 0.12)),
        origin=Origin(xyz=(0.615, 0.0, 0.84)),
        material=aluminum,
        name="right_lid_side_guide",
    )
    cabinet.visual(
        Box((0.96, 0.018, 0.02)),
        origin=Origin(xyz=(0.0, -0.382, 0.210)),
        material=charcoal,
        name="trim_hinge_mount",
    )

    cabinet.inertial = Inertial.from_geometry(
        Box((1.40, 0.82, 0.92)),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    lid_length = 0.68
    lid_glass = _curved_lid_mesh(
        mesh_name="display_freezer_curved_lid",
        length=lid_length,
        span=0.58,
        rise=0.125,
        thickness=0.007,
        edge_z=0.018,
    )

    left_lid = model.part("left_lid")
    left_lid.visual(lid_glass, material=glass, name="left_lid_glass")
    left_lid.visual(
        Box((lid_length, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, -0.300, 0.005)),
        material=aluminum,
        name="left_front_rail",
    )
    left_lid.visual(
        Box((lid_length, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.300, 0.005)),
        material=aluminum,
        name="left_rear_rail",
    )
    left_lid.visual(
        Box((0.03, 0.60, 0.05)),
        origin=Origin(xyz=(-0.325, 0.0, 0.025)),
        material=aluminum,
        name="left_outer_end_cap",
    )
    left_lid.inertial = Inertial.from_geometry(
        Box((0.68, 0.64, 0.16)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    right_lid = model.part("right_lid")
    right_lid.visual(lid_glass, material=glass, name="right_lid_glass")
    right_lid.visual(
        Box((lid_length, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, -0.300, 0.005)),
        material=aluminum,
        name="right_front_rail",
    )
    right_lid.visual(
        Box((lid_length, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.300, 0.005)),
        material=aluminum,
        name="right_rear_rail",
    )
    right_lid.visual(
        Box((0.03, 0.60, 0.05)),
        origin=Origin(xyz=(0.325, 0.0, 0.025)),
        material=aluminum,
        name="right_outer_end_cap",
    )
    right_lid.inertial = Inertial.from_geometry(
        Box((0.68, 0.64, 0.16)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    trim_panel = model.part("trim_panel")
    trim_panel.visual(
        Cylinder(radius=0.009, length=0.90),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=charcoal,
        name="trim_hinge_barrel",
    )
    trim_panel.visual(
        Box((0.92, 0.018, 0.22)),
        origin=Origin(xyz=(0.0, -0.009, 0.11)),
        material=panel_grey,
        name="trim_panel_face",
    )
    trim_panel.visual(
        Box((0.22, 0.028, 0.03)),
        origin=Origin(xyz=(0.0, -0.015, 0.19)),
        material=aluminum,
        name="trim_pull",
    )
    trim_panel.inertial = Inertial.from_geometry(
        Box((0.92, 0.03, 0.23)),
        mass=2.4,
        origin=Origin(xyz=(0.0, -0.010, 0.11)),
    )

    model.articulation(
        "cabinet_to_left_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=left_lid,
        origin=Origin(xyz=(-0.26, 0.0, 0.850)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.22,
            lower=0.0,
            upper=0.42,
        ),
    )
    model.articulation(
        "cabinet_to_right_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=right_lid,
        origin=Origin(xyz=(0.26, 0.0, 0.875)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.22,
            lower=0.0,
            upper=0.42,
        ),
    )
    model.articulation(
        "cabinet_to_trim_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=trim_panel,
        origin=Origin(xyz=(0.0, -0.400, 0.21)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    trim_panel = object_model.get_part("trim_panel")

    left_slide = object_model.get_articulation("cabinet_to_left_lid")
    right_slide = object_model.get_articulation("cabinet_to_right_lid")
    trim_hinge = object_model.get_articulation("cabinet_to_trim_panel")

    with ctx.pose({left_slide: 0.0, right_slide: 0.0, trim_hinge: 0.0}):
        ctx.expect_contact(
            left_lid,
            cabinet,
            elem_a="left_outer_end_cap",
            elem_b="left_lid_side_guide",
            name="left lid outer end cap seats against side guide at rest",
        )
        ctx.expect_contact(
            right_lid,
            cabinet,
            elem_a="right_outer_end_cap",
            elem_b="right_lid_side_guide",
            name="right lid outer end cap seats against side guide at rest",
        )
        ctx.expect_contact(
            trim_panel,
            cabinet,
            elem_a="trim_hinge_barrel",
            elem_b="trim_hinge_mount",
            name="trim access panel is hung from the cabinet hinge mount",
        )
        ctx.expect_gap(
            left_lid,
            cabinet,
            axis="z",
            positive_elem="left_front_rail",
            negative_elem="lower_front_track",
            min_gap=0.001,
            max_gap=0.004,
            name="left lid front rail rides just above lower front track",
        )
        ctx.expect_gap(
            right_lid,
            cabinet,
            axis="z",
            positive_elem="right_front_rail",
            negative_elem="upper_front_track",
            min_gap=0.001,
            max_gap=0.004,
            name="right lid front rail rides just above upper front track",
        )
        ctx.expect_overlap(
            left_lid,
            right_lid,
            axes="x",
            elem_a="left_lid_glass",
            elem_b="right_lid_glass",
            min_overlap=0.12,
            name="curved lids overlap along their travel direction",
        )
        ctx.expect_gap(
            right_lid,
            left_lid,
            axis="z",
            positive_elem="right_front_rail",
            negative_elem="left_front_rail",
            min_gap=0.014,
            max_gap=0.040,
            name="upper lid stays above lower lid for stacked sliding",
        )

    left_rest = ctx.part_world_position(left_lid)
    left_upper = left_slide.motion_limits.upper if left_slide.motion_limits is not None else None
    with ctx.pose({left_slide: left_upper if left_upper is not None else 0.0}):
        left_open = ctx.part_world_position(left_lid)
        ctx.expect_within(
            left_lid,
            cabinet,
            axes="x",
            inner_elem="left_front_rail",
            outer_elem="lower_front_track",
            margin=0.004,
            name="left lid remains captured by lower front track at max travel",
        )

    right_rest = ctx.part_world_position(right_lid)
    right_upper = right_slide.motion_limits.upper if right_slide.motion_limits is not None else None
    with ctx.pose({right_slide: right_upper if right_upper is not None else 0.0}):
        right_open = ctx.part_world_position(right_lid)
        ctx.expect_within(
            right_lid,
            cabinet,
            axes="x",
            inner_elem="right_front_rail",
            outer_elem="upper_front_track",
            margin=0.004,
            name="right lid remains captured by upper front track at max travel",
        )

    ctx.check(
        "left lid slides right to uncover the tub",
        left_rest is not None
        and left_open is not None
        and left_upper is not None
        and left_open[0] > left_rest[0] + 0.30,
        details=f"rest={left_rest}, open={left_open}, upper={left_upper}",
    )
    ctx.check(
        "right lid slides left to uncover the tub",
        right_rest is not None
        and right_open is not None
        and right_upper is not None
        and right_open[0] < right_rest[0] - 0.30,
        details=f"rest={right_rest}, open={right_open}, upper={right_upper}",
    )

    rest_pull_aabb = ctx.part_element_world_aabb(trim_panel, elem="trim_pull")
    trim_upper = trim_hinge.motion_limits.upper if trim_hinge.motion_limits is not None else None
    with ctx.pose({trim_hinge: trim_upper if trim_upper is not None else 0.0}):
        open_pull_aabb = ctx.part_element_world_aabb(trim_panel, elem="trim_pull")

    rest_pull_center = _aabb_center(rest_pull_aabb)
    open_pull_center = _aabb_center(open_pull_aabb)
    ctx.check(
        "trim access panel folds down and outward",
        rest_pull_center is not None
        and open_pull_center is not None
        and open_pull_center[1] < rest_pull_center[1] - 0.08
        and open_pull_center[2] < rest_pull_center[2] - 0.06,
        details=f"rest_pull={rest_pull_center}, open_pull={open_pull_center}, upper={trim_upper}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
