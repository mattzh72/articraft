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
    section_loft,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _xz_section(width: float, height: float, radius: float, y: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    wood = model.material("wood", rgba=(0.26, 0.17, 0.10, 1.0))
    gloss_wood = model.material("gloss_wood", rgba=(0.35, 0.22, 0.13, 1.0))
    lining = model.material("lining", rgba=(0.70, 0.63, 0.53, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.60, 0.31, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.34, 0.39, 0.43, 0.32))
    cushion = model.material("cushion", rgba=(0.18, 0.16, 0.13, 1.0))

    outer_w = 0.220
    outer_d = 0.180
    body_h = 0.095
    plinth_h = 0.012
    wall_t = 0.012
    bottom_t = 0.010

    lid_w = 0.224
    lid_d = 0.184
    lid_t = 0.018
    hinge_radius = 0.0045
    hinge_axis_y = outer_d * 0.5
    hinge_axis_z = body_h + hinge_radius

    body = model.part("body")
    body.visual(
        Box((outer_w + 0.016, outer_d + 0.012, plinth_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h * 0.5)),
        material=wood,
        name="base_plinth",
    )
    body.visual(
        Box((outer_w, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t * 0.5)),
        material=gloss_wood,
        name="bottom_panel",
    )
    body.visual(
        Box((wall_t, outer_d, body_h)),
        origin=Origin(xyz=((outer_w - wall_t) * 0.5, 0.0, body_h * 0.5)),
        material=gloss_wood,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d, body_h)),
        origin=Origin(xyz=(-(outer_w - wall_t) * 0.5, 0.0, body_h * 0.5)),
        material=gloss_wood,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, body_h - 0.002)),
        origin=Origin(xyz=(0.0, -(outer_d - wall_t) * 0.5, (body_h - 0.002) * 0.5)),
        material=gloss_wood,
        name="front_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, (outer_d - wall_t) * 0.5, body_h * 0.5)),
        material=gloss_wood,
        name="rear_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t - 0.006, outer_d - 2.0 * wall_t - 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t + 0.002)),
        material=lining,
        name="floor_lining",
    )
    body.visual(
        Box((0.112, 0.050, 0.055)),
        origin=Origin(xyz=(0.0, 0.052, 0.040)),
        material=lining,
        name="motor_pod",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(
            xyz=(0.0, 0.024, 0.047),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="bearing_boss",
    )
    body.visual(
        Box((0.156, 0.008, 0.074)),
        origin=Origin(xyz=(0.0, 0.064, 0.050)),
        material=lining,
        name="back_lining_panel",
    )
    for x_center, barrel_name in ((-0.070, "left_body_hinge"), (0.070, "right_body_hinge")):
        body.visual(
            Cylinder(radius=hinge_radius, length=0.040),
            origin=Origin(
                xyz=(x_center, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=brass,
            name=barrel_name,
        )
    body.inertial = Inertial.from_geometry(
        Box((outer_w + 0.016, outer_d + 0.012, body_h + plinth_h)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, (body_h + plinth_h) * 0.5)),
    )

    lid = model.part("lid")
    lid_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(lid_w, lid_d, 0.012, corner_segments=8),
            [rounded_rect_profile(0.156, 0.116, 0.008, corner_segments=8)],
            lid_t,
            center=True,
        ),
        "lid_frame",
    )
    lid.visual(
        lid_frame_mesh,
        origin=Origin(xyz=(0.0, -lid_d * 0.5, lid_t * 0.5 - hinge_radius)),
        material=gloss_wood,
        name="lid_frame",
    )
    lid.visual(
        Box((0.164, 0.124, 0.0035)),
        origin=Origin(xyz=(0.0, -lid_d * 0.5, lid_t - hinge_radius - 0.0030)),
        material=smoked_glass,
        name="window_panel",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brass,
        name="center_lid_hinge",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_t + 0.012)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -lid_d * 0.5, 0.002)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="spindle_shaft",
    )
    cradle.visual(
        Cylinder(radius=0.020, length=0.015),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="hub_collar",
    )
    cradle.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=lining,
        name="back_plate",
    )
    cushion_mesh = mesh_from_geometry(
        section_loft(
            [
                _xz_section(0.058, 0.052, 0.012, -0.028),
                _xz_section(0.076, 0.068, 0.016, -0.048),
                _xz_section(0.084, 0.074, 0.018, -0.070),
                _xz_section(0.072, 0.064, 0.016, -0.094),
            ]
        ),
        "cradle_cushion",
    )
    cradle.visual(
        cushion_mesh,
        material=cushion,
        name="cushion_shell",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.084, 0.102, 0.074)),
        mass=0.28,
        origin=Origin(xyz=(0.0, -0.060, 0.0)),
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.020, 0.047)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="left_wall",
        max_gap=0.006,
        max_penetration=0.0,
        name="lid rests closely on body when closed",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.14,
        name="closed lid covers the body opening",
    )
    ctx.expect_gap(
        lid,
        cradle,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="cushion_shell",
        min_gap=0.010,
        name="closed lid clears the cradle cushion",
    )
    ctx.expect_origin_distance(
        cradle,
        body,
        axes="x",
        max_dist=0.001,
        name="cradle stays centered laterally in the box",
    )

    lid_frame = ctx.part_element_world_aabb(lid, elem="lid_frame")
    closed_panel = ctx.part_element_world_aabb(lid, elem="window_panel")
    if lid_frame is not None and closed_panel is not None:
        left_margin = closed_panel[0][0] - lid_frame[0][0]
        right_margin = lid_frame[1][0] - closed_panel[1][0]
        front_margin = closed_panel[0][1] - lid_frame[0][1]
        rear_margin = lid_frame[1][1] - closed_panel[1][1]
    else:
        left_margin = right_margin = front_margin = rear_margin = None
    ctx.check(
        "window panel stays centered within the lid frame",
        left_margin is not None
        and right_margin is not None
        and front_margin is not None
        and rear_margin is not None
        and abs(left_margin - right_margin) <= 0.002
        and abs(front_margin - rear_margin) <= 0.002
        and min(left_margin, right_margin, front_margin, rear_margin) >= 0.020,
        details=(
            f"left_margin={left_margin}, right_margin={right_margin}, "
            f"front_margin={front_margin}, rear_margin={rear_margin}"
        ),
    )

    cradle_rest = ctx.part_element_world_aabb(cradle, elem="cushion_shell")
    with ctx.pose({lid_hinge: math.radians(70.0)}):
        open_panel = ctx.part_element_world_aabb(lid, elem="window_panel")
    with ctx.pose({cradle_spin: math.pi * 0.5}):
        cradle_quarter_turn = ctx.part_element_world_aabb(cradle, elem="cushion_shell")
    ctx.check(
        "lid opens upward at positive angle",
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][2] > closed_panel[1][2] + 0.05,
        details=f"closed_panel={closed_panel}, open_panel={open_panel}",
    )
    ctx.check(
        "cradle rotation changes the visible cradle orientation",
        cradle_rest is not None
        and cradle_quarter_turn is not None
        and abs(
            (cradle_rest[1][0] - cradle_rest[0][0])
            - (cradle_quarter_turn[1][0] - cradle_quarter_turn[0][0])
        )
        > 0.004,
        details=f"cradle_rest={cradle_rest}, cradle_quarter_turn={cradle_quarter_turn}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
