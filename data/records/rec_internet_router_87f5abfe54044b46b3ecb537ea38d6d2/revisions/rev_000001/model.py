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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_travel_router")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    shell_gray = model.material("shell_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.18, 0.24, 0.28, 0.75))

    body_width = 0.120
    body_depth = 0.082
    body_height = 0.028

    def rr_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    body_shell = section_loft(
        [
            rr_section(body_width, body_depth, 0.015, 0.000),
            rr_section(body_width, body_depth, 0.015, 0.006),
            rr_section(0.116, 0.078, 0.014, 0.021),
            rr_section(0.112, 0.074, 0.013, body_height),
        ]
    )

    antenna_paddle_geom = ExtrudeGeometry.centered(
        rounded_rect_profile(0.014, 0.058, 0.0038),
        0.0036,
    )
    antenna_paddle_geom.rotate_x(math.pi / 2.0)
    antenna_paddle_mesh = save_mesh("router_antenna_paddle", antenna_paddle_geom)

    kickstand_frame_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.066, 0.058, 0.007),
        [rounded_rect_profile(0.042, 0.028, 0.005)],
        0.0028,
        center=True,
    )
    kickstand_frame_geom.rotate_x(math.pi / 2.0)
    kickstand_frame_mesh = save_mesh("router_kickstand_frame", kickstand_frame_geom)

    body = model.part("body")
    body.visual(
        save_mesh("router_body_shell", body_shell),
        material=shell_white,
        name="housing_shell",
    )
    body.visual(
        Box((0.108, 0.070, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0272)),
        material=shell_gray,
        name="top_deck",
    )
    body.visual(
        Box((0.112, 0.074, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shell_gray,
        name="bottom_pan",
    )
    body.visual(
        Box((0.052, 0.0022, 0.006)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5) + 0.0006, 0.012)),
        material=glass_dark,
        name="status_window",
    )
    body.visual(
        Box((0.046, 0.0034, 0.010)),
        origin=Origin(xyz=(0.0, (body_depth * 0.5) + 0.0017, 0.010)),
        material=charcoal,
        name="rear_port_strip",
    )
    body.visual(
        Box((0.064, 0.007, 0.004)),
        origin=Origin(xyz=(0.0, (body_depth * 0.5) + 0.0035, 0.004)),
        material=charcoal,
        name="kickstand_hinge_shelf",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    def add_antenna_base(part_name: str):
        base = model.part(part_name)
        base.visual(
            Box((0.016, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=shell_white,
            name="base_pedestal",
        )
        base.visual(
            Box((0.012, 0.0025, 0.006)),
            origin=Origin(xyz=(0.0, -0.0045, 0.007)),
            material=shell_white,
            name="front_cheek",
        )
        base.visual(
            Box((0.012, 0.0025, 0.006)),
            origin=Origin(xyz=(0.0, 0.0045, 0.007)),
            material=shell_white,
            name="rear_cheek",
        )
        base.inertial = Inertial.from_geometry(
            Box((0.016, 0.012, 0.012)),
            mass=0.012,
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
        )
        return base

    def add_antenna(part_name: str):
        antenna = model.part(part_name)
        antenna.visual(
            Cylinder(radius=0.003, length=0.012),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=charcoal,
            name="antenna_barrel",
        )
        antenna.visual(
            Box((0.012, 0.004, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=charcoal,
            name="antenna_root",
        )
        antenna.visual(
            antenna_paddle_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.033)),
            material=charcoal,
            name="antenna_paddle",
        )
        antenna.inertial = Inertial.from_geometry(
            Box((0.016, 0.008, 0.066)),
            mass=0.02,
            origin=Origin(xyz=(0.0, 0.0, 0.033)),
        )
        return antenna

    left_base = add_antenna_base("left_antenna_base")
    right_base = add_antenna_base("right_antenna_base")
    left_antenna = add_antenna("left_antenna")
    right_antenna = add_antenna("right_antenna")

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.0035, length=0.058),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="kickstand_barrel",
    )
    kickstand.visual(
        Box((0.048, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.0068, -0.007)),
        material=charcoal,
        name="kickstand_upper_bridge",
    )
    kickstand.visual(
        Box((0.004, 0.006, 0.042)),
        origin=Origin(xyz=(-0.022, 0.0064, -0.021)),
        material=charcoal,
        name="left_kickstand_strut",
    )
    kickstand.visual(
        Box((0.004, 0.006, 0.042)),
        origin=Origin(xyz=(0.022, 0.0064, -0.021)),
        material=charcoal,
        name="right_kickstand_strut",
    )
    kickstand.visual(
        Box((0.060, 0.0048, 0.005)),
        origin=Origin(xyz=(0.0, 0.0068, -0.0425)),
        material=charcoal,
        name="kickstand_foot",
    )
    kickstand.inertial = Inertial.from_geometry(
        Box((0.070, 0.012, 0.060)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0068, -0.022)),
    )

    model.articulation(
        "body_to_left_base",
        ArticulationType.FIXED,
        parent=body,
        child=left_base,
        origin=Origin(xyz=(-0.045, 0.026, body_height)),
    )
    model.articulation(
        "body_to_right_base",
        ArticulationType.FIXED,
        parent=body,
        child=right_base,
        origin=Origin(xyz=(0.045, 0.026, body_height)),
    )
    model.articulation(
        "left_antenna_hinge",
        ArticulationType.REVOLUTE,
        parent=left_base,
        child=left_antenna,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.5,
            lower=-1.35,
            upper=0.25,
        ),
    )
    model.articulation(
        "right_antenna_hinge",
        ArticulationType.REVOLUTE,
        parent=right_base,
        child=right_antenna,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.5,
            lower=-1.35,
            upper=0.25,
        ),
    )
    model.articulation(
        "rear_kickstand_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, (body_depth * 0.5) + 0.0035, 0.0095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=-0.05,
            upper=0.95,
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

    body = object_model.get_part("body")
    left_base = object_model.get_part("left_antenna_base")
    right_base = object_model.get_part("right_antenna_base")
    left_antenna = object_model.get_part("left_antenna")
    right_antenna = object_model.get_part("right_antenna")
    kickstand = object_model.get_part("kickstand")

    left_antenna_hinge = object_model.get_articulation("left_antenna_hinge")
    right_antenna_hinge = object_model.get_articulation("right_antenna_hinge")
    rear_kickstand_hinge = object_model.get_articulation("rear_kickstand_hinge")

    ctx.expect_contact(
        left_base,
        body,
        elem_a="base_pedestal",
        elem_b="top_deck",
        name="left_base_mounted_to_top_deck",
    )
    ctx.expect_contact(
        right_base,
        body,
        elem_a="base_pedestal",
        elem_b="top_deck",
        name="right_base_mounted_to_top_deck",
    )
    ctx.expect_contact(
        left_antenna,
        left_base,
        elem_a="antenna_barrel",
        elem_b="base_pedestal",
        name="left_antenna_barrel_seated_on_hinge_base",
    )
    ctx.expect_contact(
        right_antenna,
        right_base,
        elem_a="antenna_barrel",
        elem_b="base_pedestal",
        name="right_antenna_barrel_seated_on_hinge_base",
    )
    ctx.expect_contact(
        kickstand,
        body,
        elem_a="kickstand_barrel",
        elem_b="kickstand_hinge_shelf",
        name="kickstand_barrel_clipped_to_rear_hinge_shelf",
    )

    ctx.check(
        "left_antenna_axis_is_lateral_hinge",
        tuple(left_antenna_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={left_antenna_hinge.axis}",
    )
    ctx.check(
        "right_antenna_axis_is_lateral_hinge",
        tuple(right_antenna_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={right_antenna_hinge.axis}",
    )
    ctx.check(
        "kickstand_axis_is_lower_rear_hinge",
        tuple(rear_kickstand_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={rear_kickstand_hinge.axis}",
    )

    left_base_pos = ctx.part_world_position(left_base)
    right_base_pos = ctx.part_world_position(right_base)
    kickstand_rest_aabb = ctx.part_world_aabb(kickstand)
    left_rest_aabb = ctx.part_world_aabb(left_antenna)
    right_rest_aabb = ctx.part_world_aabb(right_antenna)

    if left_base_pos is not None and right_base_pos is not None:
        ctx.check(
            "antenna_bases_span_top_corners",
            left_base_pos[0] < -0.03 and right_base_pos[0] > 0.03 and left_base_pos[2] > 0.025,
            details=f"left={left_base_pos}, right={right_base_pos}",
        )

    if kickstand_rest_aabb is not None:
        ctx.check(
            "kickstand_stows_near_back_face",
            kickstand_rest_aabb[1][1] < 0.055 and kickstand_rest_aabb[0][2] < -0.03,
            details=f"kickstand_rest_aabb={kickstand_rest_aabb}",
        )

    with ctx.pose({left_antenna_hinge: -1.1, right_antenna_hinge: -1.1}):
        left_folded_aabb = ctx.part_world_aabb(left_antenna)
        right_folded_aabb = ctx.part_world_aabb(right_antenna)
        ctx.expect_contact(
            left_antenna,
            left_base,
            elem_a="antenna_barrel",
            elem_b="base_pedestal",
            name="left_antenna_remains_clipped_when_folded",
        )
        ctx.expect_contact(
            right_antenna,
            right_base,
            elem_a="antenna_barrel",
            elem_b="base_pedestal",
            name="right_antenna_remains_clipped_when_folded",
        )
        if left_rest_aabb is not None and left_folded_aabb is not None:
            ctx.check(
                "left_antenna_rotates_down_from_vertical",
                left_folded_aabb[1][2] < left_rest_aabb[1][2] - 0.02,
                details=f"rest={left_rest_aabb}, folded={left_folded_aabb}",
            )
        if right_rest_aabb is not None and right_folded_aabb is not None:
            ctx.check(
                "right_antenna_rotates_down_from_vertical",
                right_folded_aabb[1][2] < right_rest_aabb[1][2] - 0.02,
                details=f"rest={right_rest_aabb}, folded={right_folded_aabb}",
            )

    with ctx.pose({rear_kickstand_hinge: 0.75}):
        kickstand_deployed_aabb = ctx.part_world_aabb(kickstand)
        ctx.expect_contact(
            kickstand,
            body,
            elem_a="kickstand_barrel",
            elem_b="kickstand_hinge_shelf",
            name="kickstand_barrel_stays_attached_when_deployed",
        )
        if kickstand_rest_aabb is not None and kickstand_deployed_aabb is not None:
            ctx.check(
                "kickstand_swings_out_to_prop_router",
                kickstand_deployed_aabb[1][1] > kickstand_rest_aabb[1][1] + 0.015
                and kickstand_deployed_aabb[0][2] > kickstand_rest_aabb[0][2] + 0.008,
                details=f"rest={kickstand_rest_aabb}, deployed={kickstand_deployed_aabb}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
