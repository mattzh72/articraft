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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)


BODY_SIZE = 0.038
BODY_HALF = BODY_SIZE / 2.0

LEVER_BARREL_RADIUS = 0.0016
LEVER_WIDTH = 0.012
LEVER_THICKNESS = 0.0038
LEVER_HEIGHT = 0.013
LEVER_TIP_HEIGHT = 0.0032
LEVER_HINGE_Z = -0.0065
LEVER_HINGE_OFFSET = BODY_HALF + 0.0034
HINGE_PAD_DEPTH = 0.0038
HINGE_PAD_CENTER = BODY_HALF - 0.0001

ROLLER_RADIUS = 0.0054
ROLLER_AXLE_RADIUS = 0.0012
ROLLER_AXLE_LENGTH = 0.013
ROLLER_CENTER_Z = BODY_HALF + 0.0062

DIAL_RADIUS = 0.0105
DIAL_THICKNESS = 0.0034
DIAL_ORIGIN_Z = -(BODY_HALF + 0.0014)


def _section(width: float, depth: float, z: float, *, exponent: float = 3.8, segments: int = 40):
    profile = superellipse_profile(width, depth, exponent=exponent, segments=segments)
    return [(x, y, z) for x, y in profile]


def _annulus_mesh(name: str, outer_diameter: float, inner_diameter: float, height: float):
    outer = superellipse_profile(outer_diameter, outer_diameter, exponent=2.0, segments=48)
    inner = list(reversed(superellipse_profile(inner_diameter, inner_diameter, exponent=2.0, segments=48)))
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [inner], height, center=True),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_cube")

    body_color = model.material("body_abs", rgba=(0.17, 0.18, 0.20, 1.0))
    lever_color = model.material("toggle_orange", rgba=(0.93, 0.42, 0.18, 1.0))
    roller_color = model.material("roller_cream", rgba=(0.93, 0.92, 0.86, 1.0))
    roller_detail = model.material("roller_red", rgba=(0.80, 0.18, 0.16, 1.0))
    dial_color = model.material("dial_gray", rgba=(0.46, 0.48, 0.52, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _section(0.028, 0.028, -BODY_HALF),
                    _section(0.034, 0.034, -0.016),
                    _section(0.038, 0.038, -0.010),
                    _section(0.039, 0.039, 0.0),
                    _section(0.038, 0.038, 0.010),
                    _section(0.034, 0.034, 0.016),
                    _section(0.028, 0.028, BODY_HALF),
                ]
            ),
            "body_shell",
        ),
        material=body_color,
        name="shell",
    )

    body.visual(
        Box((0.015, HINGE_PAD_DEPTH, 0.0044)),
        origin=Origin(xyz=(0.0, HINGE_PAD_CENTER, LEVER_HINGE_Z)),
        material=body_color,
        name="front_hinge_pad",
    )
    body.visual(
        Box((0.015, HINGE_PAD_DEPTH, 0.0044)),
        origin=Origin(xyz=(0.0, -HINGE_PAD_CENTER, LEVER_HINGE_Z)),
        material=body_color,
        name="back_hinge_pad",
    )
    body.visual(
        Box((HINGE_PAD_DEPTH, 0.015, 0.0044)),
        origin=Origin(xyz=(HINGE_PAD_CENTER, 0.0, LEVER_HINGE_Z)),
        material=body_color,
        name="right_hinge_pad",
    )
    body.visual(
        Box((HINGE_PAD_DEPTH, 0.015, 0.0044)),
        origin=Origin(xyz=(-HINGE_PAD_CENTER, 0.0, LEVER_HINGE_Z)),
        material=body_color,
        name="left_hinge_pad",
    )

    body.visual(
        _annulus_mesh("top_socket_ring", 0.0165, 0.0108, 0.0028),
        origin=Origin(xyz=(0.0, 0.0, BODY_HALF + 0.0012)),
        material=body_color,
        name="top_socket_ring",
    )
    body.visual(
        Box((0.0023, 0.0064, 0.0078)),
        origin=Origin(xyz=(0.00765, 0.0, BODY_HALF + 0.0033)),
        material=body_color,
        name="top_socket_right_yoke",
    )
    body.visual(
        Box((0.0023, 0.0064, 0.0078)),
        origin=Origin(xyz=(-0.00765, 0.0, BODY_HALF + 0.0033)),
        material=body_color,
        name="top_socket_left_yoke",
    )

    body.visual(
        _annulus_mesh("bottom_dial_bezel_mesh", 0.024, 0.017, 0.0018),
        origin=Origin(xyz=(0.0, 0.0, DIAL_ORIGIN_Z + 0.0009)),
        material=body_color,
        name="bottom_dial_bezel",
    )

    def add_side_lever(
        part_name: str,
        articulation_name: str,
        *,
        origin_xyz: tuple[float, float, float],
        origin_rpy: tuple[float, float, float],
    ) -> None:
        lever = model.part(part_name)
        lever.visual(
            Cylinder(radius=LEVER_BARREL_RADIUS, length=LEVER_WIDTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lever_color,
            name="barrel",
        )
        lever.visual(
            Box((LEVER_WIDTH, LEVER_THICKNESS, LEVER_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.00215, 0.0066)),
            material=lever_color,
            name="paddle",
        )
        lever.visual(
            Box((LEVER_WIDTH * 0.82, LEVER_THICKNESS * 0.72, LEVER_TIP_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.00275, 0.0132)),
            material=lever_color,
            name="thumb_pad",
        )
        model.articulation(
            articulation_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=lever,
            origin=Origin(xyz=origin_xyz, rpy=origin_rpy),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.6,
                velocity=3.0,
                lower=-0.25,
                upper=0.45,
            ),
        )

    add_side_lever(
        "front_lever",
        "body_to_front_lever",
        origin_xyz=(0.0, LEVER_HINGE_OFFSET, LEVER_HINGE_Z),
        origin_rpy=(0.0, 0.0, 0.0),
    )
    add_side_lever(
        "back_lever",
        "body_to_back_lever",
        origin_xyz=(0.0, -LEVER_HINGE_OFFSET, LEVER_HINGE_Z),
        origin_rpy=(0.0, 0.0, math.pi),
    )
    add_side_lever(
        "right_lever",
        "body_to_right_lever",
        origin_xyz=(LEVER_HINGE_OFFSET, 0.0, LEVER_HINGE_Z),
        origin_rpy=(0.0, 0.0, -math.pi / 2.0),
    )
    add_side_lever(
        "left_lever",
        "body_to_left_lever",
        origin_xyz=(-LEVER_HINGE_OFFSET, 0.0, LEVER_HINGE_Z),
        origin_rpy=(0.0, 0.0, math.pi / 2.0),
    )

    top_roller = model.part("top_roller")
    top_roller.visual(
        Sphere(radius=ROLLER_RADIUS),
        material=roller_color,
        name="ball",
    )
    top_roller.visual(
        Cylinder(radius=ROLLER_AXLE_RADIUS, length=ROLLER_AXLE_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_color,
        name="roller_axle",
    )
    top_roller.visual(
        Sphere(radius=0.0015),
        origin=Origin(xyz=(0.0, 0.0044, 0.0013)),
        material=roller_detail,
        name="roller_bump",
    )
    model.articulation(
        "body_to_top_roller",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_roller,
        origin=Origin(xyz=(0.0, 0.0, ROLLER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-1.1,
            upper=1.1,
        ),
    )

    bottom_dial = model.part("bottom_dial")
    bottom_dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -DIAL_THICKNESS / 2.0)),
        material=dial_color,
        name="dial_body",
    )
    bottom_dial.visual(
        Cylinder(radius=0.0114, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=dial_color,
        name="dial_grip",
    )
    bottom_dial.visual(
        Cylinder(radius=0.0011, length=0.0009),
        origin=Origin(xyz=(0.0076, 0.0, -0.0012)),
        material=roller_detail,
        name="dial_index",
    )
    model.articulation(
        "body_to_bottom_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=bottom_dial,
        origin=Origin(xyz=(0.0, 0.0, DIAL_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_lever = object_model.get_part("front_lever")
    back_lever = object_model.get_part("back_lever")
    right_lever = object_model.get_part("right_lever")
    left_lever = object_model.get_part("left_lever")
    top_roller = object_model.get_part("top_roller")
    bottom_dial = object_model.get_part("bottom_dial")

    front_hinge = object_model.get_articulation("body_to_front_lever")
    back_hinge = object_model.get_articulation("body_to_back_lever")
    right_hinge = object_model.get_articulation("body_to_right_lever")
    left_hinge = object_model.get_articulation("body_to_left_lever")
    roller_joint = object_model.get_articulation("body_to_top_roller")
    dial_joint = object_model.get_articulation("body_to_bottom_dial")

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

    ctx.expect_contact(front_lever, body, elem_a="barrel", elem_b="front_hinge_pad", name="front lever is mounted to the front hinge pad")
    ctx.expect_contact(back_lever, body, elem_a="barrel", elem_b="back_hinge_pad", name="back lever is mounted to the back hinge pad")
    ctx.expect_contact(right_lever, body, elem_a="barrel", elem_b="right_hinge_pad", name="right lever is mounted to the right hinge pad")
    ctx.expect_contact(left_lever, body, elem_a="barrel", elem_b="left_hinge_pad", name="left lever is mounted to the left hinge pad")
    ctx.expect_contact(top_roller, body, elem_a="roller_axle", elem_b="top_socket_left_yoke", name="top roller axle bears on the left socket yoke")
    ctx.expect_contact(top_roller, body, elem_a="roller_axle", elem_b="top_socket_right_yoke", name="top roller axle bears on the right socket yoke")
    ctx.expect_contact(bottom_dial, body, elem_a="dial_body", elem_b="bottom_dial_bezel", name="bottom dial rides against the bottom bezel")

    with ctx.pose({front_hinge: 0.0}):
        front_rest = _aabb_center(ctx.part_element_world_aabb(front_lever, elem="thumb_pad"))
    with ctx.pose({front_hinge: front_hinge.motion_limits.upper}):
        front_open = _aabb_center(ctx.part_element_world_aabb(front_lever, elem="thumb_pad"))
    ctx.check(
        "front lever flips outward",
        front_rest is not None and front_open is not None and front_open[1] > front_rest[1] + 0.003,
        details=f"rest={front_rest}, open={front_open}",
    )

    with ctx.pose({back_hinge: 0.0}):
        back_rest = _aabb_center(ctx.part_element_world_aabb(back_lever, elem="thumb_pad"))
    with ctx.pose({back_hinge: back_hinge.motion_limits.upper}):
        back_open = _aabb_center(ctx.part_element_world_aabb(back_lever, elem="thumb_pad"))
    ctx.check(
        "back lever flips outward",
        back_rest is not None and back_open is not None and back_open[1] < back_rest[1] - 0.003,
        details=f"rest={back_rest}, open={back_open}",
    )

    with ctx.pose({right_hinge: 0.0}):
        right_rest = _aabb_center(ctx.part_element_world_aabb(right_lever, elem="thumb_pad"))
    with ctx.pose({right_hinge: right_hinge.motion_limits.upper}):
        right_open = _aabb_center(ctx.part_element_world_aabb(right_lever, elem="thumb_pad"))
    ctx.check(
        "right lever flips outward",
        right_rest is not None and right_open is not None and right_open[0] > right_rest[0] + 0.003,
        details=f"rest={right_rest}, open={right_open}",
    )

    with ctx.pose({left_hinge: 0.0}):
        left_rest = _aabb_center(ctx.part_element_world_aabb(left_lever, elem="thumb_pad"))
    with ctx.pose({left_hinge: left_hinge.motion_limits.upper}):
        left_open = _aabb_center(ctx.part_element_world_aabb(left_lever, elem="thumb_pad"))
    ctx.check(
        "left lever flips outward",
        left_rest is not None and left_open is not None and left_open[0] < left_rest[0] - 0.003,
        details=f"rest={left_rest}, open={left_open}",
    )

    with ctx.pose({roller_joint: 0.0}):
        roller_rest = _aabb_center(ctx.part_element_world_aabb(top_roller, elem="roller_bump"))
    with ctx.pose({roller_joint: roller_joint.motion_limits.upper}):
        roller_rolled = _aabb_center(ctx.part_element_world_aabb(top_roller, elem="roller_bump"))
    ctx.check(
        "top click ball roller rotates in its socket",
        roller_rest is not None and roller_rolled is not None and roller_rolled[2] > roller_rest[2] + 0.0015,
        details=f"rest={roller_rest}, rolled={roller_rolled}",
    )

    with ctx.pose({dial_joint: 0.0}):
        dial_rest = _aabb_center(ctx.part_element_world_aabb(bottom_dial, elem="dial_index"))
    with ctx.pose({dial_joint: 1.0}):
        dial_spun = _aabb_center(ctx.part_element_world_aabb(bottom_dial, elem="dial_index"))
    ctx.check(
        "bottom dial spins about the vertical axis",
        dial_rest is not None and dial_spun is not None and dial_spun[1] > dial_rest[1] + 0.004,
        details=f"rest={dial_rest}, spun={dial_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
