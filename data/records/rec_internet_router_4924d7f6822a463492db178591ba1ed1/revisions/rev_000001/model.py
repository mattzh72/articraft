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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_WIDTH = 0.285
BODY_DEPTH = 0.190
TRAY_HEIGHT = 0.014
SHELL_BASE_Z = 0.012
SHELL_HEIGHT = 0.022
SHELL_TOP_WIDTH = 0.166
SHELL_TOP_DEPTH = 0.108
SHELL_TOP_SHIFT_Y = 0.010
BODY_TOP_Z = SHELL_BASE_Z + SHELL_HEIGHT


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _build_router_shell_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    half_w = BODY_WIDTH * 0.5
    half_d = BODY_DEPTH * 0.5
    top_half_w = SHELL_TOP_WIDTH * 0.5
    top_front_y = SHELL_TOP_SHIFT_Y - SHELL_TOP_DEPTH * 0.5
    top_back_y = SHELL_TOP_SHIFT_Y + SHELL_TOP_DEPTH * 0.5

    vertices = [
        (-half_w, -half_d, 0.0),
        (half_w, -half_d, 0.0),
        (half_w, half_d, 0.0),
        (-half_w, half_d, 0.0),
        (-top_half_w, top_front_y, SHELL_HEIGHT),
        (top_half_w, top_front_y, SHELL_HEIGHT),
        (top_half_w, top_back_y, SHELL_HEIGHT),
        (-top_half_w, top_back_y, SHELL_HEIGHT),
    ]
    ids = [geom.add_vertex(*vertex) for vertex in vertices]

    _add_quad(geom, ids[0], ids[1], ids[2], ids[3])  # base
    _add_quad(geom, ids[4], ids[7], ids[6], ids[5])  # top plateau
    _add_quad(geom, ids[0], ids[4], ids[5], ids[1])  # front facet
    _add_quad(geom, ids[1], ids[5], ids[6], ids[2])  # right facet
    _add_quad(geom, ids[2], ids[6], ids[7], ids[3])  # rear facet
    _add_quad(geom, ids[3], ids[7], ids[4], ids[0])  # left facet
    return geom


def _build_antenna_part(part, material, *, collar_axis: str) -> None:
    if collar_axis == "x":
        collar_rpy = (0.0, math.pi / 2.0, 0.0)
    else:
        collar_rpy = (math.pi / 2.0, 0.0, 0.0)

    part.visual(
        Cylinder(radius=0.0055, length=0.022),
        origin=Origin(rpy=collar_rpy),
        material=material,
        name="hinge_collar",
    )
    part.visual(
        Box((0.014, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=material,
        name="stem_block",
    )
    part.visual(
        Box((0.018, 0.008, 0.134)),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=material,
        name="antenna_blade",
    )
    part.visual(
        Box((0.014, 0.007, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.157)),
        material=material,
        name="antenna_tip",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.020, 0.010, 0.170)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )


def _element_center(ctx: TestContext, part, elem: str) -> tuple[float, float, float] | None:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_home_router")

    body_graphite = model.material("body_graphite", rgba=(0.14, 0.15, 0.16, 1.0))
    body_black = model.material("body_black", rgba=(0.08, 0.09, 0.10, 1.0))
    antenna_black = model.material("antenna_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_charcoal = model.material("satin_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    light_ring = model.material("light_ring", rgba=(0.55, 0.84, 0.98, 0.70))

    front_face_tilt = math.atan2(
        SHELL_HEIGHT,
        (BODY_DEPTH * 0.5) - (SHELL_TOP_SHIFT_Y - SHELL_TOP_DEPTH * 0.5),
    )
    knob_surface_y = -0.066
    front_face_start_y = -BODY_DEPTH * 0.5
    front_face_end_y = SHELL_TOP_SHIFT_Y - SHELL_TOP_DEPTH * 0.5
    knob_surface_z = SHELL_BASE_Z + SHELL_HEIGHT * (
        (knob_surface_y - front_face_start_y) / (front_face_end_y - front_face_start_y)
    )

    body = model.part("router_body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, TRAY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, TRAY_HEIGHT * 0.5)),
        material=body_black,
        name="lower_tray",
    )
    shell_mesh = mesh_from_geometry(_build_router_shell_mesh(), "router_shell")
    router_shell = body.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, SHELL_BASE_Z)),
        material=body_graphite,
        name="router_shell",
    )
    body.visual(
        Box((0.090, 0.022, 0.0025)),
        origin=Origin(xyz=(0.0, -0.018, BODY_TOP_Z - 0.0012)),
        material=satin_charcoal,
        name="top_trim_panel",
    )
    knob_mount_thickness = 0.005
    knob_mount_center_offset = 0.0022
    body.visual(
        Box((0.034, 0.016, knob_mount_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                knob_surface_y - math.sin(front_face_tilt) * knob_mount_center_offset,
                knob_surface_z + math.cos(front_face_tilt) * knob_mount_center_offset,
            ),
            rpy=(front_face_tilt, 0.0, 0.0),
        ),
        material=body_black,
        name="knob_mount_pod",
    )
    body.visual(
        Box((0.022, 0.016, 0.009)),
        origin=Origin(xyz=(-0.060, 0.084, 0.0285)),
        material=body_black,
        name="rear_left_mount",
    )
    body.visual(
        Box((0.022, 0.016, 0.009)),
        origin=Origin(xyz=(0.060, 0.084, 0.0285)),
        material=body_black,
        name="rear_right_mount",
    )
    body.visual(
        Box((0.046, 0.022, 0.009)),
        origin=Origin(xyz=(-0.120, 0.020, 0.0285)),
        material=body_black,
        name="left_side_mount",
    )
    body.visual(
        Box((0.046, 0.022, 0.009)),
        origin=Origin(xyz=(0.120, 0.020, 0.0285)),
        material=body_black,
        name="right_side_mount",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(-0.105, -0.060, 0.0015)),
        material=satin_charcoal,
        name="left_foot",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.105, -0.060, 0.0015)),
        material=satin_charcoal,
        name="right_foot",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_TOP_Z)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z * 0.5)),
    )

    rear_left = model.part("rear_left_antenna")
    rear_right = model.part("rear_right_antenna")
    side_left = model.part("left_side_antenna")
    side_right = model.part("right_side_antenna")
    _build_antenna_part(rear_left, antenna_black, collar_axis="x")
    _build_antenna_part(rear_right, antenna_black, collar_axis="x")
    _build_antenna_part(side_left, antenna_black, collar_axis="y")
    _build_antenna_part(side_right, antenna_black, collar_axis="y")

    rear_hinge_z = 0.033 + 0.0055
    side_hinge_z = 0.033 + 0.0055

    model.articulation(
        "body_to_rear_left_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_left,
        origin=Origin(xyz=(-0.060, 0.084, rear_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.40,
            upper=1.15,
        ),
    )
    model.articulation(
        "body_to_rear_right_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_right,
        origin=Origin(xyz=(0.060, 0.084, rear_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.40,
            upper=1.15,
        ),
    )
    model.articulation(
        "body_to_left_side_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_left,
        origin=Origin(xyz=(-0.137, 0.020, side_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.30,
            upper=1.05,
        ),
    )
    model.articulation(
        "body_to_right_side_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_right,
        origin=Origin(xyz=(0.137, 0.020, side_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.30,
            upper=1.05,
        ),
    )

    knob = model.part("brightness_knob")
    knob.visual(
        Cylinder(radius=0.0032, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_charcoal,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=0.0108, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=body_black,
        name="knob_base",
    )
    knob.visual(
        Cylinder(radius=0.0112, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0120)),
        material=light_ring,
        name="light_ring",
    )
    knob.visual(
        Cylinder(radius=0.0094, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0150)),
        material=satin_charcoal,
        name="knob_cap",
    )
    knob.visual(
        Box((0.0042, 0.0018, 0.0022)),
        origin=Origin(xyz=(0.0068, 0.0, 0.0191)),
        material=light_ring,
        name="knob_indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.015),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    knob_clearance = knob_mount_center_offset + knob_mount_thickness * 0.5
    knob_origin = Origin(
        xyz=(
            0.0,
            knob_surface_y - math.sin(front_face_tilt) * knob_clearance,
            knob_surface_z + math.cos(front_face_tilt) * knob_clearance,
        ),
        rpy=(front_face_tilt, 0.0, 0.0),
    )
    model.articulation(
        "body_to_brightness_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=knob_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("router_body")
    rear_left = object_model.get_part("rear_left_antenna")
    rear_right = object_model.get_part("rear_right_antenna")
    side_left = object_model.get_part("left_side_antenna")
    side_right = object_model.get_part("right_side_antenna")
    knob = object_model.get_part("brightness_knob")

    rear_left_joint = object_model.get_articulation("body_to_rear_left_antenna")
    rear_right_joint = object_model.get_articulation("body_to_rear_right_antenna")
    side_left_joint = object_model.get_articulation("body_to_left_side_antenna")
    side_right_joint = object_model.get_articulation("body_to_right_side_antenna")
    knob_joint = object_model.get_articulation("body_to_brightness_knob")

    ctx.expect_contact(
        rear_left,
        body,
        elem_a="hinge_collar",
        elem_b="rear_left_mount",
        name="rear left antenna collar sits on its rear mount",
    )
    ctx.expect_contact(
        rear_right,
        body,
        elem_a="hinge_collar",
        elem_b="rear_right_mount",
        name="rear right antenna collar sits on its rear mount",
    )
    ctx.expect_contact(
        side_left,
        body,
        elem_a="hinge_collar",
        elem_b="left_side_mount",
        name="left side antenna collar sits on its side mount",
    )
    ctx.expect_contact(
        side_right,
        body,
        elem_a="hinge_collar",
        elem_b="right_side_mount",
        name="right side antenna collar sits on its side mount",
    )
    ctx.expect_contact(
        knob,
        body,
        elem_a="knob_shaft",
        elem_b="knob_mount_pod",
        contact_tol=0.0008,
        name="brightness knob seats on its front mounting pod",
    )

    rear_left_rest = _element_center(ctx, rear_left, "antenna_blade")
    rear_right_rest = _element_center(ctx, rear_right, "antenna_blade")
    side_left_rest = _element_center(ctx, side_left, "antenna_blade")
    side_right_rest = _element_center(ctx, side_right, "antenna_blade")
    knob_indicator_rest = _element_center(ctx, knob, "knob_indicator")
    knob_cap_rest = _element_center(ctx, knob, "knob_cap")

    with ctx.pose(
        {
            rear_left_joint: 0.85,
            rear_right_joint: 0.85,
            side_left_joint: 0.80,
            side_right_joint: 0.80,
            knob_joint: 1.10,
        }
    ):
        rear_left_tilted = _element_center(ctx, rear_left, "antenna_blade")
        rear_right_tilted = _element_center(ctx, rear_right, "antenna_blade")
        side_left_tilted = _element_center(ctx, side_left, "antenna_blade")
        side_right_tilted = _element_center(ctx, side_right, "antenna_blade")
        knob_indicator_turned = _element_center(ctx, knob, "knob_indicator")
        knob_cap_turned = _element_center(ctx, knob, "knob_cap")

    ctx.check(
        "rear antennas pitch backward from the body",
        rear_left_rest is not None
        and rear_left_tilted is not None
        and rear_right_rest is not None
        and rear_right_tilted is not None
        and rear_left_tilted[1] > rear_left_rest[1] + 0.030
        and rear_right_tilted[1] > rear_right_rest[1] + 0.030
        and rear_left_tilted[2] < rear_left_rest[2] - 0.020
        and rear_right_tilted[2] < rear_right_rest[2] - 0.020,
        details=(
            f"rear_left_rest={rear_left_rest}, rear_left_tilted={rear_left_tilted}, "
            f"rear_right_rest={rear_right_rest}, rear_right_tilted={rear_right_tilted}"
        ),
    )
    ctx.check(
        "side antennas swing outward from the chassis flanks",
        side_left_rest is not None
        and side_left_tilted is not None
        and side_right_rest is not None
        and side_right_tilted is not None
        and side_left_tilted[0] < side_left_rest[0] - 0.030
        and side_right_tilted[0] > side_right_rest[0] + 0.030
        and side_left_tilted[2] < side_left_rest[2] - 0.020
        and side_right_tilted[2] < side_right_rest[2] - 0.020,
        details=(
            f"side_left_rest={side_left_rest}, side_left_tilted={side_left_tilted}, "
            f"side_right_rest={side_right_rest}, side_right_tilted={side_right_tilted}"
        ),
    )
    ctx.check(
        "brightness knob rotates about its local shaft",
        knob_indicator_rest is not None
        and knob_indicator_turned is not None
        and knob_cap_rest is not None
        and knob_cap_turned is not None
        and math.dist(knob_indicator_rest, knob_indicator_turned) > 0.004
        and math.dist(knob_cap_rest, knob_cap_turned) < 0.001,
        details=(
            f"indicator_rest={knob_indicator_rest}, indicator_turned={knob_indicator_turned}, "
            f"cap_rest={knob_cap_rest}, cap_turned={knob_cap_turned}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
