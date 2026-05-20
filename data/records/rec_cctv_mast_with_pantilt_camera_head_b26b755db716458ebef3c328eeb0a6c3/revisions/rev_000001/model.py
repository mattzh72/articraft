from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PAN_X = 0.150
PAN_Z = 0.130
TILT_X = 0.095
TILT_Z = 0.052


def _triangular_wall_plate() -> MeshGeometry:
    """Vertical triangular plate, extruded through X."""

    half_t = 0.006
    points_yz = [(-0.120, 0.020), (0.120, 0.020), (0.0, 0.250)]
    geom = MeshGeometry()

    for x in (-half_t, half_t):
        for y, z in points_yz:
            geom.add_vertex(x, y, z)

    # Back and front triangular faces.
    geom.add_face(0, 2, 1)
    geom.add_face(3, 4, 5)

    # Three rectangular edge bands.
    for a, b in ((0, 1), (1, 2), (2, 0)):
        geom.add_face(a, b, b + 3)
        geom.add_face(a, b + 3, a + 3)

    return geom


def _arm_gusset() -> MeshGeometry:
    """Triangular web under the extension arm, extruded through Y."""

    half_t = 0.006
    points_xz = [(0.000, 0.106), (0.118, 0.106), (0.000, 0.045)]
    geom = MeshGeometry()

    for y in (-half_t, half_t):
        for x, z in points_xz:
            geom.add_vertex(x, y, z)

    geom.add_face(0, 1, 2)
    geom.add_face(3, 5, 4)
    for a, b in ((0, 1), (1, 2), (2, 0)):
        geom.add_face(a, b, b + 3)
        geom.add_face(a, b + 3, a + 3)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_mount_bullet_cctv")

    model.material("powder_black", rgba=(0.020, 0.022, 0.024, 1.0))
    model.material("satin_black", rgba=(0.060, 0.065, 0.070, 1.0))
    model.material("dark_graphite", rgba=(0.120, 0.130, 0.140, 1.0))
    model.material("zinc_screws", rgba=(0.620, 0.640, 0.660, 1.0))
    model.material("smoked_glass", rgba=(0.060, 0.090, 0.110, 0.72))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_geometry(_triangular_wall_plate(), "triangular_wall_plate"),
        material="powder_black",
        name="triangular_plate",
    )
    wall_plate.visual(
        mesh_from_geometry(_arm_gusset(), "arm_gusset"),
        material="powder_black",
        name="triangular_gusset",
    )
    wall_plate.visual(
        Box((0.120, 0.034, 0.028)),
        origin=Origin(xyz=(0.060, 0.0, 0.110)),
        material="powder_black",
        name="extension_arm",
    )
    wall_plate.visual(
        Cylinder(radius=0.037, length=0.030),
        origin=Origin(xyz=(PAN_X, 0.0, PAN_Z - 0.015)),
        material="powder_black",
        name="pan_socket",
    )
    wall_plate.visual(
        Cylinder(radius=0.045, length=0.010),
        origin=Origin(xyz=(PAN_X, 0.0, PAN_Z - 0.005)),
        material="powder_black",
        name="socket_flange",
    )

    for index, (y_pos, z_pos) in enumerate(
        ((-0.065, 0.070), (0.065, 0.070), (-0.030, 0.168), (0.030, 0.168))
    ):
        wall_plate.visual(
            Cylinder(radius=0.010, length=0.005),
            origin=Origin(xyz=(0.0075, y_pos, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
            material="zinc_screws",
            name=f"screw_head_{index}",
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.033, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material="satin_black",
        name="swivel_hub",
    )
    pan_head.visual(
        Box((0.078, 0.128, 0.030)),
        origin=Origin(xyz=(0.039, 0.0, 0.052)),
        material="satin_black",
        name="yoke_bridge",
    )
    for side, y_pos in enumerate((-0.059, 0.059)):
        pan_head.visual(
            Box((0.082, 0.010, 0.082)),
            origin=Origin(xyz=(0.085, y_pos, 0.052)),
            material="satin_black",
            name=f"yoke_cheek_{side}",
        )
        pan_head.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(xyz=(TILT_X, y_pos * 1.0, TILT_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material="zinc_screws",
            name=f"tilt_pin_cap_{side}",
        )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.014, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="zinc_screws",
        name="tilt_trunnion",
    )
    camera.visual(
        Cylinder(radius=0.032, length=0.170),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_graphite",
        name="camera_barrel",
    )
    camera.visual(
        Cylinder(radius=0.033, length=0.020),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="satin_black",
        name="rear_collar",
    )
    camera.visual(
        Cylinder(radius=0.025, length=0.008),
        origin=Origin(xyz=(0.184, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="smoked_glass",
        name="front_glass",
    )
    camera.visual(
        Box((0.190, 0.086, 0.010)),
        origin=Origin(xyz=(0.102, 0.0, 0.035)),
        material="satin_black",
        name="sunshield_top",
    )
    for side, y_pos in enumerate((-0.043, 0.043)):
        camera.visual(
            Box((0.180, 0.006, 0.030)),
            origin=Origin(xyz=(0.098, y_pos, 0.018)),
            material="satin_black",
            name=f"sunshield_side_{side}",
        )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=pan_head,
        origin=Origin(xyz=(PAN_X, 0.0, PAN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=6.0, velocity=2.0),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(TILT_X, 0.0, TILT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.90, effort=4.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    pan_head = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    ctx.expect_gap(
        pan_head,
        wall_plate,
        axis="z",
        positive_elem="swivel_hub",
        negative_elem="pan_socket",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan hub sits on socket",
    )
    ctx.expect_overlap(
        pan_head,
        wall_plate,
        axes="xy",
        elem_a="swivel_hub",
        elem_b="pan_socket",
        min_overlap=0.050,
        name="pan hub centered over socket",
    )
    ctx.expect_within(
        camera,
        pan_head,
        axes="y",
        inner_elem="tilt_trunnion",
        margin=0.004,
        name="tilt trunnion stays within yoke span",
    )

    rest_lens = ctx.part_element_world_aabb(camera, elem="front_glass")
    with ctx.pose({pan: 0.75}):
        panned_lens = ctx.part_element_world_aabb(camera, elem="front_glass")
    with ctx.pose({tilt: 0.70}):
        tilted_lens = ctx.part_element_world_aabb(camera, elem="front_glass")

    def _aabb_center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[axis_index] + hi[axis_index]) * 0.5

    ctx.check(
        "positive pan swings camera sideways",
        rest_lens is not None
        and panned_lens is not None
        and _aabb_center(panned_lens, 1) > _aabb_center(rest_lens, 1) + 0.050,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )
    ctx.check(
        "positive tilt lowers camera nose",
        rest_lens is not None
        and tilted_lens is not None
        and _aabb_center(tilted_lens, 2) < _aabb_center(rest_lens, 2) - 0.050,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()
