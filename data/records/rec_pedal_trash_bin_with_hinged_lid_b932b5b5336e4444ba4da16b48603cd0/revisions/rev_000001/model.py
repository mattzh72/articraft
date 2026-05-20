from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_HEIGHT = 0.405
BODY_RADIUS = 0.171
HINGE_AXIS_X = -0.190
LID_HINGE_Z = 0.411
SERVICE_HINGE_Z = 0.382
PEDAL_PIVOT_X = 0.176
PEDAL_PIVOT_Z = 0.058
LID_OPEN_ANGLE = math.radians(78.0)
PEDAL_TRAVEL = math.radians(32.0)
SERVICE_OPEN_ANGLE = math.radians(52.0)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _build_body_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.157, 0.000),
            (0.160, 0.028),
            (0.164, 0.220),
            (0.167, 0.360),
            (0.169, 0.393),
            (BODY_RADIUS, BODY_HEIGHT),
        ],
        [
            (0.000, 0.008),
            (0.149, 0.013),
            (0.155, 0.030),
            (0.159, 0.220),
            (0.162, 0.360),
            (0.164, 0.399),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_body_hardware_mesh() -> MeshGeometry:
    pedal_housing = BoxGeometry((0.040, 0.118, 0.042)).translate(0.149, 0.0, 0.033)
    pedal_left_ear = BoxGeometry((0.030, 0.016, 0.020)).translate(0.161, -0.061, 0.058)
    pedal_right_ear = BoxGeometry((0.030, 0.016, 0.020)).translate(0.161, 0.061, 0.058)

    rear_band = BoxGeometry((0.018, 0.176, 0.044)).translate(-0.160, 0.0, 0.372)
    service_left_ear = BoxGeometry((0.024, 0.014, 0.018)).translate(-0.178, -0.053, 0.382)
    service_right_ear = BoxGeometry((0.024, 0.014, 0.018)).translate(-0.178, 0.053, 0.382)

    return _merge_geometries(
        pedal_housing,
        pedal_left_ear,
        pedal_right_ear,
        rear_band,
        service_left_ear,
        service_right_ear,
    )


def _build_lid_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.000, 0.050),
            (0.060, 0.048),
            (0.120, 0.034),
            (0.164, 0.012),
            (0.176, 0.000),
        ],
        [
            (0.000, 0.045),
            (0.056, 0.043),
            (0.114, 0.030),
            (0.162, 0.009),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    shell.translate(-HINGE_AXIS_X, 0.0, -0.006)
    return shell


def _build_lid_hardware_mesh() -> MeshGeometry:
    barrel = CylinderGeometry(radius=0.008, height=0.118, radial_segments=28)
    barrel.rotate_x(math.pi / 2.0)
    bridge = BoxGeometry((0.030, 0.120, 0.014)).translate(0.008, 0.0, -0.004)
    return _merge_geometries(barrel, bridge)


def _build_pedal_tread_mesh() -> MeshGeometry:
    left_arm = BoxGeometry((0.052, 0.014, 0.014)).translate(0.022, -0.034, -0.006)
    right_arm = BoxGeometry((0.052, 0.014, 0.014)).translate(0.022, 0.034, -0.006)
    tread = BoxGeometry((0.088, 0.112, 0.014)).translate(0.058, 0.0, -0.012)
    toe_lip = BoxGeometry((0.014, 0.094, 0.022)).translate(0.095, 0.0, -0.005)
    return _merge_geometries(left_arm, right_arm, tread, toe_lip)


def _build_service_cover_panel_mesh() -> MeshGeometry:
    panel = BoxGeometry((0.014, 0.092, 0.054)).translate(0.008, 0.0, -0.028)
    hinge_bridge = BoxGeometry((0.010, 0.092, 0.012)).translate(0.004, 0.0, -0.006)
    return _merge_geometries(panel, hinge_bridge)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soft_close_step_bin")

    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.15, 0.16, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_build_body_shell_mesh(), "body_shell"),
        material=brushed_steel,
        name="body_shell",
    )
    body.visual(
        mesh_from_geometry(_build_body_hardware_mesh(), "body_hardware"),
        material=dark_trim,
        name="body_hardware",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_RADIUS, length=BODY_HEIGHT),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    lid = model.part("main_lid")
    lid.visual(
        mesh_from_geometry(_build_lid_shell_mesh(), "lid_shell"),
        material=brushed_steel,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_geometry(_build_lid_hardware_mesh(), "lid_hardware"),
        material=dark_trim,
        name="lid_hardware",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.178, length=0.060),
        mass=1.2,
        origin=Origin(xyz=(-HINGE_AXIS_X, 0.0, 0.020)),
    )

    pedal = model.part("pedal")
    pedal.visual(
        mesh_from_geometry(_build_pedal_tread_mesh(), "pedal_tread"),
        material=satin_black,
        name="pedal_tread",
    )
    pedal.visual(
        Cylinder(radius=0.0045, length=0.106),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="pedal_pivot",
    )
    pedal.inertial = Inertial.from_geometry(
        Box((0.108, 0.112, 0.036)),
        mass=0.45,
        origin=Origin(xyz=(0.052, 0.0, -0.008)),
    )

    service_cover = model.part("service_cover")
    service_cover.visual(
        mesh_from_geometry(_build_service_cover_panel_mesh(), "service_cover_panel"),
        material=dark_trim,
        name="service_cover_panel",
    )
    service_cover.visual(
        Cylinder(radius=0.0045, length=0.092),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="service_cover_barrel",
    )
    service_cover.inertial = Inertial.from_geometry(
        Box((0.020, 0.094, 0.058)),
        mass=0.12,
        origin=Origin(xyz=(0.008, 0.0, -0.026)),
    )

    model.articulation(
        "body_to_main_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, LID_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=LID_OPEN_ANGLE,
        ),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(PEDAL_PIVOT_X, 0.0, PEDAL_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=0.0,
            upper=PEDAL_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_service_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_cover,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, SERVICE_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=SERVICE_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    body = object_model.get_part("body")
    lid = object_model.get_part("main_lid")
    pedal = object_model.get_part("pedal")
    service_cover = object_model.get_part("service_cover")

    lid_joint = object_model.get_articulation("body_to_main_lid")
    pedal_joint = object_model.get_articulation("body_to_pedal")
    service_joint = object_model.get_articulation("body_to_service_cover")

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_penetration=0.001,
            max_gap=0.012,
            name="main lid rests just above the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.30,
            name="main lid covers the cylindrical opening",
        )

    ctx.expect_origin_gap(
        pedal,
        body,
        axis="x",
        min_gap=0.12,
        max_gap=0.20,
        name="pedal pivot sits ahead of the body centerline",
    )
    ctx.expect_gap(
        body,
        service_cover,
        axis="x",
        positive_elem="body_shell",
        negative_elem="service_cover_panel",
        min_gap=0.003,
        max_gap=0.020,
        name="service cover sits just behind the rear shell",
    )

    with ctx.pose({lid_joint: 0.0}):
        lid_closed = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: LID_OPEN_ANGLE}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            min_gap=0.015,
            name="open lid lifts clear of the rim",
        )
    ctx.check(
        "main lid opens upward",
        lid_closed is not None
        and lid_open is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.060,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    with ctx.pose({pedal_joint: 0.0}):
        pedal_rest = ctx.part_element_world_aabb(pedal, elem="pedal_tread")
    with ctx.pose({pedal_joint: PEDAL_TRAVEL}):
        pedal_pressed = ctx.part_element_world_aabb(pedal, elem="pedal_tread")
    ctx.check(
        "pedal swings downward when pressed",
        pedal_rest is not None
        and pedal_pressed is not None
        and pedal_pressed[0][2] < pedal_rest[0][2] - 0.020,
        details=f"rest={pedal_rest}, pressed={pedal_pressed}",
    )

    with ctx.pose({service_joint: 0.0}):
        cover_closed = ctx.part_element_world_aabb(service_cover, elem="service_cover_panel")
    with ctx.pose({service_joint: SERVICE_OPEN_ANGLE}):
        cover_open = ctx.part_element_world_aabb(service_cover, elem="service_cover_panel")
    ctx.check(
        "service cover opens rearward from its hinge band",
        cover_closed is not None
        and cover_open is not None
        and cover_open[0][0] < cover_closed[0][0] - 0.012,
        details=f"closed={cover_closed}, open={cover_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
