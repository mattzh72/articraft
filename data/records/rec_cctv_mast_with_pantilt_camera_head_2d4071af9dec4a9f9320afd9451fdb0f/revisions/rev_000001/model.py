from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded rectangular solid centered on the origin."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _pan_yoke_mesh() -> cq.Workplane:
    """Connected rotating turntable, gearbox deck, and bored tilt yoke."""
    turntable = cq.Workplane("XY").cylinder(0.070, 0.120).translate((0.0, 0.0, 0.035))
    deck = (
        cq.Workplane("XY")
        .box(0.34, 0.34, 0.035)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.10, 0.0, 0.0875))
    )

    # Two thick cheek plates flank the camera.  A bridge at the rear turns the
    # cheeks into one cast/bolted yoke, while the holes leave real clearance for
    # the camera trunnion.
    cheek_0 = (
        cq.Workplane("XY")
        .box(0.11, 0.034, 0.31)
        .edges("|Z")
        .fillet(0.010)
        .translate((0.16, 0.153, 0.250))
    )
    cheek_1 = cheek_0.mirror("XZ")
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.060, 0.340, 0.230)
        .edges("|Z")
        .fillet(0.010)
        .translate((0.075, 0.0, 0.235))
    )
    bearing_0 = (
        cq.Workplane("XZ")
        .center(0.16, 0.285)
        .circle(0.045)
        .extrude(0.044, both=True)
        .translate((0.0, 0.153, 0.0))
    )
    bearing_1 = bearing_0.mirror("XZ")

    yoke = (
        turntable.union(deck)
        .union(cheek_0)
        .union(cheek_1)
        .union(rear_bridge)
        .union(bearing_0)
        .union(bearing_1)
    )
    bore = cq.Workplane("XZ").center(0.16, 0.285).circle(0.030).extrude(0.50, both=True)
    return yoke.cut(bore)


def _camera_body_mesh() -> cq.Workplane:
    """Weatherproof camera body with molded rear cap and forward lens barrel."""
    body = _rounded_box((0.38, 0.190, 0.165), 0.018).translate((0.210, 0.0, 0.0))
    rear_cap = (
        _rounded_box((0.055, 0.185, 0.155), 0.012)
        .translate((-0.005, 0.0, 0.0))
    )
    nose = (
        cq.Workplane("YZ")
        .circle(0.085)
        .extrude(0.060)
        .translate((0.400, 0.0, 0.0))
    )
    lower_heat_sink = (
        cq.Workplane("XY")
        .box(0.250, 0.160, 0.020)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.205, 0.0, -0.092))
    )
    return body.union(rear_cap).union(nose).union(lower_heat_sink)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cctv_mast_pan_tilt")

    galvanized = model.material(name="galvanized_steel", rgba=(0.56, 0.58, 0.56, 1.0))
    dark_steel = model.material(name="dark_powder_coat", rgba=(0.05, 0.055, 0.055, 1.0))
    warm_gray = model.material(name="warm_gray_enclosure", rgba=(0.74, 0.75, 0.71, 1.0))
    black = model.material(name="black_rubber", rgba=(0.01, 0.011, 0.012, 1.0))
    glass = model.material(name="smoked_glass", rgba=(0.02, 0.04, 0.055, 0.78))
    cable_mat = model.material(name="matte_black_cable", rgba=(0.0, 0.0, 0.0, 1.0))
    warning_yellow = model.material(name="warning_label_yellow", rgba=(0.95, 0.78, 0.10, 1.0))

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.32, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=galvanized,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.070, length=2.420),
        origin=Origin(xyz=(0.0, 0.0, 1.250)),
        material=galvanized,
        name="support_pole",
    )
    mast.visual(
        Cylinder(radius=0.130, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 2.485)),
        material=galvanized,
        name="top_flange",
    )

    # Four anchor studs, nuts, and pole gussets make the base read as an
    # installed outdoor mast rather than a decorative stand.
    for index, (x, y) in enumerate(((0.225, 0.225), (-0.225, 0.225), (-0.225, -0.225), (0.225, -0.225))):
        mast.visual(
            Cylinder(radius=0.014, length=0.070),
            origin=Origin(xyz=(x, y, 0.075)),
            material=dark_steel,
            name=f"anchor_stud_{index}",
        )
        mast.visual(
            Cylinder(radius=0.030, length=0.020),
            origin=Origin(xyz=(x, y, 0.050)),
            material=dark_steel,
            name=f"anchor_nut_{index}",
        )

    for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        mast.visual(
            Box((0.018, 0.185, 0.270)),
            origin=Origin(xyz=(0.0, 0.090, 0.165), rpy=(0.0, 0.0, yaw)),
            material=galvanized,
            name=f"base_gusset_{index}",
        )

    mast.visual(
        Box((0.105, 0.230, 0.320)),
        origin=Origin(xyz=(-0.1225, 0.0, 1.050)),
        material=warm_gray,
        name="service_box",
    )
    mast.visual(
        Box((0.014, 0.200, 0.285)),
        origin=Origin(xyz=(-0.1820, 0.0, 1.050)),
        material=galvanized,
        name="service_door_panel",
    )
    mast.visual(
        Box((0.010, 0.030, 0.090)),
        origin=Origin(xyz=(-0.191, 0.100, 1.050)),
        material=dark_steel,
        name="service_latch",
    )
    mast.visual(
        Box((0.011, 0.120, 0.045)),
        origin=Origin(xyz=(-0.1915, -0.020, 1.155)),
        material=warning_yellow,
        name="surveillance_label",
    )

    # A real installation needs weatherproof cabling.  The conduit runs up the
    # back of the pole and is tied to it by metal straps.
    mast.visual(
        Cylinder(radius=0.011, length=1.250),
        origin=Origin(xyz=(-0.095, 0.0, 1.725)),
        material=cable_mat,
        name="vertical_conduit",
    )
    for index, z in enumerate((1.240, 1.620, 2.000, 2.360)):
        mast.visual(
            Box((0.055, 0.035, 0.030)),
            origin=Origin(xyz=(-0.071, 0.0, z)),
            material=dark_steel,
            name=f"conduit_strap_{index}",
        )
    mast.visual(
        Cylinder(radius=0.012, length=0.155),
        origin=Origin(xyz=(-0.110, 0.0, 1.225), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cable_mat,
        name="box_cable_gland",
    )

    pan_base = model.part("pan_base")
    pan_base.visual(
        mesh_from_cadquery(_pan_yoke_mesh(), "pan_yoke"),
        material=galvanized,
        name="pan_yoke",
    )
    pan_base.visual(
        Box((0.055, 0.030, 0.040)),
        origin=Origin(xyz=(-0.070, 0.0, 0.125)),
        material=black,
        name="cable_exit_gland",
    )

    camera = model.part("camera")
    camera.visual(
        mesh_from_cadquery(_camera_body_mesh(), "camera_housing"),
        material=warm_gray,
        name="housing",
    )
    camera.visual(
        Box((0.455, 0.245, 0.026)),
        origin=Origin(xyz=(0.215, 0.0, 0.095)),
        material=warm_gray,
        name="sun_shield_top",
    )
    camera.visual(
        Box((0.345, 0.020, 0.075)),
        origin=Origin(xyz=(0.315, 0.122, 0.062)),
        material=warm_gray,
        name="sun_shield_side_0",
    )
    camera.visual(
        Box((0.345, 0.020, 0.075)),
        origin=Origin(xyz=(0.315, -0.122, 0.062)),
        material=warm_gray,
        name="sun_shield_side_1",
    )
    camera.visual(
        Cylinder(radius=0.070, length=0.115),
        origin=Origin(xyz=(0.495, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.057, length=0.010),
        origin=Origin(xyz=(0.557, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    camera.visual(
        Cylinder(radius=0.030, length=0.335),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_pin",
    )
    for index, y in enumerate((-0.045, 0.0, 0.045)):
        camera.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.557, y, -0.050)),
            material=glass,
            name=f"ir_led_{index}",
        )

    model.articulation(
        "mast_to_pan",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=pan_base,
        origin=Origin(xyz=(0.0, 0.0, 2.510)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.2, lower=-2.90, upper=2.90),
        motion_properties=MotionProperties(damping=0.4, friction=0.12),
    )

    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_base,
        child=camera,
        origin=Origin(xyz=(0.160, 0.0, 0.285)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.8, lower=-0.70, upper=0.65),
        motion_properties=MotionProperties(damping=0.25, friction=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    pan_base = object_model.get_part("pan_base")
    camera = object_model.get_part("camera")
    pan_joint = object_model.get_articulation("mast_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_camera")

    ctx.allow_overlap(
        camera,
        pan_base,
        elem_a="trunnion_pin",
        elem_b="pan_yoke",
        reason=(
            "The camera trunnion is intentionally represented as a captured shaft "
            "seated in the yoke's bearing bores."
        ),
    )
    ctx.expect_contact(
        pan_base,
        mast,
        elem_a="pan_yoke",
        elem_b="top_flange",
        contact_tol=0.003,
        name="pan turntable is seated on the mast flange",
    )
    ctx.expect_within(
        camera,
        pan_base,
        axes="y",
        inner_elem="housing",
        outer_elem="pan_yoke",
        margin=0.010,
        name="camera housing fits between yoke cheeks",
    )
    ctx.expect_within(
        camera,
        pan_base,
        axes="xz",
        inner_elem="trunnion_pin",
        outer_elem="pan_yoke",
        margin=0.006,
        name="trunnion pin is coaxial with the tilt bearings",
    )
    ctx.expect_overlap(
        camera,
        pan_base,
        axes="y",
        elem_a="trunnion_pin",
        elem_b="pan_yoke",
        min_overlap=0.080,
        name="trunnion pin remains captured across the yoke width",
    )

    rest_aabb = ctx.part_world_aabb(camera)
    with ctx.pose({tilt_joint: 0.55}):
        up_aabb = ctx.part_world_aabb(camera)
    with ctx.pose({tilt_joint: -0.55}):
        down_aabb = ctx.part_world_aabb(camera)
    ctx.check(
        "tilt raises and lowers the camera nose",
        rest_aabb is not None
        and up_aabb is not None
        and down_aabb is not None
        and up_aabb[1][2] > rest_aabb[1][2] + 0.015
        and down_aabb[1][2] < rest_aabb[1][2] + 0.010,
        details=f"rest={rest_aabb}, up={up_aabb}, down={down_aabb}",
    )

    rest_pos = ctx.part_world_position(camera)
    with ctx.pose({pan_joint: 1.2}):
        yaw_pos = ctx.part_world_position(camera)
    ctx.check(
        "pan joint swings the head around the mast",
        rest_pos is not None
        and yaw_pos is not None
        and abs(yaw_pos[1] - rest_pos[1]) > 0.09,
        details=f"rest={rest_pos}, yawed={yaw_pos}",
    )

    return ctx.report()


object_model = build_object_model()
