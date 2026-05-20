from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CASE_W = 0.74
CASE_D = 0.40
CASE_H = 0.44
CASE_CENTER_Z = 0.34
LID_W = 0.76
LID_D = 0.44
LID_H = 0.16
HINGE_Y = -LID_D / 2.0
HINGE_Z = CASE_CENTER_Z + CASE_H / 2.0
HANDLE_TRAVEL = 0.36
DRAWER_TRAVEL = 0.22


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Small CadQuery helper that gracefully falls back if a fillet is too tight."""
    shape = cq.Workplane("XY").box(*size)
    if radius <= 0.0:
        return shape
    try:
        return shape.edges("|Z").fillet(radius)
    except Exception:
        return shape


def _lower_case_shape() -> cq.Workplane:
    shell = _rounded_box((CASE_W, CASE_D, CASE_H), 0.035)

    # A real toolbox lower case is a hollow tub under the lid.  The top cut leaves
    # a broad rim, while the side cut leaves a shallow drawer bay through one
    # flank.
    top_cavity = cq.Workplane("XY").box(0.58, 0.28, 0.34).translate((0.0, 0.0, 0.20))
    drawer_bay = cq.Workplane("XY").box(0.40, 0.25, 0.16).translate((0.24, 0.03, 0.02))
    return shell.cut(top_cavity).cut(drawer_bay)


def _lid_shape() -> cq.Workplane:
    lid = _rounded_box((LID_W, LID_D, LID_H), 0.026)
    # Shallow underside relief so the lid reads as a molded cover rather than a
    # plain slab.
    underside_relief = cq.Workplane("XY").box(0.62, 0.30, 0.05).translate((0.0, 0.0, -0.07))
    return lid.cut(underside_relief)


def _handle_frame_shape() -> cq.Workplane:
    rail_w = 0.024
    rail_d = 0.020
    rail_h = 0.76
    rail_z = -0.10
    x_span = 0.52
    top = cq.Workplane("XY").box(x_span + rail_w, 0.030, 0.045).translate((0.0, 0.0, 0.300))
    rail_0 = cq.Workplane("XY").box(rail_w, rail_d, rail_h).translate((-x_span / 2.0, 0.0, rail_z))
    rail_1 = cq.Workplane("XY").box(rail_w, rail_d, rail_h).translate((x_span / 2.0, 0.0, rail_z))
    return top.union(rail_0).union(rail_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stackable_rolling_toolbox")

    red = model.material("molded_red", rgba=(0.72, 0.05, 0.03, 1.0))
    dark_red = model.material("dark_red_rib", rgba=(0.42, 0.02, 0.02, 1.0))
    black = model.material("black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    gray = model.material("dark_gray_hardware", rgba=(0.19, 0.20, 0.20, 1.0))
    metal = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    latch_yellow = model.material("yellow_latches", rgba=(0.96, 0.70, 0.08, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_lower_case_shape(), "lower_shell", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, CASE_CENTER_Z)),
        material=red,
        name="lower_shell",
    )

    # Front reinforcing ribs and latch details on the stout lower case.
    case.visual(
        Box((0.66, 0.016, 0.030)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 + 0.008, 0.48)),
        material=dark_red,
        name="front_upper_rib",
    )
    case.visual(
        Box((0.62, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 + 0.007, 0.25)),
        material=dark_red,
        name="front_lower_rib",
    )
    for i, x in enumerate((-0.21, 0.21)):
        case.visual(
            Box((0.085, 0.020, 0.060)),
            origin=Origin(xyz=(x, CASE_D / 2.0 + 0.008, 0.510)),
            material=latch_yellow,
            name=f"front_latch_{i}",
        )
        case.visual(
            Box((0.060, 0.016, 0.018)),
            origin=Origin(xyz=(x, CASE_D / 2.0 + 0.007, 0.480)),
            material=gray,
            name=f"latch_hinge_{i}",
        )

    # Side drawer surround on the positive-X flank.
    for name, y, z, size in (
        ("drawer_trim_top", 0.03, 0.455, (0.018, 0.300, 0.018)),
        ("drawer_trim_bottom", 0.03, 0.265, (0.018, 0.300, 0.018)),
        ("drawer_trim_rear", -0.125, 0.360, (0.018, 0.018, 0.190)),
        ("drawer_trim_front", 0.185, 0.360, (0.018, 0.018, 0.190)),
    ):
        case.visual(
            Box(size),
            origin=Origin(xyz=(CASE_W / 2.0 + 0.009, y, z)),
            material=gray,
            name=name,
        )
    for i, y in enumerate((-0.091, 0.151)):
        case.visual(
            Box((0.300, 0.008, 0.026)),
            origin=Origin(xyz=(0.235, y, 0.350)),
            material=gray,
            name=f"drawer_runner_{i}",
        )

    # Rear telescoping guide sleeves.  They are represented as solid sleeve
    # proxies; scoped test allowances document the hidden sliding fit.
    sleeve_xs = (-0.31, 0.31)
    for i, x in enumerate(sleeve_xs):
        case.visual(
            Box((0.046, 0.036, 0.390)),
            origin=Origin(xyz=(x, -CASE_D / 2.0 - 0.030, 0.355)),
            material=black,
            name=f"rear_sleeve_{i}",
        )
        case.visual(
            Box((0.074, 0.028, 0.050)),
            origin=Origin(xyz=(x, -CASE_D / 2.0 - 0.030, 0.535)),
            material=gray,
            name=f"sleeve_cap_{i}",
        )
        case.visual(
            Box((0.060, 0.026, 0.280)),
            origin=Origin(xyz=(x, -CASE_D / 2.0 - 0.006, 0.350)),
            material=black,
            name=f"sleeve_standoff_{i}",
        )

    # Rear hinge hardware mounted to the case, with a central gap for the lid
    # knuckle.
    for i, x in enumerate((-0.20, 0.20)):
        case.visual(
            Cylinder(radius=0.018, length=0.10),
            origin=Origin(xyz=(x, HINGE_Y - 0.018, HINGE_Z + 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gray,
            name=f"hinge_barrel_{i}",
        )
        case.visual(
            Box((0.11, 0.040, 0.032)),
            origin=Origin(xyz=(x, HINGE_Y + 0.001, HINGE_Z - 0.018)),
            material=gray,
            name=f"hinge_leaf_{i}",
        )

    # Wheel mounts and front feet keep the rolling case visibly supported.
    for i, x in enumerate((-0.405, 0.405)):
        case.visual(
            Box((0.020, 0.070, 0.100)),
            origin=Origin(xyz=(-0.366 if x < 0.0 else 0.366, -0.145, 0.115)),
            material=black,
            name=f"wheel_bracket_{i}",
        )
    for i, x in enumerate((-0.24, 0.24)):
        case.visual(
            Box((0.095, 0.070, 0.050)),
            origin=Origin(xyz=(x, 0.155, 0.095)),
            material=rubber,
            name=f"front_foot_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "lid_shell", tolerance=0.0015),
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_H / 2.0)),
        material=red,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.016, length=0.28),
        origin=Origin(xyz=(0.0, -0.012, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gray,
        name="lid_hinge_barrel",
    )
    lid.visual(
        mesh_from_cadquery(_rounded_box((0.34, 0.092, 0.040), 0.018), "molded_grip", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.245, LID_H + 0.020)),
        material=black,
        name="molded_grip",
    )
    lid.visual(
        Box((0.42, 0.125, 0.010)),
        origin=Origin(xyz=(0.0, 0.245, LID_H + 0.004)),
        material=dark_red,
        name="grip_recess",
    )
    for i, (x, y) in enumerate(((-0.285, 0.080), (0.285, 0.080), (-0.285, 0.360), (0.285, 0.360))):
        lid.visual(
            Box((0.090, 0.058, 0.018)),
            origin=Origin(xyz=(x, y, LID_H + 0.009)),
            material=gray,
            name=f"stacking_pad_{i}",
        )

    handle_frame = model.part("handle_frame")
    handle_frame.visual(
        Box((0.668, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=metal,
        name="top_bar",
    )
    handle_frame.visual(
        Box((0.34, 0.040, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=rubber,
        name="top_grip",
    )
    for i, x in enumerate(sleeve_xs):
        handle_frame.visual(
            Box((0.024, 0.020, 0.76)),
            origin=Origin(xyz=(x, 0.0, -0.10)),
            material=metal,
            name=f"rail_{i}",
        )

    side_drawer = model.part("side_drawer")
    side_drawer.visual(
        Box((0.024, 0.260, 0.130)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=red,
        name="drawer_face",
    )
    side_drawer.visual(
        Box((0.280, 0.220, 0.012)),
        origin=Origin(xyz=(-0.140, 0.0, -0.056)),
        material=gray,
        name="drawer_tray_bottom",
    )
    for i, y in enumerate((-0.111, 0.111)):
        side_drawer.visual(
            Box((0.280, 0.012, 0.090)),
            origin=Origin(xyz=(-0.140, y, -0.010)),
            material=gray,
            name=f"drawer_tray_side_{i}",
        )
    side_drawer.visual(
        Box((0.014, 0.220, 0.090)),
        origin=Origin(xyz=(-0.273, 0.0, -0.010)),
        material=gray,
        name="drawer_tray_back",
    )
    side_drawer.visual(
        Box((0.010, 0.135, 0.026)),
        origin=Origin(xyz=(0.027, 0.0, 0.006)),
        material=black,
        name="drawer_pull",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            0.044,
            rim=WheelRim(inner_radius=0.035, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.020,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.024, hole_diameter=0.0035),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.058,
            inner_radius=0.050,
            tread=TireTread(style="block", depth=0.0045, count=18, land_ratio=0.56),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "utility_tire",
    )
    for i in range(2):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(wheel_mesh, material=metal, name="rim")

    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "case_to_handle_frame",
        ArticulationType.PRISMATIC,
        parent=case,
        child=handle_frame,
        origin=Origin(xyz=(0.0, -CASE_D / 2.0 - 0.030, HINGE_Z + 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=HANDLE_TRAVEL),
    )
    model.articulation(
        "case_to_side_drawer",
        ArticulationType.PRISMATIC,
        parent=case,
        child=side_drawer,
        origin=Origin(xyz=(CASE_W / 2.0 + 0.018, 0.030, 0.360)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=DRAWER_TRAVEL),
    )
    for i, x in enumerate((-0.405, 0.405)):
        model.articulation(
            f"case_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=case,
            child=f"wheel_{i}",
            origin=Origin(xyz=(x, -0.145, 0.090)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle_frame")
    drawer = object_model.get_part("side_drawer")
    lid_hinge = object_model.get_articulation("case_to_lid")
    handle_slide = object_model.get_articulation("case_to_handle_frame")
    drawer_slide = object_model.get_articulation("case_to_side_drawer")

    # The rear guide sleeves are simplified as solid proxy housings; the rail
    # solids intentionally occupy those sleeves to show a captured telescoping fit.
    for i in range(2):
        ctx.allow_overlap(
            case,
            handle,
            elem_a=f"rear_sleeve_{i}",
            elem_b=f"rail_{i}",
            reason="The telescoping rail is intentionally represented as sliding inside the rear sleeve proxy.",
        )
        ctx.allow_overlap(
            case,
            handle,
            elem_a=f"sleeve_cap_{i}",
            elem_b=f"rail_{i}",
            reason="The rail passes through a simplified solid cap that stands in for the sleeve's molded collar.",
        )
        ctx.expect_within(
            handle,
            case,
            axes="xy",
            inner_elem=f"rail_{i}",
            outer_elem=f"rear_sleeve_{i}",
            margin=0.003,
            name=f"handle rail {i} stays centered in sleeve",
        )
        ctx.expect_overlap(
            handle,
            case,
            axes="z",
            elem_a=f"rail_{i}",
            elem_b=f"rear_sleeve_{i}",
            min_overlap=0.25,
            name=f"collapsed handle rail {i} remains inserted",
        )
        ctx.expect_overlap(
            handle,
            case,
            axes="z",
            elem_a=f"rail_{i}",
            elem_b=f"sleeve_cap_{i}",
            min_overlap=0.03,
            name=f"handle rail {i} passes through collar cap",
        )

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="lower_shell",
        max_gap=0.002,
        max_penetration=0.001,
        name="closed lid seats on lower case rim",
    )

    ctx.expect_overlap(
        drawer,
        case,
        axes="x",
        elem_a="drawer_tray_bottom",
        elem_b="lower_shell",
        min_overlap=0.10,
        name="closed drawer tray is retained inside case",
    )

    rest_handle_aabb = ctx.part_world_aabb(handle)
    rest_drawer_aabb = ctx.part_world_aabb(drawer)
    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({handle_slide: HANDLE_TRAVEL, drawer_slide: DRAWER_TRAVEL, lid_hinge: 1.05}):
        extended_handle_aabb = ctx.part_world_aabb(handle)
        extended_drawer_aabb = ctx.part_world_aabb(drawer)
        open_lid_aabb = ctx.part_world_aabb(lid)
        for i in range(2):
            ctx.expect_overlap(
                handle,
                case,
                axes="z",
                elem_a=f"rail_{i}",
                elem_b=f"rear_sleeve_{i}",
                min_overlap=0.08,
                name=f"extended handle rail {i} keeps retained insertion",
            )
        ctx.expect_overlap(
            drawer,
            case,
            axes="x",
            elem_a="drawer_tray_bottom",
            elem_b="lower_shell",
            min_overlap=0.025,
            name="extended drawer keeps hidden runner engaged",
        )

    ctx.check(
        "handle frame slides upward",
        rest_handle_aabb is not None
        and extended_handle_aabb is not None
        and extended_handle_aabb[1][2] > rest_handle_aabb[1][2] + 0.30,
        details=f"rest={rest_handle_aabb}, extended={extended_handle_aabb}",
    )
    ctx.check(
        "side drawer slides from flank",
        rest_drawer_aabb is not None
        and extended_drawer_aabb is not None
        and extended_drawer_aabb[1][0] > rest_drawer_aabb[1][0] + 0.18,
        details=f"rest={rest_drawer_aabb}, extended={extended_drawer_aabb}",
    )
    ctx.check(
        "top lid lifts on rear hinge",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.18,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "transport wheels are continuous",
        all(
            object_model.get_articulation(f"case_to_wheel_{i}").articulation_type
            == ArticulationType.CONTINUOUS
            for i in range(2)
        ),
    )

    return ctx.report()


object_model = build_object_model()
