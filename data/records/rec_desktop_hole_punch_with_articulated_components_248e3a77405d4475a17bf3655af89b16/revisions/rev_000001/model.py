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
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    z_center: float,
    corner_radius: float | None = None,
) -> tuple[tuple[float, float, float], ...]:
    radius = corner_radius
    if radius is None:
        radius = min(width, height) * 0.22
    radius = min(radius, width * 0.25, height * 0.45)
    profile = rounded_rect_profile(width, height, radius, corner_segments=8)
    return tuple((x_pos, y_pos, z_center + z_pos) for y_pos, z_pos in profile)


def _build_handle_shell():
    sections = (
        _yz_section(0.012, width=0.088, height=0.016, z_center=0.0005),
        _yz_section(0.038, width=0.098, height=0.024, z_center=0.0065),
        _yz_section(0.076, width=0.104, height=0.028, z_center=0.0055),
        _yz_section(0.112, width=0.090, height=0.018, z_center=0.0010),
    )
    return mesh_from_geometry(section_loft(sections), "handle_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_hole_punch")

    steel = model.material("steel", rgba=(0.72, 0.74, 0.78, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.128, 0.116, 0.003)),
        origin=Origin(xyz=(0.002, 0.0, 0.0225)),
        material=steel,
        name="top_plate",
    )
    base.visual(
        Box((0.122, 0.006, 0.021)),
        origin=Origin(xyz=(0.002, -0.055, 0.0105)),
        material=steel,
        name="left_skirt",
    )
    base.visual(
        Box((0.122, 0.006, 0.021)),
        origin=Origin(xyz=(0.002, 0.055, 0.0105)),
        material=steel,
        name="right_skirt",
    )
    base.visual(
        Box((0.032, 0.108, 0.014)),
        origin=Origin(xyz=(-0.046, 0.0, 0.018)),
        material=steel,
        name="rear_housing",
    )
    base.visual(
        Box((0.014, 0.020, 0.009)),
        origin=Origin(xyz=(-0.047, -0.043, 0.0215)),
        material=steel,
        name="hinge_support_left",
    )
    base.visual(
        Box((0.014, 0.020, 0.009)),
        origin=Origin(xyz=(-0.047, 0.043, 0.0215)),
        material=steel,
        name="hinge_support_right",
    )
    base.visual(
        Box((0.018, 0.022, 0.004)),
        origin=Origin(xyz=(-0.040, 0.0, 0.022)),
        material=steel,
        name="lock_mount",
    )
    base.visual(
        Box((0.014, 0.104, 0.008)),
        origin=Origin(xyz=(0.058, 0.0, 0.018)),
        material=steel,
        name="front_bridge",
    )
    base.visual(
        Box((0.070, 0.005, 0.010)),
        origin=Origin(xyz=(0.010, -0.034, 0.016)),
        material=steel,
        name="guide_left",
    )
    base.visual(
        Box((0.070, 0.005, 0.010)),
        origin=Origin(xyz=(0.010, 0.034, 0.016)),
        material=steel,
        name="guide_right",
    )
    base.visual(
        Box((0.022, 0.062, 0.0025)),
        origin=Origin(xyz=(0.010, 0.0, 0.02325)),
        material=graphite,
        name="die_bridge",
    )
    base.visual(
        Cylinder(radius=0.0045, length=0.0025),
        origin=Origin(xyz=(0.012, -0.020, 0.02325)),
        material=graphite,
        name="die_left",
    )
    base.visual(
        Cylinder(radius=0.0045, length=0.0025),
        origin=Origin(xyz=(0.012, 0.020, 0.02325)),
        material=graphite,
        name="die_right",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(
            xyz=(-0.047, -0.043, 0.0315),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="hinge_left",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(
            xyz=(-0.047, 0.043, 0.0315),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="hinge_right",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.128, 0.116, 0.036)),
        mass=1.2,
        origin=Origin(xyz=(0.002, 0.0, 0.018)),
    )

    handle = model.part("handle")
    handle.visual(
        _build_handle_shell(),
        material=steel,
        name="shell",
    )
    handle.visual(
        Cylinder(radius=0.005, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_sleeve",
    )
    handle.visual(
        Box((0.018, 0.016, 0.011)),
        origin=Origin(xyz=(0.008, -0.028, 0.001)),
        material=steel,
        name="left_gusset",
    )
    handle.visual(
        Box((0.018, 0.016, 0.011)),
        origin=Origin(xyz=(0.008, 0.028, 0.001)),
        material=steel,
        name="right_gusset",
    )
    handle.visual(
        Box((0.010, 0.086, 0.004)),
        origin=Origin(xyz=(0.116, 0.0, -0.003)),
        material=steel,
        name="front_lip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.126, 0.104, 0.030)),
        mass=0.45,
        origin=Origin(xyz=(0.062, 0.0, 0.006)),
    )

    lock = model.part("lock")
    lock.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="pivot",
    )
    lock.visual(
        Box((0.006, 0.014, 0.003)),
        origin=Origin(xyz=(-0.001, 0.0, 0.0015)),
        material=dark_plastic,
        name="pad",
    )
    lock.visual(
        Box((0.020, 0.012, 0.002)),
        origin=Origin(xyz=(0.010, 0.0, 0.001)),
        material=dark_plastic,
        name="tab",
    )
    lock.visual(
        Box((0.005, 0.012, 0.007)),
        origin=Origin(xyz=(0.0205, 0.0, 0.0045)),
        material=dark_plastic,
        name="hook",
    )
    lock.inertial = Inertial.from_geometry(
        Box((0.026, 0.018, 0.009)),
        mass=0.03,
        origin=Origin(xyz=(0.010, 0.0, 0.004)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.072, 0.080, 0.0025)),
        origin=Origin(xyz=(0.036, 0.0, 0.00125)),
        material=dark_plastic,
        name="bottom_panel",
    )
    tray.visual(
        Box((0.072, 0.0025, 0.010)),
        origin=Origin(xyz=(0.036, -0.03875, 0.006)),
        material=dark_plastic,
        name="left_wall",
    )
    tray.visual(
        Box((0.072, 0.0025, 0.010)),
        origin=Origin(xyz=(0.036, 0.03875, 0.006)),
        material=dark_plastic,
        name="right_wall",
    )
    tray.visual(
        Box((0.003, 0.080, 0.010)),
        origin=Origin(xyz=(0.0015, 0.0, 0.006)),
        material=dark_plastic,
        name="rear_wall",
    )
    tray.visual(
        Box((0.006, 0.086, 0.011)),
        origin=Origin(xyz=(0.069, 0.0, 0.0055)),
        material=dark_plastic,
        name="front_panel",
    )
    tray.visual(
        Box((0.004, 0.034, 0.004)),
        origin=Origin(xyz=(0.074, 0.0, 0.008)),
        material=dark_plastic,
        name="grip",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.078, 0.086, 0.012)),
        mass=0.08,
        origin=Origin(xyz=(0.038, 0.0, 0.006)),
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.047, 0.0, 0.0315)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "base_to_lock",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lock,
        origin=Origin(xyz=(-0.040, 0.0, 0.0275)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "base_to_tray",
        ArticulationType.PRISMATIC,
        parent=base,
        child=tray,
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.15,
            lower=0.0,
            upper=0.032,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    lock = object_model.get_part("lock")
    tray = object_model.get_part("tray")

    handle_hinge = object_model.get_articulation("base_to_handle")
    lock_hinge = object_model.get_articulation("base_to_lock")
    tray_slide = object_model.get_articulation("base_to_tray")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="front_lip",
        negative_elem="top_plate",
        min_gap=0.002,
        max_gap=0.010,
        name="handle rests just above the punch deck",
    )
    ctx.expect_overlap(
        handle,
        base,
        axes="xy",
        min_overlap=0.080,
        name="handle covers the base footprint",
    )
    ctx.expect_within(
        tray,
        base,
        axes="yz",
        margin=0.004,
        name="tray stays aligned within the underside channel",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="x",
        min_overlap=0.060,
        name="tray remains inserted when closed",
    )

    handle_rest_aabb = ctx.part_element_world_aabb(handle, elem="front_lip")
    with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
        handle_open_aabb = ctx.part_element_world_aabb(handle, elem="front_lip")
        ctx.check(
            "handle front lifts upward",
            handle_rest_aabb is not None
            and handle_open_aabb is not None
            and handle_open_aabb[0][2] > handle_rest_aabb[0][2] + 0.030,
            details=f"rest={handle_rest_aabb}, open={handle_open_aabb}",
        )

    lock_rest_aabb = ctx.part_element_world_aabb(lock, elem="hook")
    with ctx.pose({lock_hinge: lock_hinge.motion_limits.upper}):
        lock_open_aabb = ctx.part_element_world_aabb(lock, elem="hook")
        ctx.check(
            "lock tab flips upward",
            lock_rest_aabb is not None
            and lock_open_aabb is not None
            and lock_open_aabb[0][2] > lock_rest_aabb[0][2] + 0.010,
            details=f"rest={lock_rest_aabb}, open={lock_open_aabb}",
        )

    tray_rest_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        tray_extended_pos = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            base,
            axes="yz",
            margin=0.004,
            name="tray stays guided when extended",
        )
        ctx.expect_overlap(
            tray,
            base,
            axes="x",
            min_overlap=0.025,
            name="tray keeps retained insertion when extended",
        )
    ctx.check(
        "tray slides forward",
        tray_rest_pos is not None
        and tray_extended_pos is not None
        and tray_extended_pos[0] > tray_rest_pos[0] + 0.020,
        details=f"rest={tray_rest_pos}, extended={tray_extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
