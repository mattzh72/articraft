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
    superellipse_profile,
    sweep_profile_along_spline,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_center: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for z, y in rounded_rect_profile(height, width, radius, corner_segments=corner_segments)
    ]


def _oval_ring_mesh(
    *,
    name: str,
    outer_width: float,
    outer_height: float,
    outer_radius: float,
    inner_width: float,
    inner_height: float,
    thickness: float,
):
    outer = rounded_rect_profile(
        outer_height,
        outer_width,
        outer_radius,
        corner_segments=10,
    )
    inner = superellipse_profile(inner_height, inner_width, exponent=2.5, segments=48)
    geom = ExtrudeWithHolesGeometry(outer, [inner], height=thickness, center=True).rotate_y(math.pi / 2.0)
    return _save_mesh(name, geom)


def _oval_plate_mesh(*, name: str, width: float, height: float, thickness: float):
    profile = superellipse_profile(height, width, exponent=2.4, segments=48)
    geom = ExtrudeGeometry(profile, thickness, center=True).rotate_y(math.pi / 2.0)
    return _save_mesh(name, geom)


def _cup_shell_mesh():
    sections = [
        _yz_section(0.046, 0.060, 0.013, -0.036, z_center=-0.046),
        _yz_section(0.056, 0.074, 0.016, -0.028, z_center=-0.046),
        _yz_section(0.066, 0.086, 0.019, -0.016, z_center=-0.046),
        _yz_section(0.072, 0.092, 0.021, -0.004, z_center=-0.046),
    ]
    return _save_mesh("cup_shell", section_loft(sections))


def _headband_outer_mesh():
    points = [
        (-0.104, 0.0, 0.082),
        (-0.096, 0.0, 0.104),
        (-0.074, 0.0, 0.128),
        (-0.040, 0.0, 0.143),
        (0.0, 0.0, 0.148),
        (0.040, 0.0, 0.143),
        (0.074, 0.0, 0.128),
        (0.096, 0.0, 0.104),
        (0.104, 0.0, 0.082),
    ]
    profile = rounded_rect_profile(0.010, 0.032, radius=0.004, corner_segments=8)
    return _save_mesh(
        "headband_outer",
        sweep_profile_along_spline(
            points,
            profile=profile,
            samples_per_segment=14,
            cap_profile=True,
        ),
    )


def _headband_pad_mesh():
    points = [
        (-0.078, 0.0, 0.084),
        (-0.060, 0.0, 0.106),
        (-0.026, 0.0, 0.120),
        (0.0, 0.0, 0.123),
        (0.026, 0.0, 0.120),
        (0.060, 0.0, 0.106),
        (0.078, 0.0, 0.084),
    ]
    profile = rounded_rect_profile(0.014, 0.024, radius=0.006, corner_segments=8)
    return _save_mesh(
        "headband_pad",
        sweep_profile_along_spline(
            points,
            profile=profile,
            samples_per_segment=14,
            cap_profile=True,
        ),
    )


def _build_yoke(part, material_frame, material_trim) -> None:
    part.visual(
        Box((0.008, 0.008, 0.038)),
        origin=Origin(xyz=(0.0, -0.046, 0.019)),
        material=material_frame,
        name="front_arm",
    )
    part.visual(
        Box((0.008, 0.008, 0.038)),
        origin=Origin(xyz=(0.0, 0.046, 0.019)),
        material=material_frame,
        name="rear_arm",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, -0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material_trim,
        name="front_receiver",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material_trim,
        name="rear_receiver",
    )
    part.visual(
        Box((0.010, 0.100, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=material_frame,
        name="bridge",
    )
    part.visual(
        Box((0.008, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=material_frame,
        name="mount_stem",
    )
    part.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=material_trim,
        name="mount_block",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.016, 0.100, 0.074)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
    )


def _build_cup(
    part,
    *,
    shell_mesh,
    carrier_mesh,
    cushion_mesh,
    baffle_mesh,
    shell_material,
    trim_material,
    pad_material,
    cloth_material,
    accent_material,
    mirrored: bool = False,
    with_cable_port: bool = False,
) -> None:
    visual_rpy = (0.0, 0.0, math.pi) if mirrored else (0.0, 0.0, 0.0)

    part.visual(
        Cylinder(radius=0.005, length=0.084),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, visual_rpy[2])),
        material=trim_material,
        name="axle",
    )
    part.visual(
        Box((0.010, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006), rpy=visual_rpy),
        material=trim_material,
        name="axle_block",
    )
    part.visual(
        shell_mesh,
        origin=Origin(rpy=visual_rpy),
        material=shell_material,
        name="shell",
    )
    part.visual(
        carrier_mesh,
        origin=Origin(xyz=(-0.002, 0.0, -0.046), rpy=visual_rpy),
        material=trim_material,
        name="carrier_ring",
    )
    part.visual(
        cushion_mesh,
        origin=Origin(xyz=(0.010, 0.0, -0.046), rpy=visual_rpy),
        material=pad_material,
        name="cushion",
    )
    part.visual(
        baffle_mesh,
        origin=Origin(xyz=(0.001, 0.0, -0.046), rpy=visual_rpy),
        material=cloth_material,
        name="inner_baffle",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.050, 0.086, 0.098)),
        mass=0.18,
        origin=Origin(xyz=(-0.007, 0.0, -0.046)),
    )

    if with_cable_port:
        part.visual(
            Box((0.0015, 0.014, 0.016)),
            origin=Origin(xyz=(-0.02925, 0.022, -0.060)),
            material=trim_material,
            name="door_seat",
        )
        part.visual(
            Cylinder(radius=0.004, length=0.007),
            origin=Origin(
                xyz=(-0.0265, 0.022, -0.061),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=accent_material,
            name="cable_socket",
        )


def _aabb_extents(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (max_x - min_x, max_y - min_y, max_z - min_z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_over_ear_headphones")

    body_black = model.material("body_black", rgba=(0.11, 0.12, 0.13, 1.0))
    soft_black = model.material("soft_black", rgba=(0.07, 0.07, 0.08, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.69, 0.71, 0.74, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    cloth_gray = model.material("cloth_gray", rgba=(0.25, 0.26, 0.28, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.42, 0.44, 0.47, 1.0))

    headband_outer = _headband_outer_mesh()
    headband_pad = _headband_pad_mesh()
    cup_shell = _cup_shell_mesh()
    cup_carrier = _oval_ring_mesh(
        name="cup_carrier_ring",
        outer_width=0.076,
        outer_height=0.096,
        outer_radius=0.020,
        inner_width=0.048,
        inner_height=0.064,
        thickness=0.004,
    )
    cup_cushion = _oval_ring_mesh(
        name="cup_cushion_ring",
        outer_width=0.080,
        outer_height=0.100,
        outer_radius=0.022,
        inner_width=0.052,
        inner_height=0.068,
        thickness=0.018,
    )
    cup_baffle = _oval_plate_mesh(
        name="cup_baffle_plate",
        width=0.050,
        height=0.066,
        thickness=0.002,
    )

    headband = model.part("headband")
    headband.visual(headband_outer, material=body_black, name="outer_band")
    headband.visual(headband_pad, material=soft_black, name="inner_pad")
    headband.visual(
        Box((0.014, 0.020, 0.028)),
        origin=Origin(xyz=(-0.095, 0.0, 0.086)),
        material=dark_metal,
        name="left_mount_block",
    )
    headband.visual(
        Box((0.014, 0.020, 0.028)),
        origin=Origin(xyz=(0.095, 0.0, 0.086)),
        material=dark_metal,
        name="right_mount_block",
    )
    headband.visual(
        Box((0.018, 0.018, 0.016)),
        origin=Origin(xyz=(-0.088, 0.0, 0.101)),
        material=satin_metal,
        name="left_slider_shell",
    )
    headband.visual(
        Box((0.018, 0.018, 0.016)),
        origin=Origin(xyz=(0.088, 0.0, 0.101)),
        material=satin_metal,
        name="right_slider_shell",
    )
    headband.inertial = Inertial.from_geometry(
        Box((0.240, 0.050, 0.160)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    left_yoke = model.part("left_yoke")
    _build_yoke(left_yoke, material_frame=satin_metal, material_trim=dark_metal)

    right_yoke = model.part("right_yoke")
    _build_yoke(right_yoke, material_frame=satin_metal, material_trim=dark_metal)

    left_cup = model.part("left_cup")
    _build_cup(
        left_cup,
        shell_mesh=cup_shell,
        carrier_mesh=cup_carrier,
        cushion_mesh=cup_cushion,
        baffle_mesh=cup_baffle,
        shell_material=body_black,
        trim_material=dark_metal,
        pad_material=soft_black,
        cloth_material=cloth_gray,
        accent_material=accent_gray,
        mirrored=False,
        with_cable_port=True,
    )

    right_cup = model.part("right_cup")
    _build_cup(
        right_cup,
        shell_mesh=cup_shell,
        carrier_mesh=cup_carrier,
        cushion_mesh=cup_cushion,
        baffle_mesh=cup_baffle,
        shell_material=body_black,
        trim_material=dark_metal,
        pad_material=soft_black,
        cloth_material=cloth_gray,
        accent_material=accent_gray,
        mirrored=True,
        with_cable_port=False,
    )

    cable_door = model.part("cable_door")
    cable_door.visual(
        Cylinder(radius=0.0018, length=0.014),
        origin=Origin(xyz=(-0.001, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="door_hinge_barrel",
    )
    cable_door.visual(
        Box((0.002, 0.014, 0.016)),
        origin=Origin(xyz=(-0.001, 0.0, -0.008)),
        material=body_black,
        name="door_panel",
    )
    cable_door.visual(
        Box((0.002, 0.010, 0.004)),
        origin=Origin(xyz=(-0.001, 0.0, -0.002)),
        material=body_black,
        name="door_rib",
    )
    cable_door.inertial = Inertial.from_geometry(
        Box((0.004, 0.016, 0.018)),
        mass=0.01,
        origin=Origin(xyz=(-0.001, 0.0, -0.008)),
    )

    model.articulation(
        "headband_to_left_yoke",
        ArticulationType.FIXED,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.095, 0.0, 0.0)),
    )
    model.articulation(
        "headband_to_right_yoke",
        ArticulationType.FIXED,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
    )
    model.articulation(
        "left_cup_fold",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "right_cup_fold",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "left_cup_cable_door",
        ArticulationType.REVOLUTE,
        parent=left_cup,
        child=cable_door,
        origin=Origin(xyz=(-0.030, 0.022, -0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(100.0),
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

    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")
    cable_door = object_model.get_part("cable_door")

    left_fold = object_model.get_articulation("left_cup_fold")
    right_fold = object_model.get_articulation("right_cup_fold")
    door_hinge = object_model.get_articulation("left_cup_cable_door")

    left_limits = left_fold.motion_limits
    right_limits = right_fold.motion_limits
    door_limits = door_hinge.motion_limits

    ctx.check(
        "left cup fold axis is signed to fold flat upward",
        left_fold.axis == (0.0, -1.0, 0.0)
        and left_limits is not None
        and left_limits.lower == 0.0
        and left_limits.upper is not None
        and left_limits.upper > 1.5,
        details=f"axis={left_fold.axis}, limits={left_limits}",
    )
    ctx.check(
        "right cup fold axis mirrors the left cup",
        right_fold.axis == (0.0, 1.0, 0.0)
        and right_limits is not None
        and right_limits.lower == 0.0
        and right_limits.upper is not None
        and right_limits.upper > 1.5,
        details=f"axis={right_fold.axis}, limits={right_limits}",
    )
    ctx.check(
        "cable door hinge opens outward from the shell",
        door_hinge.axis == (0.0, 1.0, 0.0)
        and door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and door_limits.upper > 1.4,
        details=f"axis={door_hinge.axis}, limits={door_limits}",
    )

    ctx.expect_contact(
        left_cup,
        left_yoke,
        elem_a="axle",
        elem_b="front_receiver",
        name="left cup axle is supported by the left yoke",
    )
    ctx.expect_contact(
        right_cup,
        right_yoke,
        elem_a="axle",
        elem_b="front_receiver",
        name="right cup axle is supported by the right yoke",
    )
    ctx.expect_contact(
        left_yoke,
        headband,
        elem_a="mount_block",
        elem_b="left_mount_block",
        name="left yoke is mounted into the headband block",
    )
    ctx.expect_contact(
        right_yoke,
        headband,
        elem_a="mount_block",
        elem_b="right_mount_block",
        name="right yoke is mounted into the headband block",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            left_cup,
            cable_door,
            axis="x",
            positive_elem="door_seat",
            negative_elem="door_panel",
            max_gap=0.001,
            max_penetration=0.0005,
            name="cable door sits flush over the cable-port seat when closed",
        )

    with ctx.pose({door_hinge: 0.0}):
        door_closed_aabb = ctx.part_element_world_aabb(cable_door, elem="door_panel")
    with ctx.pose({door_hinge: math.radians(75.0)}):
        door_open_aabb = ctx.part_element_world_aabb(cable_door, elem="door_panel")

    ctx.check(
        "cable door swings outward to expose the cable port",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[0][0] < door_closed_aabb[0][0] - 0.006,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )

    left_rest_aabb = ctx.part_world_aabb(left_cup)
    right_rest_aabb = ctx.part_world_aabb(right_cup)
    with ctx.pose({left_fold: math.radians(90.0), right_fold: math.radians(90.0)}):
        left_flat_aabb = ctx.part_world_aabb(left_cup)
        right_flat_aabb = ctx.part_world_aabb(right_cup)

    left_rest_extents = _aabb_extents(left_rest_aabb)
    right_rest_extents = _aabb_extents(right_rest_aabb)
    left_flat_extents = _aabb_extents(left_flat_aabb)
    right_flat_extents = _aabb_extents(right_flat_aabb)

    ctx.check(
        "left cup changes from vertical hang to flat fold",
        left_rest_extents is not None
        and left_flat_extents is not None
        and left_rest_extents[2] > left_rest_extents[0] * 1.4
        and left_flat_extents[0] > left_flat_extents[2] * 1.2,
        details=f"rest={left_rest_extents}, flat={left_flat_extents}",
    )
    ctx.check(
        "right cup changes from vertical hang to flat fold",
        right_rest_extents is not None
        and right_flat_extents is not None
        and right_rest_extents[2] > right_rest_extents[0] * 1.4
        and right_flat_extents[0] > right_flat_extents[2] * 1.2,
        details=f"rest={right_rest_extents}, flat={right_flat_extents}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
