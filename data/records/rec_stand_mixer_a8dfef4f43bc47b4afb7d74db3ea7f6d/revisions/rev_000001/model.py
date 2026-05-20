from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _xy_section(
    center_x: float,
    center_y: float,
    z: float,
    width: float,
    depth: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + px, center_y + py, z)
        for px, py in rounded_rect_profile(width, depth, radius)
    ]


def _yz_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius)
    ]


def _rotated_box(size: tuple[float, float, float], angle: float) -> BoxGeometry:
    return BoxGeometry(size).rotate_z(angle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_mixer")

    enamel = model.material("enamel", rgba=(0.78, 0.14, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.88, 0.89, 0.90, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.85, 0.87, 1.0))

    base = model.part("base")
    base_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.268, 0.188, 0.034),
        0.026,
    )
    base_geom.merge(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.228, 0.154, 0.028),
            0.026,
        ).translate(0.012, 0.0, 0.022)
    )
    base_geom.merge(
        section_loft(
            [
                _xy_section(-0.060, 0.0, 0.042, 0.100, 0.116, 0.028),
                _xy_section(-0.055, 0.0, 0.150, 0.082, 0.100, 0.024),
                _xy_section(-0.052, 0.0, 0.278, 0.066, 0.086, 0.020),
            ]
        )
    )
    base_geom.merge(CylinderGeometry(radius=0.058, height=0.012).translate(0.085, 0.0, 0.050))
    for angle in (math.radians(10.0), math.radians(130.0), math.radians(250.0)):
        lug_x = 0.085 + 0.042 * math.cos(angle)
        lug_y = 0.042 * math.sin(angle)
        base_geom.merge(
            _rotated_box((0.016, 0.010, 0.006), angle).translate(lug_x, lug_y, 0.055)
        )
    base.visual(mesh_from_geometry(base_geom, "mixer_base"), material=enamel, name="body_shell")
    base.visual(
        Cylinder(radius=0.052, length=0.004),
        origin=Origin(xyz=(0.085, 0.0, 0.058)),
        material=dark_trim,
        name="bowl_mount",
    )
    base.visual(
        Box((0.018, 0.018, 0.032)),
        origin=Origin(xyz=(-0.056, -0.029, 0.292)),
        material=dark_trim,
        name="hinge_ear_0",
    )
    base.visual(
        Box((0.018, 0.018, 0.032)),
        origin=Origin(xyz=(-0.056, 0.029, 0.292)),
        material=dark_trim,
        name="hinge_ear_1",
    )

    head = model.part("head")
    head_geom = section_loft(
        [
            _yz_section(0.055, 0.070, 0.082, 0.018, z_center=0.020),
            _yz_section(0.125, 0.122, 0.138, 0.028, z_center=0.004),
            _yz_section(0.195, 0.154, 0.156, 0.034, z_center=-0.008),
            _yz_section(0.258, 0.102, 0.106, 0.020, z_center=-0.008),
        ]
    )
    head.visual(mesh_from_geometry(head_geom, "mixer_head"), material=enamel, name="head_shell")
    head.visual(
        Cylinder(radius=0.030, length=0.022),
        origin=Origin(xyz=(0.141, 0.0, -0.078)),
        material=enamel,
        name="drive_housing",
    )
    head.visual(
        Box((0.060, 0.040, 0.024)),
        origin=Origin(xyz=(0.030, 0.0, 0.006)),
        material=enamel,
        name="hinge_bridge",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.008, 0.050, 0.038)),
        origin=Origin(xyz=(0.255, 0.0, -0.010)),
        material=chrome,
        name="nose_face",
    )

    bowl = model.part("bowl")
    bowl_geom = LatheGeometry.from_shell_profiles(
        [
            (0.030, 0.000),
            (0.050, 0.006),
            (0.078, 0.030),
            (0.090, 0.078),
            (0.086, 0.120),
            (0.090, 0.126),
        ],
        [
            (0.000, 0.004),
            (0.046, 0.010),
            (0.072, 0.032),
            (0.082, 0.078),
            (0.078, 0.120),
        ],
        segments=56,
    )
    bowl_geom.merge(
        tube_from_spline_points(
            [
                (0.010, 0.084, 0.102),
                (0.026, 0.104, 0.108),
                (0.042, 0.118, 0.092),
                (0.048, 0.122, 0.066),
                (0.042, 0.118, 0.042),
                (0.026, 0.104, 0.028),
                (0.010, 0.084, 0.034),
            ],
            radius=0.004,
            samples_per_segment=18,
            radial_segments=18,
        )
    )
    for angle in (0.0, math.radians(120.0), math.radians(240.0)):
        tab_x = 0.036 * math.cos(angle)
        tab_y = 0.036 * math.sin(angle)
        bowl_geom.merge(
            _rotated_box((0.016, 0.010, 0.008), angle).translate(tab_x, tab_y, 0.006)
        )
    bowl.visual(mesh_from_geometry(bowl_geom, "mixer_bowl"), material=steel, name="bowl_shell")

    cover = model.part("cover")
    cover_panel = (
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.046, 0.034, 0.008),
            0.004,
        )
        .rotate_y(math.pi / 2.0)
        .translate(0.0045, 0.0, -0.017)
    )
    cover.visual(mesh_from_geometry(cover_panel, "mixer_cover_panel"), material=chrome, name="cover_panel")
    cover.visual(
        Cylinder(radius=0.0035, length=0.032),
        origin=Origin(xyz=(0.0015, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="cover_hinge",
    )

    whisk = model.part("whisk")
    whisk_geom = CylinderGeometry(radius=0.0054, height=0.030).translate(0.0, 0.0, -0.024)
    whisk_geom.merge(CylinderGeometry(radius=0.0088, height=0.022).translate(0.0, 0.0, -0.049))
    whisk_geom.merge(CylinderGeometry(radius=0.0055, height=0.012).translate(0.0, 0.0, -0.150))
    loop_path = [
        (0.007, 0.0, -0.059),
        (0.018, 0.0, -0.078),
        (0.032, 0.0, -0.104),
        (0.040, 0.0, -0.132),
        (0.000, 0.0, -0.150),
        (-0.040, 0.0, -0.132),
        (-0.032, 0.0, -0.104),
        (-0.018, 0.0, -0.078),
        (-0.007, 0.0, -0.060),
    ]
    for angle in (
        0.0,
        math.radians(30.0),
        math.radians(60.0),
        math.radians(90.0),
        math.radians(120.0),
        math.radians(150.0),
    ):
        whisk_geom.merge(
            tube_from_spline_points(
                loop_path,
                radius=0.0015,
                samples_per_segment=14,
                radial_segments=16,
            ).rotate_z(angle)
        )
    whisk.visual(mesh_from_geometry(whisk_geom, "mixer_whisk"), material=steel, name="whisk_assembly")

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.056, 0.0, 0.308)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "base_to_bowl",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.085, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(40.0),
        ),
    )
    model.articulation(
        "head_to_cover",
        ArticulationType.REVOLUTE,
        parent=head,
        child=cover,
        origin=Origin(xyz=(0.259, 0.0, 0.009)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.141, 0.0, -0.080)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=28.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    bowl = object_model.get_part("bowl")
    cover = object_model.get_part("cover")
    whisk = object_model.get_part("whisk")

    head_hinge = object_model.get_articulation("base_to_head")
    bowl_turn = object_model.get_articulation("base_to_bowl")
    cover_hinge = object_model.get_articulation("head_to_cover")
    whisk_spin = object_model.get_articulation("head_to_whisk")

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="bowl_shell",
        negative_elem="bowl_mount",
        max_gap=0.006,
        max_penetration=1e-5,
        name="bowl seats on the mount ring",
    )
    ctx.expect_within(
        whisk,
        bowl,
        axes="xy",
        inner_elem="whisk_assembly",
        outer_elem="bowl_shell",
        margin=0.004,
        name="closed whisk stays within the bowl footprint",
    )
    ctx.expect_gap(
        cover,
        head,
        axis="x",
        positive_elem="cover_panel",
        negative_elem="nose_face",
        max_gap=0.005,
        max_penetration=0.0,
        name="nose cover closes flush to the front face",
    )

    hinge_limits = head_hinge.motion_limits
    cover_limits = cover_hinge.motion_limits
    bowl_limits = bowl_turn.motion_limits
    ctx.check(
        "whisk uses continuous spin articulation",
        whisk_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {whisk_spin.articulation_type!r}",
    )

    closed_whisk_pos = ctx.part_world_position(whisk)
    closed_cover_box = ctx.part_element_world_aabb(cover, elem="cover_panel")
    closed_bowl_pos = ctx.part_world_position(bowl)

    with ctx.pose({head_hinge: hinge_limits.upper}):
        open_whisk_pos = ctx.part_world_position(whisk)
        ctx.expect_gap(
            whisk,
            bowl,
            axis="z",
            positive_elem="whisk_assembly",
            negative_elem="bowl_shell",
            min_gap=0.020,
            name="tilted head lifts the whisk above the bowl",
        )
        ctx.check(
            "head tilt raises the whisk",
            closed_whisk_pos is not None
            and open_whisk_pos is not None
            and open_whisk_pos[2] > closed_whisk_pos[2] + 0.080,
            details=f"closed={closed_whisk_pos}, open={open_whisk_pos}",
        )

    with ctx.pose({cover_hinge: cover_limits.upper}):
        open_cover_box = ctx.part_element_world_aabb(cover, elem="cover_panel")
        ctx.check(
            "cover flap swings upward",
            closed_cover_box is not None
            and open_cover_box is not None
            and ((open_cover_box[0][2] + open_cover_box[1][2]) * 0.5)
            > ((closed_cover_box[0][2] + closed_cover_box[1][2]) * 0.5) + 0.015,
            details=f"closed={closed_cover_box}, open={open_cover_box}",
        )

    with ctx.pose({bowl_turn: bowl_limits.upper}):
        turned_bowl_pos = ctx.part_world_position(bowl)
        ctx.expect_gap(
            bowl,
            base,
            axis="z",
            positive_elem="bowl_shell",
            negative_elem="bowl_mount",
            max_gap=0.006,
            max_penetration=1e-5,
            name="bowl stays seated while rotated on the bayonet mount",
        )
        ctx.check(
            "bowl rotates in place on the mount",
            closed_bowl_pos is not None
            and turned_bowl_pos is not None
            and abs(turned_bowl_pos[0] - closed_bowl_pos[0]) < 1e-6
            and abs(turned_bowl_pos[1] - closed_bowl_pos[1]) < 1e-6
            and abs(turned_bowl_pos[2] - closed_bowl_pos[2]) < 1e-6,
            details=f"closed={closed_bowl_pos}, turned={turned_bowl_pos}",
        )

    return ctx.report()


object_model = build_object_model()
