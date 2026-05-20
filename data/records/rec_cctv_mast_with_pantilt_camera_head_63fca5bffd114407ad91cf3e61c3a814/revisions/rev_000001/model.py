from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _add_side_diagonal_x(
    part,
    *,
    y: float,
    x0: float,
    x1: float,
    z0: float,
    z1: float,
    thickness: float,
    material: Material,
    name: str,
) -> None:
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    pitch = -math.atan2(dz, dx)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, y, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_side_diagonal_y(
    part,
    *,
    x: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
    thickness: float,
    material: Material,
    name: str,
) -> None:
    dy = y1 - y0
    dz = z1 - z0
    length = math.hypot(dy, dz)
    roll = math.atan2(dz, dy)
    part.visual(
        Box((thickness, length, thickness)),
        origin=Origin(
            xyz=(x, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(roll, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_cctv_mast")

    galvanized = _mat(model, "galvanized_steel", (0.56, 0.58, 0.55, 1.0))
    dark_steel = _mat(model, "dark_bearing_steel", (0.08, 0.085, 0.085, 1.0))
    bolt_metal = _mat(model, "zinc_bolt_heads", (0.72, 0.72, 0.68, 1.0))
    camera_shell = _mat(model, "warm_white_camera_shell", (0.88, 0.86, 0.80, 1.0))
    lens_black = _mat(model, "matte_black_lens", (0.01, 0.011, 0.012, 1.0))
    glass_blue = _mat(model, "dark_blue_glass", (0.02, 0.05, 0.09, 0.82))

    # Root: one welded/static rooftop mast assembly.
    mast = model.part("mast")
    mast.visual(
        Box((0.72, 0.52, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=galvanized,
        name="base_plate",
    )

    for i, (x, y) in enumerate(
        [(-0.27, -0.17), (0.27, -0.17), (-0.27, 0.17), (0.27, 0.17)]
    ):
        mast.visual(
            Cylinder(radius=0.025, length=0.014),
            origin=Origin(xyz=(x, y, 0.057)),
            material=bolt_metal,
            name=f"anchor_bolt_{i}",
        )

    # Four square-section corner tubes define the lattice mast.
    post_half_span = 0.16
    post_size = 0.04
    mast_z0 = 0.05
    mast_z1 = 1.75
    mast_h = mast_z1 - mast_z0
    for i, (x, y) in enumerate(
        [
            (-post_half_span, -post_half_span),
            (post_half_span, -post_half_span),
            (-post_half_span, post_half_span),
            (post_half_span, post_half_span),
        ]
    ):
        mast.visual(
            Box((post_size, post_size, mast_h)),
            origin=Origin(xyz=(x, y, mast_z0 + mast_h * 0.5)),
            material=galvanized,
            name=f"corner_post_{i}",
        )

    rail_size = 0.028
    rail_span = 2.0 * post_half_span + post_size
    for li, z in enumerate([0.30, 0.58, 0.86, 1.14, 1.42, 1.70]):
        mast.visual(
            Box((rail_span, rail_size, rail_size)),
            origin=Origin(xyz=(0.0, -post_half_span, z)),
            material=galvanized,
            name=f"front_rail_{li}",
        )
        mast.visual(
            Box((rail_span, rail_size, rail_size)),
            origin=Origin(xyz=(0.0, post_half_span, z)),
            material=galvanized,
            name=f"rear_rail_{li}",
        )
        mast.visual(
            Box((rail_size, rail_span, rail_size)),
            origin=Origin(xyz=(-post_half_span, 0.0, z)),
            material=galvanized,
            name=f"side_rail_{li}",
        )
        mast.visual(
            Box((rail_size, rail_span, rail_size)),
            origin=Origin(xyz=(post_half_span, 0.0, z)),
            material=galvanized,
            name=f"side_rail_mirror_{li}",
        )

    # X-bracing on all four faces.  The braces deliberately tuck into the posts
    # like welded square tube so the mast reads as one connected lattice.
    brace = 0.022
    levels = [0.18, 0.48, 0.78, 1.08, 1.38, 1.68]
    for bi, (z0, z1) in enumerate(zip(levels[:-1], levels[1:])):
        _add_side_diagonal_x(
            mast,
            y=-post_half_span,
            x0=-post_half_span,
            x1=post_half_span,
            z0=z0,
            z1=z1,
            thickness=brace,
            material=galvanized,
            name=f"front_brace_{bi}",
        )
        _add_side_diagonal_x(
            mast,
            y=-post_half_span,
            x0=post_half_span,
            x1=-post_half_span,
            z0=z0,
            z1=z1,
            thickness=brace,
            material=galvanized,
            name=f"front_cross_brace_{bi}",
        )
        _add_side_diagonal_x(
            mast,
            y=post_half_span,
            x0=-post_half_span,
            x1=post_half_span,
            z0=z0,
            z1=z1,
            thickness=brace,
            material=galvanized,
            name=f"rear_brace_{bi}",
        )
        _add_side_diagonal_x(
            mast,
            y=post_half_span,
            x0=post_half_span,
            x1=-post_half_span,
            z0=z0,
            z1=z1,
            thickness=brace,
            material=galvanized,
            name=f"rear_cross_brace_{bi}",
        )
        _add_side_diagonal_y(
            mast,
            x=-post_half_span,
            y0=-post_half_span,
            y1=post_half_span,
            z0=z0,
            z1=z1,
            thickness=brace,
            material=galvanized,
            name=f"side_brace_{bi}",
        )
        _add_side_diagonal_y(
            mast,
            x=-post_half_span,
            y0=post_half_span,
            y1=-post_half_span,
            z0=z0,
            z1=z1,
            thickness=brace,
            material=galvanized,
            name=f"side_cross_brace_{bi}",
        )
        _add_side_diagonal_y(
            mast,
            x=post_half_span,
            y0=-post_half_span,
            y1=post_half_span,
            z0=z0,
            z1=z1,
            thickness=brace,
            material=galvanized,
            name=f"side_brace_mirror_{bi}",
        )
        _add_side_diagonal_y(
            mast,
            x=post_half_span,
            y0=post_half_span,
            y1=-post_half_span,
            z0=z0,
            z1=z1,
            thickness=brace,
            material=galvanized,
            name=f"side_cross_mirror_{bi}",
        )

    mast.visual(
        Box((0.42, 0.42, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.80)),
        material=galvanized,
        name="platform_plate",
    )
    mast.visual(
        Cylinder(radius=0.135, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.84)),
        material=dark_steel,
        name="fixed_bearing_race",
    )

    # Continuous pan stage: a turntable and a yoke that carries the tilt axis.
    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.135, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="turntable_disk",
    )
    pan_head.visual(
        Cylinder(radius=0.048, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark_steel,
        name="pan_neck",
    )
    pan_head.visual(
        Box((0.30, 0.24, 0.045)),
        origin=Origin(xyz=(0.02, 0.0, 0.1825)),
        material=dark_steel,
        name="yoke_base",
    )
    pan_head.visual(
        Box((0.11, 0.025, 0.220)),
        origin=Origin(xyz=(0.03, 0.105, 0.315)),
        material=dark_steel,
        name="yoke_cheek_pos",
    )
    pan_head.visual(
        Box((0.11, 0.025, 0.220)),
        origin=Origin(xyz=(0.03, -0.105, 0.315)),
        material=dark_steel,
        name="yoke_cheek_neg",
    )
    pan_head.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.03, 0.1225, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_metal,
        name="tilt_boss_pos",
    )
    pan_head.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.03, -0.1225, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_metal,
        name="tilt_boss_neg",
    )

    azimuth = model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.85)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5),
    )
    azimuth.meta["description"] = "Continuous 360 degree pan bearing at the camera platform."

    # Tilt camera: rectangular outdoor CCTV head with rain hood, lens, glass, and
    # a trunnion axle that is captured between the yoke cheeks.
    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.026, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_metal,
        name="tilt_axle",
    )
    camera.visual(
        Box((0.220, 0.140, 0.100)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=camera_shell,
        name="camera_body",
    )
    camera.visual(
        Box((0.275, 0.180, 0.015)),
        origin=Origin(xyz=(0.075, 0.0, 0.0575)),
        material=camera_shell,
        name="rain_hood",
    )
    camera.visual(
        Cylinder(radius=0.035, length=0.080),
        origin=Origin(xyz=(0.225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.045, length=0.035),
        origin=Origin(xyz=(0.2825, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="lens_hood",
    )
    camera.visual(
        Cylinder(radius=0.032, length=0.004),
        origin=Origin(xyz=(0.302, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_blue,
        name="front_glass",
    )

    model.articulation(
        "tilt_trunnion",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.03, 0.0, 0.325)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.65, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    pan_head = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")
    azimuth = object_model.get_articulation("azimuth_bearing")
    tilt = object_model.get_articulation("tilt_trunnion")

    ctx.check(
        "mast is the only fixed root",
        len(object_model.root_parts()) == 1 and object_model.root_parts()[0].name == "mast",
        details=f"roots={[p.name for p in object_model.root_parts()]}",
    )
    ctx.check(
        "azimuth bearing is continuous",
        azimuth.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={azimuth.articulation_type}",
    )
    ctx.check(
        "tilt joint has CCTV pitch limits",
        tilt.motion_limits is not None
        and tilt.motion_limits.lower <= -0.6
        and tilt.motion_limits.upper >= 0.5,
        details=f"limits={tilt.motion_limits}",
    )

    ctx.expect_gap(
        pan_head,
        mast,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="platform_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan turntable sits on platform",
    )
    ctx.expect_contact(
        camera,
        pan_head,
        elem_a="tilt_axle",
        elem_b="yoke_cheek_pos",
        contact_tol=0.001,
        name="tilt axle reaches positive yoke cheek",
    )
    ctx.expect_contact(
        camera,
        pan_head,
        elem_a="tilt_axle",
        elem_b="yoke_cheek_neg",
        contact_tol=0.001,
        name="tilt axle reaches negative yoke cheek",
    )
    ctx.expect_within(
        camera,
        pan_head,
        axes="y",
        inner_elem="camera_body",
        margin=0.0,
        name="camera body fits between yoke sides",
    )

    def _elem_center_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[2] + hi[2]) * 0.5

    def _elem_center_y(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[1] + hi[1]) * 0.5

    rest_glass_z = _elem_center_z(camera, "front_glass")
    with ctx.pose({tilt: 0.45}):
        up_glass_z = _elem_center_z(camera, "front_glass")
    ctx.check(
        "positive tilt raises camera nose",
        rest_glass_z is not None and up_glass_z is not None and up_glass_z > rest_glass_z + 0.08,
        details=f"rest_z={rest_glass_z}, tilted_z={up_glass_z}",
    )

    rest_glass_y = _elem_center_y(camera, "front_glass")
    with ctx.pose({azimuth: math.pi / 2.0}):
        panned_glass_y = _elem_center_y(camera, "front_glass")
    ctx.check(
        "azimuth joint pans camera head",
        rest_glass_y is not None
        and panned_glass_y is not None
        and panned_glass_y > rest_glass_y + 0.20,
        details=f"rest_y={rest_glass_y}, panned_y={panned_glass_y}",
    )

    return ctx.report()


object_model = build_object_model()
