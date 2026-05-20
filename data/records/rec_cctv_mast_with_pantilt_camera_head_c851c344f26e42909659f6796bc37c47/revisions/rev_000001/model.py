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
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_square_lattice_mast(
    part,
    *,
    width: float,
    bottom_z: float,
    top_z: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
    cable_material,
) -> None:
    half = width * 0.5
    corners = [
        (half, half),
        (half, -half),
        (-half, -half),
        (-half, half),
    ]
    levels = [bottom_z + (top_z - bottom_z) * i / panels for i in range(panels + 1)]

    for index, (x, y) in enumerate(corners):
        _add_member(
            part,
            (x, y, bottom_z),
            (x, y, top_z),
            chord_radius,
            material,
            name=f"mast_chord_{index}",
        )

    for level_index, z in enumerate(levels):
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _add_member(
                part,
                (x0, y0, z),
                (x1, y1, z),
                brace_radius,
                material,
                name=f"mast_ring_{level_index}_{i}",
            )

    for panel_index in range(panels):
        z0 = levels[panel_index]
        z1 = levels[panel_index + 1]
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _add_member(
                part,
                (x0, y0, z0),
                (x1, y1, z1),
                brace_radius,
                material,
            )
            _add_member(
                part,
                (x1, y1, z0),
                (x0, y0, z1),
                brace_radius,
                material,
            )

    conduit_x = half + 0.010
    conduit_y = 0.020
    conduit_bottom = bottom_z + 0.12
    conduit_top = top_z - 0.10
    _add_member(
        part,
        (conduit_x, -conduit_y, conduit_bottom),
        (conduit_x, -conduit_y, conduit_top),
        0.0045,
        cable_material,
        name="service_conduit_left",
    )
    _add_member(
        part,
        (conduit_x, conduit_y, conduit_bottom),
        (conduit_x, conduit_y, conduit_top),
        0.0045,
        cable_material,
        name="service_conduit_right",
    )
    rung_count = 14
    for i in range(rung_count + 1):
        z = conduit_bottom + (conduit_top - conduit_bottom) * i / rung_count
        _add_member(
            part,
            (conduit_x, -conduit_y, z),
            (conduit_x, conduit_y, z),
            0.0030,
            cable_material,
        )
    for z in (conduit_bottom, conduit_top):
        _add_member(
            part,
            (half, -conduit_y, z),
            (conduit_x, -conduit_y, z),
            0.0032,
            cable_material,
        )
        _add_member(
            part,
            (half, conduit_y, z),
            (conduit_x, conduit_y, z),
            0.0032,
            cable_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_cctv_mast")

    galvanized = model.material("galvanized", rgba=(0.69, 0.71, 0.73, 1.0))
    dark_paint = model.material("dark_paint", rgba=(0.19, 0.20, 0.22, 1.0))
    camera_white = model.material("camera_white", rgba=(0.85, 0.86, 0.84, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.23, 0.27, 0.65))
    cable_black = model.material("cable_black", rgba=(0.11, 0.11, 0.12, 1.0))

    mast_structure = model.part("mast_structure")
    mast_structure.visual(
        Box((0.46, 0.30, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=galvanized,
        name="base_plate",
    )
    mast_structure.visual(
        Box((0.16, 0.16, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=dark_paint,
        name="mast_socket",
    )
    for sx in (-0.125, 0.125):
        for sy in (-0.080, 0.080):
            mast_structure.visual(
                Cylinder(radius=0.010, length=0.030),
                origin=Origin(xyz=(sx, sy, 0.027)),
                material=dark_paint,
            )

    for sy in (-0.060, 0.060):
        _add_member(
            mast_structure,
            (0.068, sy, 0.024),
            (0.080, sy, 0.084),
            0.008,
            dark_paint,
        )
        _add_member(
            mast_structure,
            (-0.068, sy, 0.024),
            (-0.080, sy, 0.084),
            0.008,
            dark_paint,
        )

    mast_bottom_z = 0.084
    mast_top_z = 1.820
    _add_square_lattice_mast(
        mast_structure,
        width=0.160,
        bottom_z=mast_bottom_z,
        top_z=mast_top_z,
        panels=7,
        chord_radius=0.008,
        brace_radius=0.0048,
        material=galvanized,
        cable_material=cable_black,
    )

    platform_center_z = 1.836
    platform_size = (0.240, 0.200, 0.012)
    deck_half_x = platform_size[0] * 0.5
    deck_half_y = platform_size[1] * 0.5
    mast_half = 0.080
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            _add_member(
                mast_structure,
                (sx * mast_half, sy * mast_half, mast_top_z),
                (sx * deck_half_x, sy * deck_half_y, platform_center_z - platform_size[2] * 0.5),
                0.006,
                galvanized,
            )
    mast_structure.visual(
        Box(platform_size),
        origin=Origin(xyz=(0.0, 0.0, platform_center_z)),
        material=galvanized,
        name="platform_deck",
    )
    mast_structure.visual(
        Box((0.125, 0.125, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, platform_center_z + 0.017)),
        material=dark_paint,
        name="platform_cap",
    )
    mast_structure.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, platform_center_z + 0.043)),
        material=dark_paint,
        name="bearing_pedestal",
    )
    mast_structure.inertial = Inertial.from_geometry(
        Box((0.46, 0.30, 1.90)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.058, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dark_paint,
        name="pan_base",
    )
    pan_head.visual(
        Box((0.090, 0.080, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=dark_paint,
        name="pan_pedestal",
    )
    pan_head.visual(
        Box((0.030, 0.052, 0.050)),
        origin=Origin(xyz=(-0.028, 0.0, 0.112)),
        material=dark_paint,
        name="rear_column",
    )
    pan_head.visual(
        Box((0.092, 0.146, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=dark_paint,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.100, 0.012, 0.110)),
        origin=Origin(xyz=(0.005, 0.067, 0.105)),
        material=dark_paint,
        name="left_yoke_arm",
    )
    pan_head.visual(
        Box((0.100, 0.012, 0.110)),
        origin=Origin(xyz=(0.005, -0.067, 0.105)),
        material=dark_paint,
        name="right_yoke_arm",
    )
    _add_member(
        pan_head,
        (-0.010, 0.032, 0.090),
        (0.012, 0.061, 0.056),
        0.006,
        galvanized,
    )
    _add_member(
        pan_head,
        (-0.010, -0.032, 0.090),
        (0.012, -0.061, 0.056),
        0.006,
        galvanized,
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.13, 0.16, 0.16)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Cylinder(radius=0.011, length=0.122),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="tilt_shaft",
    )
    camera_head.visual(
        Box((0.220, 0.092, 0.102)),
        origin=Origin(xyz=(0.120, 0.0, -0.006)),
        material=camera_white,
        name="camera_shell",
    )
    camera_head.visual(
        Box((0.070, 0.090, 0.060)),
        origin=Origin(xyz=(0.080, 0.0, 0.038)),
        material=camera_white,
        name="rear_housing",
    )
    camera_head.visual(
        Box((0.080, 0.112, 0.078)),
        origin=Origin(xyz=(0.245, 0.0, -0.010)),
        material=camera_white,
        name="lens_shroud",
    )
    camera_head.visual(
        Cylinder(radius=0.029, length=0.080),
        origin=Origin(xyz=(0.257, 0.0, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoked_glass,
        name="lens_barrel",
    )
    camera_head.visual(
        Box((0.235, 0.118, 0.006)),
        origin=Origin(xyz=(0.122, 0.0, 0.047)),
        material=camera_white,
        name="sunshield",
    )
    camera_head.visual(
        Box((0.080, 0.050, 0.026)),
        origin=Origin(xyz=(0.070, 0.0, -0.068)),
        material=dark_paint,
        name="underslung_module",
    )
    camera_head.inertial = Inertial.from_geometry(
        Box((0.320, 0.130, 0.150)),
        mass=4.0,
        origin=Origin(xyz=(0.120, 0.0, -0.005)),
    )

    model.articulation(
        "azimuth_pan",
        ArticulationType.CONTINUOUS,
        parent=mast_structure,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, platform_center_z + 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_head,
        origin=Origin(xyz=(0.045, 0.0, 0.107)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-1.05,
            upper=0.55,
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

    mast_structure = object_model.get_part("mast_structure")
    pan_head = object_model.get_part("pan_head")
    camera_head = object_model.get_part("camera_head")
    azimuth_pan = object_model.get_articulation("azimuth_pan")
    camera_tilt = object_model.get_articulation("camera_tilt")

    ctx.expect_gap(
        pan_head,
        mast_structure,
        axis="z",
        positive_elem="pan_base",
        negative_elem="bearing_pedestal",
        min_gap=0.0,
        max_gap=0.001,
        name="pan base sits on azimuth pedestal",
    )
    ctx.expect_overlap(
        pan_head,
        mast_structure,
        axes="xy",
        elem_a="pan_base",
        elem_b="bearing_pedestal",
        min_overlap=0.10,
        name="pan base remains centered over platform bearing",
    )

    rest_lens = ctx.part_element_world_aabb(camera_head, elem="lens_barrel")
    with ctx.pose({azimuth_pan: math.pi / 2.0}):
        panned_lens = ctx.part_element_world_aabb(camera_head, elem="lens_barrel")
    if rest_lens is None or panned_lens is None:
        ctx.fail(
            "lens AABB available for pan check",
            f"rest={rest_lens}, panned={panned_lens}",
        )
    else:
        rest_center = tuple((lo + hi) * 0.5 for lo, hi in zip(rest_lens[0], rest_lens[1]))
        panned_center = tuple(
            (lo + hi) * 0.5 for lo, hi in zip(panned_lens[0], panned_lens[1])
        )
        ctx.check(
            "camera pans around vertical mast axis",
            rest_center[0] > 0.20
            and abs(rest_center[1]) < 0.03
            and panned_center[1] > 0.20
            and abs(panned_center[0]) < 0.04,
            details=f"rest_center={rest_center}, panned_center={panned_center}",
        )

    rest_lens_tilt = ctx.part_element_world_aabb(camera_head, elem="lens_barrel")
    with ctx.pose({camera_tilt: 0.45}):
        tilted_up_lens = ctx.part_element_world_aabb(camera_head, elem="lens_barrel")
    with ctx.pose({camera_tilt: -0.90}):
        tilted_down_lens = ctx.part_element_world_aabb(camera_head, elem="lens_barrel")
    if (
        rest_lens_tilt is None
        or tilted_up_lens is None
        or tilted_down_lens is None
    ):
        ctx.fail(
            "lens AABB available for tilt check",
            f"rest={rest_lens_tilt}, up={tilted_up_lens}, down={tilted_down_lens}",
        )
    else:
        rest_center = tuple(
            (lo + hi) * 0.5 for lo, hi in zip(rest_lens_tilt[0], rest_lens_tilt[1])
        )
        up_center = tuple(
            (lo + hi) * 0.5 for lo, hi in zip(tilted_up_lens[0], tilted_up_lens[1])
        )
        down_center = tuple(
            (lo + hi) * 0.5 for lo, hi in zip(tilted_down_lens[0], tilted_down_lens[1])
        )
        ctx.check(
            "positive tilt raises the lens",
            up_center[2] > rest_center[2] + 0.04,
            details=f"rest_center={rest_center}, up_center={up_center}",
        )
        ctx.check(
            "negative tilt lowers the lens",
            down_center[2] < rest_center[2] - 0.07,
            details=f"rest_center={rest_center}, down_center={down_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
