from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

LEG_EXTENSION_TRAVEL = 0.14
LEG_LABELS = ("a", "b", "c")
LEG_ANGLES = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


WHITE_PAINT = _make_material("white_paint", (0.93, 0.94, 0.96, 1.0))
BLACK_ANODIZED = _make_material("black_anodized", (0.12, 0.12, 0.13, 1.0))
DARK_PLASTIC = _make_material("dark_plastic", (0.19, 0.20, 0.22, 1.0))
BRUSHED_STEEL = _make_material("brushed_steel", (0.69, 0.71, 0.73, 1.0))
RUBBER = _make_material("rubber_black", (0.08, 0.08, 0.09, 1.0))
OPTICAL_GLASS = _make_material("optical_glass", (0.42, 0.58, 0.68, 0.35))


def _v_add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_scale(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _v_norm(v: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / mag, v[1] / mag, v[2] / mag)


def _save_mesh(name: str, geom):
    return mesh_from_geometry(geom, ASSETS.mesh_path(f"{name}.obj"))


def _tripod_layout() -> list[dict[str, tuple[float, float, float] | float]]:
    layout = []
    for angle in LEG_ANGLES:
        c = math.cos(angle)
        s = math.sin(angle)
        top = (0.055 * c, 0.055 * s, -0.014)
        mid = (0.195 * c, 0.195 * s, -0.42)
        socket = (0.325 * c, 0.325 * s, -0.79)
        foot = (0.47 * c, 0.47 * s, -1.12)
        brace = (0.225 * c, 0.225 * s, -0.535)
        direction = _v_norm(_v_sub(foot, socket))
        layout.append(
            {
                "angle": angle,
                "top": top,
                "mid": mid,
                "socket": socket,
                "brace": brace,
                "foot": foot,
                "direction": direction,
                "unit_xy": (c, s, 0.0),
            }
        )
    return layout


LEG_LAYOUT = _tripod_layout()


def _build_tripod_leg_mesh():
    first = LEG_LAYOUT[0]
    geom = tube_from_spline_points(
        [first["top"], first["mid"], first["socket"]],
        radius=0.016,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    for leg in LEG_LAYOUT[1:]:
        geom.merge(
            tube_from_spline_points(
                [leg["top"], leg["mid"], leg["socket"]],
                radius=0.016,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            )
        )
    for leg in LEG_LAYOUT:
        ux, uy, _ = leg["unit_xy"]
        geom.merge(
            tube_from_spline_points(
                [
                    (0.0, 0.0, -0.548),
                    (0.11 * ux, 0.11 * uy, -0.548),
                    leg["brace"],
                ],
                radius=0.008,
                samples_per_segment=12,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return geom


def _build_accessory_tray_mesh():
    profile = [
        (0.0, 0.175),
        (-0.155, -0.085),
        (0.155, -0.085),
    ]
    geom = ExtrudeGeometry.centered(profile, 0.01, cap=True, closed=True)
    geom.translate(0.0, 0.0, -0.55)
    return geom


def _build_control_cable_mesh():
    return tube_from_spline_points(
        [
            (-0.055, 0.055, 0.125),
            (-0.090, 0.090, 0.105),
            (-0.118, 0.120, 0.080),
            (-0.138, 0.150, 0.045),
        ],
        radius=0.004,
        samples_per_segment=12,
        radial_segments=12,
        cap_ends=True,
    )


def _build_lower_leg_body_mesh(direction: tuple[float, float, float]):
    return tube_from_spline_points(
        [_v_scale(direction, -0.03), _v_scale(direction, 0.34)],
        radius=0.012,
        samples_per_segment=6,
        radial_segments=16,
        cap_ends=True,
    )


def _build_lower_leg_sleeve_mesh(direction: tuple[float, float, float]):
    return tube_from_spline_points(
        [_v_scale(direction, -0.012), _v_scale(direction, 0.065)],
        radius=0.015,
        samples_per_segment=6,
        radial_segments=16,
        cap_ends=True,
    )


def _aabb_bounds(aabb) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if all(hasattr(aabb, name) for name in ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z")):
        return (
            (aabb.min_x, aabb.min_y, aabb.min_z),
            (aabb.max_x, aabb.max_y, aabb.max_z),
        )
    for min_name, max_name in (
        ("min", "max"),
        ("minimum", "maximum"),
        ("mins", "maxs"),
        ("lower", "upper"),
    ):
        if hasattr(aabb, min_name) and hasattr(aabb, max_name):
            mins = tuple(getattr(aabb, min_name))
            maxs = tuple(getattr(aabb, max_name))
            if len(mins) == 3 and len(maxs) == 3:
                return mins, maxs
    if isinstance(aabb, (tuple, list)) and len(aabb) == 2:
        mins = tuple(aabb[0])
        maxs = tuple(aabb[1])
        if len(mins) == 3 and len(maxs) == 3:
            return mins, maxs
    raise TypeError(f"Unsupported AABB representation: {aabb!r}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="astronomical_telescope_tripod", assets=ASSETS)
    model.materials.extend(
        [
            WHITE_PAINT,
            BLACK_ANODIZED,
            DARK_PLASTIC,
            BRUSHED_STEEL,
            RUBBER,
            OPTICAL_GLASS,
        ]
    )

    tripod_leg_mesh = _save_mesh("tripod_upper_legs", _build_tripod_leg_mesh())
    tripod_tray_mesh = _save_mesh("tripod_accessory_tray", _build_accessory_tray_mesh())
    control_cable_mesh = _save_mesh("mount_control_cable", _build_control_cable_mesh())

    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=0.08, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=BLACK_ANODIZED,
    )
    tripod_base.visual(
        Cylinder(radius=0.024, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, -0.29)),
        material=BLACK_ANODIZED,
    )
    tripod_base.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.55)),
        material=DARK_PLASTIC,
    )
    tripod_base.visual(tripod_leg_mesh, material=BLACK_ANODIZED)
    tripod_base.visual(tripod_tray_mesh, material=DARK_PLASTIC)
    for leg in LEG_LAYOUT:
        tripod_base.visual(
            Sphere(radius=0.023),
            origin=Origin(xyz=leg["socket"]),
            material=DARK_PLASTIC,
        )
    tripod_base.inertial = Inertial.from_geometry(
        Box((0.95, 0.95, 1.16)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, -0.58)),
    )

    for label, leg in zip(LEG_LABELS, LEG_LAYOUT):
        child_name = f"leg_extension_{label}"
        body_mesh = _save_mesh(
            f"{child_name}_body",
            _build_lower_leg_body_mesh(leg["direction"]),
        )
        sleeve_mesh = _save_mesh(
            f"{child_name}_sleeve",
            _build_lower_leg_sleeve_mesh(leg["direction"]),
        )
        foot_center = _v_scale(leg["direction"], 0.355)

        lower_leg = model.part(child_name)
        lower_leg.visual(body_mesh, material=BRUSHED_STEEL)
        lower_leg.visual(sleeve_mesh, material=BLACK_ANODIZED)
        lower_leg.visual(
            Sphere(radius=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=DARK_PLASTIC,
        )
        lower_leg.visual(
            Sphere(radius=0.020),
            origin=Origin(xyz=foot_center),
            material=RUBBER,
        )
        lower_leg.inertial = Inertial.from_geometry(
            Box((0.08, 0.08, 0.42)),
            mass=0.8,
            origin=Origin(xyz=_v_scale(leg["direction"], 0.16)),
        )
        model.articulation(
            f"leg_{label}_extension",
            ArticulationType.PRISMATIC,
            parent="tripod_base",
            child=child_name,
            origin=Origin(xyz=leg["socket"]),
            axis=leg["direction"],
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.08,
                lower=0.0,
                upper=LEG_EXTENSION_TRAVEL,
            ),
        )

    mount_head = model.part("mount_head")
    mount_head.visual(
        Cylinder(radius=0.075, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=BLACK_ANODIZED,
    )
    mount_head.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=BRUSHED_STEEL,
    )
    mount_head.visual(
        Box((0.10, 0.085, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=BLACK_ANODIZED,
    )
    mount_head.visual(
        Box((0.07, 0.11, 0.16)),
        origin=Origin(xyz=(-0.008, 0.0, 0.16)),
        material=BLACK_ANODIZED,
    )
    mount_head.visual(
        Box((0.14, 0.10, 0.036)),
        origin=Origin(xyz=(0.015, 0.0, 0.222)),
        material=DARK_PLASTIC,
    )
    mount_head.visual(
        Box((0.09, 0.16, 0.045)),
        origin=Origin(xyz=(0.0, 0.055, 0.242)),
        material=BLACK_ANODIZED,
    )
    mount_head.visual(
        Cylinder(radius=0.021, length=0.065),
        origin=Origin(xyz=(0.0, 0.112, 0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRUSHED_STEEL,
    )
    mount_head.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRUSHED_STEEL,
    )
    mount_head.visual(
        Box((0.05, 0.035, 0.06)),
        origin=Origin(xyz=(0.058, 0.0, 0.102)),
        material=DARK_PLASTIC,
    )
    mount_head.visual(
        Cylinder(radius=0.014, length=0.05),
        origin=Origin(xyz=(-0.055, 0.03, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RUBBER,
    )
    mount_head.visual(
        Cylinder(radius=0.012, length=0.04),
        origin=Origin(xyz=(0.054, -0.028, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RUBBER,
    )
    mount_head.visual(control_cable_mesh, material=RUBBER)
    mount_head.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(-0.138, 0.150, 0.045)),
        material=RUBBER,
    )
    mount_head.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 0.28)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        Cylinder(radius=0.061, length=0.74),
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=WHITE_PAINT,
    )
    optical_tube.visual(
        Cylinder(radius=0.070, length=0.24),
        origin=Origin(xyz=(0.59, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=WHITE_PAINT,
    )
    optical_tube.visual(
        Cylinder(radius=0.067, length=0.022),
        origin=Origin(xyz=(0.47, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK_ANODIZED,
    )
    optical_tube.visual(
        Cylinder(radius=0.056, length=0.006),
        origin=Origin(xyz=(0.706, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=OPTICAL_GLASS,
    )
    optical_tube.visual(
        Box((0.09, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=DARK_PLASTIC,
    )
    optical_tube.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRUSHED_STEEL,
    )
    optical_tube.visual(
        Cylinder(radius=0.070, length=0.022),
        origin=Origin(xyz=(-0.04, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK_ANODIZED,
    )
    optical_tube.visual(
        Cylinder(radius=0.070, length=0.022),
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK_ANODIZED,
    )
    optical_tube.visual(
        Box((0.026, 0.06, 0.045)),
        origin=Origin(xyz=(-0.04, 0.0, -0.03)),
        material=DARK_PLASTIC,
    )
    optical_tube.visual(
        Box((0.026, 0.06, 0.045)),
        origin=Origin(xyz=(0.14, 0.0, -0.03)),
        material=DARK_PLASTIC,
    )
    optical_tube.visual(
        Box((0.28, 0.036, 0.018)),
        origin=Origin(xyz=(0.05, 0.0, -0.041)),
        material=BLACK_ANODIZED,
    )
    optical_tube.visual(
        Cylinder(radius=0.043, length=0.12),
        origin=Origin(xyz=(-0.17, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK_ANODIZED,
    )
    optical_tube.visual(
        Cylinder(radius=0.024, length=0.10),
        origin=Origin(xyz=(-0.28, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BRUSHED_STEEL,
    )
    optical_tube.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-0.35, 0.0, 0.035), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=BLACK_ANODIZED,
    )
    optical_tube.visual(
        Cylinder(radius=0.017, length=0.085),
        origin=Origin(xyz=(-0.405, 0.0, 0.108)),
        material=BLACK_ANODIZED,
    )
    optical_tube.visual(
        Cylinder(radius=0.015, length=0.18),
        origin=Origin(xyz=(0.20, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK_ANODIZED,
    )
    optical_tube.visual(
        Box((0.015, 0.032, 0.06)),
        origin=Origin(xyz=(0.10, 0.0, 0.075)),
        material=BLACK_ANODIZED,
    )
    optical_tube.visual(
        Box((0.015, 0.032, 0.06)),
        origin=Origin(xyz=(0.28, 0.0, 0.075)),
        material=BLACK_ANODIZED,
    )
    optical_tube.visual(
        Cylinder(radius=0.011, length=0.03),
        origin=Origin(xyz=(-0.17, 0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RUBBER,
    )
    optical_tube.visual(
        Cylinder(radius=0.011, length=0.03),
        origin=Origin(xyz=(-0.17, -0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RUBBER,
    )
    optical_tube.inertial = Inertial.from_geometry(
        Box((1.02, 0.22, 0.26)),
        mass=4.6,
        origin=Origin(xyz=(0.18, 0.0, 0.015)),
    )

    model.articulation(
        "mount_azimuth",
        ArticulationType.REVOLUTE,
        parent="tripod_base",
        child="mount_head",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "tube_elevation",
        ArticulationType.REVOLUTE,
        parent="mount_head",
        child="optical_tube",
        origin=Origin(xyz=(0.0, 0.112, 0.24)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.8,
            lower=-0.25,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("mount_head", "tripod_base", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("mount_head", "tripod_base", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_gap("mount_head", "tripod_base", axis="z", max_gap=0.002, max_penetration=0.0)

    ctx.expect_aabb_overlap("optical_tube", "mount_head", axes="xy", min_overlap=0.05)

    ctx.expect_joint_motion_axis(
        "leg_a_extension",
        "leg_extension_a",
        world_axis="z",
        direction="negative",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "leg_b_extension",
        "leg_extension_b",
        world_axis="z",
        direction="negative",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "leg_c_extension",
        "leg_extension_c",
        world_axis="z",
        direction="negative",
        min_delta=0.05,
    )

    tube_rest = ctx.part_world_position("optical_tube")
    if abs(tube_rest[1]) < 0.08:
        raise AssertionError("optical tube should be side-mounted clear of the tripod centerline")

    with ctx.pose(tube_elevation=-0.20):
        low_bounds = _aabb_bounds(ctx.part_world_aabb("optical_tube"))
        low_pos = ctx.part_world_position("optical_tube")
        ctx.expect_aabb_overlap("optical_tube", "mount_head", axes="xy", min_overlap=0.05)

    with ctx.pose(tube_elevation=1.10):
        high_bounds = _aabb_bounds(ctx.part_world_aabb("optical_tube"))
        high_pos = ctx.part_world_position("optical_tube")
        ctx.expect_aabb_overlap("optical_tube", "mount_head", axes="xy", min_overlap=0.02)

    low_mins, low_maxs = low_bounds
    high_mins, high_maxs = high_bounds
    if high_maxs[2] < low_maxs[2] + 0.26:
        raise AssertionError("optical tube should swing upward to a much higher objective height")
    if high_maxs[0] > low_maxs[0] - 0.16:
        raise AssertionError("optical tube should pull its nose inward in x as it elevates")
    if abs(high_pos[1]) < 0.08 or abs(low_pos[1]) < 0.08:
        raise AssertionError(
            "optical tube should stay offset on the side-saddle mount through elevation"
        )

    with ctx.pose(mount_azimuth=1.70, tube_elevation=0.65):
        ctx.expect_origin_distance("mount_head", "tripod_base", axes="xy", max_dist=0.01)
        ctx.expect_aabb_overlap("mount_head", "tripod_base", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_gap("mount_head", "tripod_base", axis="z", max_gap=0.002, max_penetration=0.0)
        ctx.expect_aabb_overlap("optical_tube", "mount_head", axes="xy", min_overlap=0.02)

    rest_positions = {
        label: ctx.part_world_position(f"leg_extension_{label}") for label in LEG_LABELS
    }
    with ctx.pose(
        {
            "leg_a_extension": LEG_EXTENSION_TRAVEL,
            "leg_b_extension": LEG_EXTENSION_TRAVEL,
            "leg_c_extension": LEG_EXTENSION_TRAVEL,
        }
    ):
        for label in LEG_LABELS:
            rest = rest_positions[label]
            extended = ctx.part_world_position(f"leg_extension_{label}")
            if extended[2] > rest[2] - 0.05:
                raise AssertionError(f"leg_extension_{label} should drop noticeably when extended")
            if math.hypot(extended[0], extended[1]) < math.hypot(rest[0], rest[1]) + 0.015:
                raise AssertionError(
                    f"leg_extension_{label} should slide outward as the tripod extends"
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
