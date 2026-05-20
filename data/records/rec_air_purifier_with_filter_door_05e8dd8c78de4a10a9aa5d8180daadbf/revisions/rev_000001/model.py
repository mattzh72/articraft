from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _arc_points(
    radius: float,
    z: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 18,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        points.append((radius * cos(angle), radius * sin(angle), z))
    return points


def _rounded_cap_mesh(name: str, *, width: float, height: float, depth: float):
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, height, min(width, height) * 0.16, corner_segments=8),
            depth,
            cap=True,
            closed=True,
        ).rotate_y(pi / 2.0),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cup_holder_air_purifier")

    body_dark = model.material("body_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    body_black = model.material("body_black", rgba=(0.08, 0.09, 0.10, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.62, 0.65, 0.68, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.30, 0.58, 0.82, 1.0))
    filter_media = model.material("filter_media", rgba=(0.86, 0.89, 0.88, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.035, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=body_black,
        name="cup_holder_stem",
    )
    body.visual(
        Cylinder(radius=0.039, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=body_dark,
        name="mount_collar",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=body_dark,
        name="neck_post",
    )
    body.visual(
        Cylinder(radius=0.066, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        material=body_dark,
        name="lower_disc_cover",
    )
    body.visual(
        Cylinder(radius=0.066, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
        material=body_dark,
        name="upper_disc_cover",
    )
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _arc_points(0.046, 0.132, 0.96, 2.0 * pi - 0.96, segments=20),
                radius=0.020,
                samples_per_segment=6,
                radial_segments=22,
                cap_ends=True,
            ),
            "purifier_outer_rim",
        ),
        material=body_dark,
        name="outer_rim",
    )
    body.visual(
        Box((0.034, 0.046, 0.004)),
        origin=Origin(xyz=(0.043, 0.0, 0.150)),
        material=body_dark,
        name="opening_upper_rail",
    )
    body.visual(
        Box((0.034, 0.046, 0.004)),
        origin=Origin(xyz=(0.043, 0.0, 0.114)),
        material=body_dark,
        name="opening_lower_rail",
    )
    body.visual(
        Box((0.028, 0.004, 0.036)),
        origin=Origin(xyz=(0.045, 0.024, 0.132)),
        material=body_dark,
        name="opening_left_rail",
    )
    body.visual(
        Box((0.028, 0.004, 0.036)),
        origin=Origin(xyz=(0.045, -0.024, 0.132)),
        material=body_dark,
        name="opening_right_rail",
    )
    body.visual(
        Cylinder(radius=0.044, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.157)),
        material=satin_silver,
        name="top_grille_ring",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.1585)),
        material=accent_blue,
        name="status_disc",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.140, 0.140, 0.160)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    cap = model.part("filter_cap")
    cap.visual(
        _rounded_cap_mesh("purifier_filter_cap_shell", width=0.052, height=0.036, depth=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=body_black,
        name="cap_shell",
    )
    cap.visual(
        Box((0.006, 0.022, 0.010)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=satin_silver,
        name="cap_grip_tab",
    )
    cap.visual(
        Box((0.004, 0.016, 0.004)),
        origin=Origin(xyz=(0.016, 0.0, 0.006)),
        material=satin_silver,
        name="cap_twist_rib_upper",
    )
    cap.visual(
        Box((0.004, 0.016, 0.004)),
        origin=Origin(xyz=(0.016, 0.0, -0.006)),
        material=satin_silver,
        name="cap_twist_rib_lower",
    )
    cap.inertial = Inertial.from_geometry(
        Box((0.022, 0.056, 0.038)),
        mass=0.08,
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
    )

    cartridge = model.part("filter_cartridge")
    cartridge.visual(
        Box((0.068, 0.036, 0.024)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=body_black,
        name="cartridge_frame",
    )
    cartridge.visual(
        Box((0.054, 0.028, 0.018)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=filter_media,
        name="filter_core",
    )
    cartridge.visual(
        Box((0.046, 0.030, 0.004)),
        origin=Origin(xyz=(0.034, 0.0, 0.014)),
        material=body_black,
        name="upper_runner",
    )
    cartridge.visual(
        Box((0.046, 0.030, 0.004)),
        origin=Origin(xyz=(0.034, 0.0, -0.014)),
        material=body_black,
        name="lower_runner",
    )
    for index, local_y in enumerate((-0.010, -0.005, 0.0, 0.005, 0.010)):
        cartridge.visual(
            Box((0.046, 0.0015, 0.017)),
            origin=Origin(xyz=(0.034, local_y, 0.0)),
            material=satin_silver,
            name=f"filter_pleat_{index}",
        )
    cartridge.visual(
        Box((0.006, 0.026, 0.016)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material=satin_silver,
        name="pull_tab",
    )
    cartridge.inertial = Inertial.from_geometry(
        Box((0.074, 0.040, 0.026)),
        mass=0.10,
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_filter_cap",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.066, 0.0, 0.132)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.022,
        ),
    )
    model.articulation(
        "body_to_filter_cartridge",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cartridge,
        origin=Origin(xyz=(-0.008, 0.0, 0.132)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.048,
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
    cap = object_model.get_part("filter_cap")
    cartridge = object_model.get_part("filter_cartridge")
    cap_slide = object_model.get_articulation("body_to_filter_cap")
    cartridge_slide = object_model.get_articulation("body_to_filter_cartridge")

    with ctx.pose({cap_slide: 0.0, cartridge_slide: 0.0}):
        ctx.expect_gap(
            cap,
            body,
            axis="x",
            min_gap=0.0,
            max_gap=0.001,
            name="filter cap seats flush to the purifier housing",
        )
        ctx.expect_overlap(
            cap,
            body,
            axes="yz",
            min_overlap=0.032,
            name="filter cap covers the side opening",
        )
        ctx.expect_gap(
            cap,
            cartridge,
            axis="x",
            min_gap=0.004,
            max_gap=0.010,
            name="closed cap leaves the cartridge just behind the opening",
        )
        ctx.expect_within(
            cartridge,
            body,
            axes="yz",
            margin=0.018,
            name="cartridge is centered within the round housing",
        )

    cap_closed_position = ctx.part_world_position(cap)
    with ctx.pose({cap_slide: 0.022, cartridge_slide: 0.0}):
        cap_open_position = ctx.part_world_position(cap)
        ctx.check(
            "filter cap slides outward on its short twist-lock travel",
            cap_closed_position is not None
            and cap_open_position is not None
            and cap_open_position[0] > cap_closed_position[0] + 0.015,
            details=f"closed={cap_closed_position}, open={cap_open_position}",
        )

    cartridge_closed_position = ctx.part_world_position(cartridge)
    with ctx.pose({cap_slide: 0.022, cartridge_slide: 0.048}):
        cartridge_open_position = ctx.part_world_position(cartridge)
        ctx.expect_overlap(
            cartridge,
            cap,
            axes="yz",
            min_overlap=0.022,
            name="cartridge stays aligned with the cap opening when extended",
        )
        ctx.expect_overlap(
            cartridge,
            body,
            axes="x",
            min_overlap=0.020,
            name="extended cartridge retains insertion in the purifier body",
        )
        ctx.check(
            "filter cartridge slides out through the opened side cap",
            cartridge_closed_position is not None
            and cartridge_open_position is not None
            and cartridge_open_position[0] > cartridge_closed_position[0] + 0.035,
            details=f"closed={cartridge_closed_position}, open={cartridge_open_position}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
