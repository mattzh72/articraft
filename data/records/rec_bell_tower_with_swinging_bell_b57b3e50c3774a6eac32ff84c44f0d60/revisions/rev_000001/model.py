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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _straight_timber_mesh(
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    beam_size: float,
):
    profile = rounded_rect_profile(
        beam_size,
        beam_size,
        radius=beam_size * 0.14,
        corner_segments=4,
    )
    geom = sweep_profile_along_spline(
        [start, end],
        profile=profile,
        samples_per_segment=2,
        cap_profile=True,
    )
    return mesh_from_geometry(geom, name)


def _bell_shell_mesh(
    name: str,
    *,
    mouth_radius: float,
    height: float,
    wall_thickness: float,
):
    top_z = -0.105
    bottom_z = top_z - height

    outer = [
        (mouth_radius, bottom_z),
        (mouth_radius * 0.97, bottom_z + 0.06 * height),
        (mouth_radius * 0.92, bottom_z + 0.17 * height),
        (mouth_radius * 0.82, bottom_z + 0.35 * height),
        (mouth_radius * 0.66, bottom_z + 0.58 * height),
        (mouth_radius * 0.48, bottom_z + 0.79 * height),
        (mouth_radius * 0.27, top_z),
    ]
    inner = [
        (max(0.012, mouth_radius - wall_thickness), bottom_z),
        (mouth_radius * 0.88, bottom_z + 0.08 * height),
        (mouth_radius * 0.80, bottom_z + 0.20 * height),
        (mouth_radius * 0.71, bottom_z + 0.38 * height),
        (mouth_radius * 0.56, bottom_z + 0.60 * height),
        (mouth_radius * 0.36, bottom_z + 0.80 * height),
        (max(0.016, mouth_radius * 0.11), top_z + 0.012),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def _add_bell(
    model: ArticulatedObject,
    *,
    part_name: str,
    shell_name: str,
    headstock_name: str,
    saddle_name: str,
    strap_prefix: str,
    shell_mesh_name: str,
    bell_x: float,
    mouth_radius: float,
    bell_height: float,
    shell_wall: float,
    headstock_length: float,
    axle_radius: float,
    swing_axis: tuple[float, float, float],
    bronze_material,
    timber_material,
    iron_material,
):
    bell = model.part(part_name)
    bell.visual(
        Box((0.12, headstock_length, 0.10)),
        material=timber_material,
        name=headstock_name,
    )
    bell.visual(
        Cylinder(radius=axle_radius, length=0.68),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_material,
        name=f"{part_name}_axle",
    )
    bell.visual(
        Box((0.09, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=timber_material,
        name=saddle_name,
    )
    bell.visual(
        Box((0.02, 0.10, 0.13)),
        origin=Origin(xyz=(-0.042, 0.0, -0.090)),
        material=iron_material,
        name=f"{strap_prefix}_left",
    )
    bell.visual(
        Box((0.02, 0.10, 0.13)),
        origin=Origin(xyz=(0.042, 0.0, -0.090)),
        material=iron_material,
        name=f"{strap_prefix}_right",
    )
    bell.visual(
        _bell_shell_mesh(
            shell_mesh_name,
            mouth_radius=mouth_radius,
            height=bell_height,
            wall_thickness=shell_wall,
        ),
        material=bronze_material,
        name=shell_name,
    )
    bell.inertial = Inertial.from_geometry(
        Box((mouth_radius * 2.15, headstock_length, bell_height + 0.11)),
        mass=90.0 if mouth_radius > 0.20 else 60.0,
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
    )

    model.articulation(
        f"frame_to_{part_name}",
        ArticulationType.REVOLUTE,
        parent="frame",
        child=bell,
        origin=Origin(xyz=(bell_x, 0.0, 1.47)),
        axis=swing_axis,
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=1.2,
            lower=-0.42,
            upper=0.42,
        ),
    )
    return bell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_bell_open_belfry")

    timber = model.material("timber", rgba=(0.50, 0.34, 0.20, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.39, 0.25, 0.14, 1.0))
    bronze = model.material("bell_bronze", rgba=(0.70, 0.54, 0.24, 1.0))
    iron = model.material("forged_iron", rgba=(0.20, 0.20, 0.22, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.46, 0.64, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_timber,
        name="floor_deck",
    )
    frame.visual(
        Box((1.66, 0.10, 0.14)),
        origin=Origin(xyz=(0.0, 0.37, 0.07)),
        material=timber,
        name="front_base_sill",
    )
    frame.visual(
        Box((1.66, 0.10, 0.14)),
        origin=Origin(xyz=(0.0, -0.37, 0.07)),
        material=timber,
        name="rear_base_sill",
    )
    frame.visual(
        Box((0.10, 0.74, 0.14)),
        origin=Origin(xyz=(-0.78, 0.0, 0.07)),
        material=timber,
        name="left_base_sill",
    )
    frame.visual(
        Box((0.10, 0.74, 0.14)),
        origin=Origin(xyz=(0.78, 0.0, 0.07)),
        material=timber,
        name="right_base_sill",
    )

    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        for end_name, y_sign in (("front", 1.0), ("rear", -1.0)):
            frame.visual(
                Box((0.10, 0.10, 1.26)),
                origin=Origin(xyz=(0.78 * x_sign, 0.37 * y_sign, 0.77)),
                material=timber,
                name=f"{side_name}_{end_name}_post",
            )

    frame.visual(
        Box((1.66, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.37, 1.35)),
        material=timber,
        name="front_top_beam",
    )
    frame.visual(
        Box((1.66, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, -0.37, 1.35)),
        material=timber,
        name="rear_top_beam",
    )
    frame.visual(
        Box((0.10, 0.74, 0.10)),
        origin=Origin(xyz=(-0.78, 0.0, 1.35)),
        material=timber,
        name="left_plate_beam",
    )
    frame.visual(
        Box((0.10, 0.74, 0.10)),
        origin=Origin(xyz=(0.78, 0.0, 1.35)),
        material=timber,
        name="right_plate_beam",
    )

    frame.visual(
        Box((0.10, 0.74, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.82)),
        material=timber,
        name="ridge_beam",
    )
    frame.visual(
        Box((0.08, 0.08, 0.44)),
        origin=Origin(xyz=(0.0, 0.37, 1.60)),
        material=timber,
        name="front_kingpost",
    )
    frame.visual(
        Box((0.08, 0.08, 0.44)),
        origin=Origin(xyz=(0.0, -0.37, 1.60)),
        material=timber,
        name="rear_kingpost",
    )

    frame.visual(
        _straight_timber_mesh(
            "front_left_rafter_mesh",
            (-0.76, 0.37, 1.38),
            (0.0, 0.37, 1.80),
            0.08,
        ),
        material=timber,
        name="front_left_rafter",
    )
    frame.visual(
        _straight_timber_mesh(
            "front_right_rafter_mesh",
            (0.76, 0.37, 1.38),
            (0.0, 0.37, 1.80),
            0.08,
        ),
        material=timber,
        name="front_right_rafter",
    )
    frame.visual(
        _straight_timber_mesh(
            "rear_left_rafter_mesh",
            (-0.76, -0.37, 1.38),
            (0.0, -0.37, 1.80),
            0.08,
        ),
        material=timber,
        name="rear_left_rafter",
    )
    frame.visual(
        _straight_timber_mesh(
            "rear_right_rafter_mesh",
            (0.76, -0.37, 1.38),
            (0.0, -0.37, 1.80),
            0.08,
        ),
        material=timber,
        name="rear_right_rafter",
    )

    for bell_label, bell_x in (("large", -0.34), ("small", 0.34)):
        for end_name, y_sign in (("front", 1.0), ("rear", -1.0)):
            frame.visual(
                Box((0.14, 0.06, 0.18)),
                origin=Origin(xyz=(bell_x, 0.37 * y_sign, 1.47)),
                material=dark_timber,
                name=f"{bell_label}_{end_name}_bearing_block",
            )

    frame.inertial = Inertial.from_geometry(
        Box((1.66, 0.84, 1.90)),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
    )

    _add_bell(
        model,
        part_name="large_bell",
        shell_name="large_bell_shell",
        headstock_name="large_headstock",
        saddle_name="large_saddle_block",
        strap_prefix="large_hanger",
        shell_mesh_name="large_bell_shell_mesh",
        bell_x=-0.34,
        mouth_radius=0.23,
        bell_height=0.38,
        shell_wall=0.028,
        headstock_length=0.58,
        axle_radius=0.017,
        swing_axis=(0.0, 1.0, 0.0),
        bronze_material=bronze,
        timber_material=dark_timber,
        iron_material=iron,
    )
    _add_bell(
        model,
        part_name="small_bell",
        shell_name="small_bell_shell",
        headstock_name="small_headstock",
        saddle_name="small_saddle_block",
        strap_prefix="small_hanger",
        shell_mesh_name="small_bell_shell_mesh",
        bell_x=0.34,
        mouth_radius=0.18,
        bell_height=0.31,
        shell_wall=0.022,
        headstock_length=0.52,
        axle_radius=0.015,
        swing_axis=(0.0, -1.0, 0.0),
        bronze_material=bronze,
        timber_material=dark_timber,
        iron_material=iron,
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

    frame = object_model.get_part("frame")
    large_bell = object_model.get_part("large_bell")
    small_bell = object_model.get_part("small_bell")
    large_joint = object_model.get_articulation("frame_to_large_bell")
    small_joint = object_model.get_articulation("frame_to_small_bell")
    ridge_beam = frame.get_visual("ridge_beam")
    large_shell = large_bell.get_visual("large_bell_shell")
    small_shell = small_bell.get_visual("small_bell_shell")

    ctx.expect_gap(
        small_bell,
        large_bell,
        axis="x",
        min_gap=0.08,
        positive_elem=small_shell,
        negative_elem=large_shell,
        name="bells are separated side by side at rest",
    )
    ctx.expect_overlap(
        large_bell,
        small_bell,
        axes="y",
        min_overlap=0.26,
        elem_a=large_shell,
        elem_b=small_shell,
        name="both bells hang in the same front-back bay",
    )
    ctx.expect_gap(
        frame,
        large_bell,
        axis="z",
        min_gap=0.28,
        positive_elem=ridge_beam,
        negative_elem=large_shell,
        name="large bell hangs below the ridge beam",
    )
    ctx.expect_gap(
        frame,
        small_bell,
        axis="z",
        min_gap=0.28,
        positive_elem=ridge_beam,
        negative_elem=small_shell,
        name="small bell hangs below the ridge beam",
    )

    def _elem_center_x(part_name: str, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    rest_large_x = _elem_center_x("large_bell", large_shell.name)
    rest_small_x = _elem_center_x("small_bell", small_shell.name)

    large_swing = 0.32
    with ctx.pose({large_joint: large_swing}):
        swung_large_x = _elem_center_x("large_bell", large_shell.name)
        still_small_x = _elem_center_x("small_bell", small_shell.name)
    ctx.check(
        "large bell swings independently toward the left",
        rest_large_x is not None
        and rest_small_x is not None
        and swung_large_x is not None
        and still_small_x is not None
        and swung_large_x < rest_large_x - 0.05
        and abs(still_small_x - rest_small_x) < 0.01,
        details=(
            f"rest_large_x={rest_large_x}, swung_large_x={swung_large_x}, "
            f"rest_small_x={rest_small_x}, still_small_x={still_small_x}"
        ),
    )

    small_swing = 0.32
    with ctx.pose({small_joint: small_swing}):
        swung_small_x = _elem_center_x("small_bell", small_shell.name)
        still_large_x = _elem_center_x("large_bell", large_shell.name)
    ctx.check(
        "small bell swings independently toward the right",
        rest_large_x is not None
        and rest_small_x is not None
        and swung_small_x is not None
        and still_large_x is not None
        and swung_small_x > rest_small_x + 0.04
        and abs(still_large_x - rest_large_x) < 0.01,
        details=(
            f"rest_small_x={rest_small_x}, swung_small_x={swung_small_x}, "
            f"rest_large_x={rest_large_x}, still_large_x={still_large_x}"
        ),
    )

    with ctx.pose({large_joint: 0.30, small_joint: 0.30}):
        ctx.expect_gap(
            small_bell,
            large_bell,
            axis="x",
            min_gap=0.12,
            positive_elem=small_shell,
            negative_elem=large_shell,
            name="bells clear one another when both swing outward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
