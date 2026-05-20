from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lathe_shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 96,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
        name,
    )


def _outer_housing_mesh():
    # Cylindrical lens shell with a large central clearance pocket for the
    # shifting optical block and narrower front/rear light baffles.
    return _lathe_shell_mesh(
        "outer_housing_shell",
        [
            (0.048, -0.066),
            (0.052, -0.061),
            (0.052, -0.047),
            (0.061, -0.041),
            (0.064, -0.025),
            (0.064, 0.025),
            (0.061, 0.041),
            (0.052, 0.047),
            (0.052, 0.061),
            (0.048, 0.066),
        ],
        [
            (0.039, -0.066),
            (0.039, -0.058),
            (0.043, -0.047),
            (0.056, -0.037),
            (0.056, 0.037),
            (0.043, 0.047),
            (0.042, 0.058),
            (0.042, 0.066),
        ],
    )


def _rotation_collar_mesh():
    # Circumferential raised ribs make this read as a grippy rotation collar
    # while keeping the part a single connected hollow ring.
    outer: list[tuple[float, float]] = []
    z_min = -0.026
    step = 0.004
    for i in range(14):
        z = z_min + i * step
        radius = 0.075 if i % 2 == 0 else 0.0715
        outer.append((radius, z))
    outer.append((0.075, 0.026))
    inner = [(0.0664, -0.026), (0.0664, 0.026)]
    return _lathe_shell_mesh("rotation_collar_shell", outer, inner, segments=96)


def _shift_saddle_mesh():
    return _lathe_shell_mesh(
        "shift_saddle_ring",
        [(0.052, -0.010), (0.052, 0.010)],
        [(0.030, -0.010), (0.030, 0.010)],
        segments=96,
    )


def _optical_block_mesh():
    return _lathe_shell_mesh(
        "optical_block_barrel",
        [
            (0.020, -0.047),
            (0.023, -0.041),
            (0.023, 0.024),
            (0.0275, 0.032),
            (0.0275, 0.047),
            (0.024, 0.052),
        ],
        [
            (0.012, -0.047),
            (0.012, -0.041),
            (0.015, 0.024),
            (0.020, 0.036),
            (0.020, 0.052),
        ],
        segments=96,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_shift_architectural_prime_lens")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.04, 0.045, 0.05, 1.0))
    rubber_black = model.material("ribbed_rubber", rgba=(0.008, 0.008, 0.009, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    white_marking = model.material("engraved_white_marking", rgba=(0.92, 0.93, 0.88, 1.0))
    red_dot = model.material("mount_red_dot", rgba=(0.78, 0.02, 0.015, 1.0))
    glass_blue = model.material("coated_glass_blue", rgba=(0.25, 0.48, 0.68, 0.48))
    glass_green = model.material("coated_glass_green", rgba=(0.18, 0.55, 0.45, 0.42))

    outer_housing = model.part("outer_housing")
    outer_housing.visual(
        _outer_housing_mesh(),
        material=satin_black,
        name="barrel_shell",
    )
    outer_housing.visual(
        Sphere(radius=0.0016),
        origin=Origin(xyz=(0.0, 0.0530, 0.055)),
        material=red_dot,
        name="mount_index_dot",
    )

    rotation_collar = model.part("rotation_collar")
    rotation_collar.visual(
        _rotation_collar_mesh(),
        material=rubber_black,
        name="collar_shell",
    )
    rotation_collar.visual(
        Sphere(radius=0.0015),
        origin=Origin(xyz=(0.0, 0.0757, -0.018)),
        material=white_marking,
        name="collar_index_mark",
    )
    for index, (x, y) in enumerate(
        [
            (0.06525, 0.0),
            (-0.06525, 0.0),
            (0.0, 0.06525),
            (0.0, -0.06525),
        ]
    ):
        rotation_collar.visual(
            Sphere(radius=0.00125),
            origin=Origin(xyz=(x, y, 0.0)),
            material=dark_metal,
            name=f"bearing_ball_{index}",
        )

    shift_carriage = model.part("shift_carriage")
    shift_carriage.visual(
        _shift_saddle_mesh(),
        material=dark_metal,
        name="shift_saddle",
    )
    shift_carriage.visual(
        Sphere(radius=0.0022),
        origin=Origin(xyz=(0.0, 0.0538, 0.0)),
        material=dark_metal,
        name="guide_shoe_0",
    )
    shift_carriage.visual(
        Sphere(radius=0.0022),
        origin=Origin(xyz=(0.0, -0.0538, 0.0)),
        material=dark_metal,
        name="guide_shoe_1",
    )

    optical_block = model.part("optical_block")
    optical_block.visual(
        _optical_block_mesh(),
        material=matte_black,
        name="optical_barrel",
    )
    optical_block.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=glass_blue,
        name="front_glass",
    )
    optical_block.visual(
        Cylinder(radius=0.014, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=glass_green,
        name="rear_glass",
    )
    optical_block.visual(
        Cylinder(radius=0.0032, length=0.068),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_trunnion",
    )

    model.articulation(
        "collar_rotation",
        ArticulationType.REVOLUTE,
        parent=outer_housing,
        child=rotation_collar,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "block_shift",
        ArticulationType.PRISMATIC,
        parent=rotation_collar,
        child=shift_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.08,
            lower=-0.012,
            upper=0.012,
        ),
    )
    model.articulation(
        "block_tilt",
        ArticulationType.REVOLUTE,
        parent=shift_carriage,
        child=optical_block,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.6,
            lower=-0.15,
            upper=0.15,
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    outer = object_model.get_part("outer_housing")
    collar = object_model.get_part("rotation_collar")
    carriage = object_model.get_part("shift_carriage")
    optical = object_model.get_part("optical_block")
    collar_rotation = object_model.get_articulation("collar_rotation")
    block_shift = object_model.get_articulation("block_shift")
    block_tilt = object_model.get_articulation("block_tilt")

    ctx.expect_within(
        carriage,
        outer,
        axes="xy",
        inner_elem="shift_saddle",
        outer_elem="barrel_shell",
        margin=0.001,
        name="shift saddle sits inside cylindrical housing envelope",
    )
    ctx.expect_within(
        optical,
        carriage,
        axes="xy",
        inner_elem="optical_barrel",
        outer_elem="shift_saddle",
        margin=0.001,
        name="optical barrel is carried by shift saddle aperture",
    )
    ctx.allow_overlap(
        optical,
        carriage,
        elem_a="tilt_trunnion",
        elem_b="shift_saddle",
        reason="The optical block's transverse trunnion is intentionally captured in the saddle's simplified bearing ring.",
    )
    ctx.expect_overlap(
        optical,
        carriage,
        axes="y",
        elem_a="tilt_trunnion",
        elem_b="shift_saddle",
        min_overlap=0.004,
        name="tilt trunnion remains inserted in saddle bearings",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({block_shift: 0.012}):
        shifted_pos = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic shift moves perpendicular to optical axis",
        rest_pos is not None
        and shifted_pos is not None
        and shifted_pos[0] > rest_pos[0] + 0.010
        and abs(shifted_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, shifted={shifted_pos}",
    )

    with ctx.pose({collar_rotation: math.pi / 2.0, block_shift: 0.012}):
        rotated_shift_pos = ctx.part_world_position(carriage)
    ctx.check(
        "rotation collar redirects the shift axis",
        rest_pos is not None
        and rotated_shift_pos is not None
        and abs(rotated_shift_pos[0] - rest_pos[0]) < 0.002
        and rotated_shift_pos[1] > rest_pos[1] + 0.010,
        details=f"rest={rest_pos}, rotated_shift={rotated_shift_pos}",
    )

    front_rest = ctx.part_element_world_aabb(optical, elem="front_glass")
    with ctx.pose({block_tilt: 0.15}):
        front_tilted = ctx.part_element_world_aabb(optical, elem="front_glass")
    if front_rest is not None and front_tilted is not None:
        rest_center_x = (front_rest[0][0] + front_rest[1][0]) * 0.5
        tilted_center_x = (front_tilted[0][0] + front_tilted[1][0]) * 0.5
    else:
        rest_center_x = tilted_center_x = None
    ctx.check(
        "tilt joint tips the front optical group",
        rest_center_x is not None
        and tilted_center_x is not None
        and tilted_center_x > rest_center_x + 0.005,
        details=f"front_glass_rest_x={rest_center_x}, front_glass_tilt_x={tilted_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
