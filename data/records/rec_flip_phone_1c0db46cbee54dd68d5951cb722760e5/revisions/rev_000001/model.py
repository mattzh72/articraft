from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


OPEN_ANGLE = math.radians(170.0)
LOWER_LENGTH = 0.105
PHONE_WIDTH = 0.052
LOWER_THICKNESS = 0.012
UPPER_THICKNESS = 0.010
HINGE_AXIS_Z = 0.018
BODY_X0 = 0.006
UPPER_Z_MIN = -0.0035


def rounded_slab(
    length: float,
    width: float,
    thickness: float,
    corner_radius: float,
    *,
    x_center: float = 0.0,
    z_min: float = 0.0,
) -> cq.Workplane:
    """Rounded-rectangle slab with a flat bottom in local coordinates."""
    return (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .edges("|Z")
        .fillet(corner_radius)
        .translate((x_center, 0.0, z_min + thickness / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamshell_flip_phone")

    shell_black = model.material("slightly_pearl_black", rgba=(0.015, 0.016, 0.018, 1.0))
    soft_black = model.material("matte_black_rubber", rgba=(0.004, 0.004, 0.005, 1.0))
    dark_glass = model.material("dark_lcd_glass", rgba=(0.02, 0.035, 0.045, 1.0))
    blue_lcd = model.material("lit_blue_lcd", rgba=(0.16, 0.35, 0.55, 1.0))
    key_silver = model.material("satin_silver_keys", rgba=(0.70, 0.72, 0.72, 1.0))
    hinge_metal = model.material("dark_hinge_metal", rgba=(0.09, 0.09, 0.10, 1.0))

    lower_shell_shape = rounded_slab(
        LOWER_LENGTH,
        PHONE_WIDTH,
        LOWER_THICKNESS,
        0.008,
        x_center=BODY_X0 + LOWER_LENGTH / 2.0,
    )
    upper_shell_shape = rounded_slab(
        LOWER_LENGTH,
        PHONE_WIDTH,
        UPPER_THICKNESS,
        0.008,
        x_center=BODY_X0 + LOWER_LENGTH / 2.0,
        z_min=UPPER_Z_MIN,
    )
    key_shape = rounded_slab(0.0082, 0.0105, 0.0016, 0.0018, z_min=0.0)
    soft_key_shape = rounded_slab(0.0070, 0.0170, 0.0015, 0.0020, z_min=0.0)
    nav_shape = rounded_slab(0.0140, 0.0200, 0.0019, 0.0040, z_min=0.0)

    lower_body = model.part("lower_body")
    lower_body.visual(
        mesh_from_cadquery(lower_shell_shape, "lower_shell", tolerance=0.0006),
        material=shell_black,
        name="lower_shell",
    )
    # Raised hinge lugs grow out of the lower case and sit just in front of the hinge barrels.
    for idx, y in enumerate((-0.0185, 0.0185)):
        lower_body.visual(
            Box((0.012, 0.008, 0.010)),
            origin=Origin(xyz=(BODY_X0 + 0.004, y, LOWER_THICKNESS + 0.005)),
            material=hinge_metal,
            name=f"hinge_lug_{idx}",
        )
    # A small chin below the keypad makes the lower body read as a single molded phone half.
    lower_body.visual(
        Box((0.022, 0.036, 0.0008)),
        origin=Origin(xyz=(BODY_X0 + 0.016, 0.0, LOWER_THICKNESS + 0.0004)),
        material=soft_black,
        name="keypad_chin",
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        mesh_from_cadquery(upper_shell_shape, "upper_shell", tolerance=0.0006),
        material=shell_black,
        name="upper_shell",
    )
    # The hinge spine and two separated barrels rotate with the display half.
    upper_body.visual(
        Box((0.010, 0.046, 0.006)),
        origin=Origin(xyz=(BODY_X0 - 0.001, 0.0, 0.0)),
        material=hinge_metal,
        name="hinge_spine",
    )
    for idx, y in enumerate((-0.0185, 0.0185)):
        upper_body.visual(
            Cylinder(radius=0.005, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"hinge_barrel_{idx}",
        )
    # Display details are on the inner face; in the default open pose they face upward.
    upper_body.visual(
        Box((0.068, 0.039, 0.0004)),
        origin=Origin(xyz=(BODY_X0 + 0.056, 0.0, UPPER_Z_MIN - 0.00018)),
        material=dark_glass,
        name="screen_glass",
    )
    upper_body.visual(
        Box((0.055, 0.030, 0.00045)),
        origin=Origin(xyz=(BODY_X0 + 0.056, 0.0, UPPER_Z_MIN - 0.00043)),
        material=blue_lcd,
        name="screen_lcd",
    )
    for idx, y in enumerate((-0.010, -0.005, 0.0, 0.005, 0.010)):
        upper_body.visual(
            Box((0.0012, 0.0032, 0.0005)),
            origin=Origin(xyz=(BODY_X0 + 0.020, y, UPPER_Z_MIN - 0.0002)),
            material=soft_black,
            name=f"speaker_slot_{idx}",
        )

    model.articulation(
        "body_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        # q=0 is the display-open presentation pose; q=-170 deg folds the halves shut.
        origin=Origin(xyz=(0.0, 0.0, HINGE_AXIS_Z), rpy=(0.0, -OPEN_ANGLE, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-OPEN_ANGLE, upper=0.0, effort=2.0, velocity=2.0),
    )

    # Individual rubber-dome controls with short prismatic travel into the keypad body.
    key_mesh = mesh_from_cadquery(key_shape, "number_key", tolerance=0.0004)
    soft_mesh = mesh_from_cadquery(soft_key_shape, "soft_key", tolerance=0.0004)
    nav_mesh = mesh_from_cadquery(nav_shape, "nav_key", tolerance=0.0004)

    key_origins: list[tuple[str, float, float, object]] = []
    for row, x in enumerate((0.052, 0.066, 0.080, 0.094)):
        for col, y in enumerate((-0.014, 0.0, 0.014)):
            key_origins.append((f"key_{row}_{col}", BODY_X0 + x, y, key_mesh))
    for idx, y in enumerate((-0.012, 0.012)):
        key_origins.append((f"soft_key_{idx}", BODY_X0 + 0.038, y, soft_mesh))
    key_origins.append(("nav_key", BODY_X0 + 0.028, 0.0, nav_mesh))

    for name, x, y, mesh in key_origins:
        key = model.part(name)
        key.visual(mesh, material=key_silver, name="cap")
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=lower_body,
            child=key,
            origin=Origin(xyz=(x, y, LOWER_THICKNESS)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.0015, effort=0.6, velocity=0.08),
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

    lower = object_model.get_part("lower_body")
    upper = object_model.get_part("upper_body")
    hinge = object_model.get_articulation("body_hinge")
    nav_key = object_model.get_part("nav_key")

    ctx.check(
        "hinge opens about 170 degrees",
        abs((hinge.motion_limits.upper - hinge.motion_limits.lower) - OPEN_ANGLE) < math.radians(1.0),
        details=f"limits={hinge.motion_limits}",
    )

    ctx.expect_gap(
        nav_key,
        lower,
        axis="z",
        max_gap=0.00005,
        max_penetration=0.0,
        positive_elem="cap",
        negative_elem="lower_shell",
        name="nav key sits on keypad body",
    )
    ctx.expect_overlap(
        nav_key,
        lower,
        axes="xy",
        min_overlap=0.010,
        elem_a="cap",
        elem_b="lower_shell",
        name="nav key is within lower footprint",
    )

    open_upper_aabb = ctx.part_world_aabb(upper)
    lower_aabb = ctx.part_world_aabb(lower)
    ctx.check(
        "default pose presents the phone open",
        open_upper_aabb is not None
        and lower_aabb is not None
        and open_upper_aabb[1][0] < lower_aabb[0][0] + 0.012,
        details=f"upper_aabb={open_upper_aabb}, lower_aabb={lower_aabb}",
    )

    with ctx.pose({hinge: -OPEN_ANGLE}):
        ctx.expect_overlap(
            upper,
            lower,
            axes="xy",
            min_overlap=0.040,
            elem_a="upper_shell",
            elem_b="lower_shell",
            name="closed halves align as matching rounded rectangles",
        )
        ctx.expect_gap(
            upper,
            nav_key,
            axis="z",
            min_gap=0.0002,
            max_gap=0.004,
            positive_elem="screen_glass",
            negative_elem="cap",
            name="closed display clears raised keypad",
        )

    return ctx.report()


object_model = build_object_model()
