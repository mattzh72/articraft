from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


L1 = 0.18
L2 = 0.42
L3 = 0.13
PIVOT_AXIS = (0.0, -1.0, 0.0)


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Cylinder helper: SDK cylinders start along local Z; rotate them onto Y."""
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hatch_support_arm")

    model.material("galvanized_steel", rgba=(0.62, 0.64, 0.61, 1.0))
    model.material("dark_bore", rgba=(0.055, 0.057, 0.055, 1.0))
    model.material("zinc_pin", rgba=(0.80, 0.78, 0.70, 1.0))
    model.material("shadow_gap", rgba=(0.03, 0.032, 0.033, 1.0))

    base_lug = model.part("base_lug")
    base_lug.visual(
        Box((0.160, 0.076, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, -0.080)),
        material="galvanized_steel",
        name="base_plate",
    )
    for x in (-0.055, 0.055):
        for y in (-0.025, 0.025):
            base_lug.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, y, -0.072)),
                material="zinc_pin",
                name=f"plate_bolt_{x:+.3f}_{y:+.3f}",
            )
    for y in (-0.020, 0.020):
        base_lug.visual(
            Box((0.072, 0.008, 0.072)),
            origin=Origin(xyz=(0.000, y, -0.040)),
            material="galvanized_steel",
            name=f"lug_side_{y:+.3f}",
        )
        base_lug.visual(
            Cylinder(radius=0.030, length=0.008),
            origin=_y_cylinder_origin(0.000, y, 0.000),
            material="galvanized_steel",
            name=f"lug_round_{y:+.3f}",
        )
        base_lug.visual(
            Cylinder(radius=0.018, length=0.003),
            origin=_y_cylinder_origin(0.000, y * 1.26, 0.000),
            material="zinc_pin",
            name=f"base_pin_head_{y:+.3f}",
        )
    base_lug.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=_y_cylinder_origin(0.000, 0.000, 0.000),
        material="zinc_pin",
        name="base_pivot_pin",
    )

    first_link = model.part("first_link")
    first_link.visual(
        Box((L1 - 0.048, 0.012, 0.023)),
        origin=Origin(xyz=(L1 / 2.0, 0.000, 0.000)),
        material="galvanized_steel",
        name="first_web",
    )
    first_link.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=_y_cylinder_origin(0.000, 0.000, 0.000),
        material="galvanized_steel",
        name="first_eye_root",
    )
    first_link.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=_y_cylinder_origin(L1, 0.000, 0.000),
        material="galvanized_steel",
        name="first_eye_tip",
    )

    center_link = model.part("center_link")
    for y in (-0.020, 0.020):
        center_link.visual(
            Box((L2, 0.008, 0.021)),
            origin=Origin(xyz=(L2 / 2.0, y, 0.000)),
            material="galvanized_steel",
            name=f"center_side_{y:+.3f}",
        )
        for x in (0.000, L2):
            center_link.visual(
                Cylinder(radius=0.024, length=0.008),
                origin=_y_cylinder_origin(x, y, 0.000),
                material="galvanized_steel",
                name=f"center_eye_{x:.2f}_{y:+.3f}",
            )
            center_link.visual(
                Cylinder(radius=0.016, length=0.003),
                origin=_y_cylinder_origin(x, y * 1.23, 0.000),
                material="zinc_pin",
                name=f"center_pin_head_{x:.2f}_{y:+.3f}",
            )
    for x in (0.145, 0.275):
        center_link.visual(
            Box((0.032, 0.048, 0.010)),
            origin=Origin(xyz=(x, 0.000, 0.012)),
            material="zinc_pin",
            name=f"center_spacer_{x:.3f}",
        )
    center_link.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=_y_cylinder_origin(0.145, 0.000, 0.012),
        material="dark_bore",
        name="center_spacer_bolt",
    )
    center_link.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=_y_cylinder_origin(0.000, 0.000, 0.000),
        material="zinc_pin",
        name="center_pivot_pin",
    )
    center_link.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=_y_cylinder_origin(L2, 0.000, 0.000),
        material="zinc_pin",
        name="terminal_pivot_pin",
    )

    terminal_link = model.part("terminal_link")
    terminal_link.visual(
        Box((L3 - 0.036, 0.010, 0.018)),
        origin=Origin(xyz=(L3 / 2.0, 0.000, 0.000)),
        material="galvanized_steel",
        name="terminal_web",
    )
    terminal_link.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=_y_cylinder_origin(0.000, 0.000, 0.000),
        material="galvanized_steel",
        name="terminal_eye_root",
    )
    terminal_link.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=_y_cylinder_origin(L3, 0.000, 0.000),
        material="galvanized_steel",
        name="terminal_eye_tip",
    )
    terminal_link.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=_y_cylinder_origin(L3, 0.000, 0.000),
        material="dark_bore",
        name="terminal_bore_tip",
    )
    terminal_link.visual(
        Box((0.070, 0.014, 0.050)),
        origin=Origin(xyz=(L3 + 0.034, 0.000, 0.000)),
        material="galvanized_steel",
        name="tip_mount_pad",
    )
    for z in (-0.015, 0.015):
        terminal_link.visual(
            Cylinder(radius=0.0055, length=0.016),
            origin=_y_cylinder_origin(L3 + 0.040, 0.000, z),
            material="dark_bore",
            name=f"pad_bolt_hole_{z:+.3f}",
        )

    model.articulation(
        "base_to_first",
        ArticulationType.REVOLUTE,
        parent=base_lug,
        child=first_link,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        axis=PIVOT_AXIS,
        motion_limits=MotionLimits(lower=-0.75, upper=1.15, effort=35.0, velocity=1.4),
    )
    model.articulation(
        "first_to_center",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=center_link,
        origin=Origin(xyz=(L1, 0.000, 0.000)),
        axis=PIVOT_AXIS,
        motion_limits=MotionLimits(lower=-1.10, upper=1.10, effort=25.0, velocity=1.6),
    )
    model.articulation(
        "center_to_terminal",
        ArticulationType.REVOLUTE,
        parent=center_link,
        child=terminal_link,
        origin=Origin(xyz=(L2, 0.000, 0.000)),
        axis=PIVOT_AXIS,
        motion_limits=MotionLimits(lower=-0.95, upper=1.25, effort=18.0, velocity=1.8),
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

    joints = [
        object_model.get_articulation("base_to_first"),
        object_model.get_articulation("first_to_center"),
        object_model.get_articulation("center_to_terminal"),
    ]
    ctx.check(
        "three parallel revolute pivots",
        len(joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(round(v, 6) for v in j.axis) == PIVOT_AXIS for j in joints),
        details=f"axes={[getattr(j, 'axis', None) for j in joints]}",
    )

    first = object_model.get_part("first_link")
    center = object_model.get_part("center_link")
    terminal = object_model.get_part("terminal_link")
    ctx.allow_overlap(
        "base_lug",
        first,
        elem_a="base_pivot_pin",
        elem_b="first_eye_root",
        reason="The base pivot pin is intentionally captured through the first link eye.",
    )
    ctx.allow_overlap(
        center,
        first,
        elem_a="center_pivot_pin",
        elem_b="first_eye_tip",
        reason="The first-to-center pivot pin is intentionally captured through the first link eye.",
    )
    ctx.allow_overlap(
        center,
        terminal,
        elem_a="terminal_pivot_pin",
        elem_b="terminal_eye_root",
        reason="The terminal pivot pin is intentionally captured through the terminal link eye.",
    )
    ctx.expect_overlap(
        "base_lug",
        first,
        axes="xz",
        elem_a="base_pivot_pin",
        elem_b="first_eye_root",
        min_overlap=0.010,
        name="base pin passes through the first eye",
    )
    ctx.expect_overlap(
        center,
        first,
        axes="xz",
        elem_a="center_pivot_pin",
        elem_b="first_eye_tip",
        min_overlap=0.010,
        name="center pin passes through the first link tip eye",
    )
    ctx.expect_overlap(
        center,
        terminal,
        axes="xz",
        elem_a="terminal_pivot_pin",
        elem_b="terminal_eye_root",
        min_overlap=0.009,
        name="terminal pin passes through the terminal root eye",
    )
    ctx.expect_origin_distance(
        "base_lug",
        center,
        axes="x",
        min_dist=L1 - 0.002,
        max_dist=L1 + 0.002,
        name="first link carries the center pivot",
    )
    ctx.expect_origin_distance(
        center,
        terminal,
        axes="x",
        min_dist=L2 - 0.002,
        max_dist=L2 + 0.002,
        name="center link carries the terminal pivot",
    )

    rest_terminal = ctx.part_world_position(terminal)
    with ctx.pose({"base_to_first": 0.55}):
        raised_center = ctx.part_world_position(center)
        raised_terminal = ctx.part_world_position(terminal)
    ctx.check(
        "positive base pivot lifts the linkage plane",
        rest_terminal is not None
        and raised_center is not None
        and raised_terminal is not None
        and raised_center[2] > 0.040
        and raised_terminal[2] > rest_terminal[2] + 0.120,
        details=f"rest_terminal={rest_terminal}, raised_center={raised_center}, raised_terminal={raised_terminal}",
    )

    ctx.expect_within(
        first,
        "base_lug",
        axes="y",
        inner_elem="first_eye_root",
        outer_elem="base_plate",
        margin=0.0,
        name="first link fits inside the base lug width",
    )

    return ctx.report()


object_model = build_object_model()
