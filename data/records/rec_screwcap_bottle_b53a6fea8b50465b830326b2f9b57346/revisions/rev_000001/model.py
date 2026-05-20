from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _helix_points(
    *,
    radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    samples_per_turn: int = 36,
    phase: float = 0.0,
) -> list[tuple[float, float, float]]:
    steps = max(8, int(math.ceil(turns * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for step in range(steps + 1):
        t = step / steps
        angle = phase + turns * math.tau * t
        z = z_start + (z_end - z_start) * t
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return points


def _build_cap_shell() -> object:
    outer_skirt = CylinderGeometry(radius=0.0185, height=0.019, radial_segments=48).translate(
        0.0,
        0.0,
        -0.0095,
    )
    inner_void = CylinderGeometry(radius=0.0168, height=0.021, radial_segments=48).translate(
        0.0,
        0.0,
        -0.0095,
    )
    shell = boolean_difference(outer_skirt, inner_void)
    shell.merge(
        CylinderGeometry(radius=0.0185, height=0.0035, radial_segments=48).translate(
            0.0,
            0.0,
            0.00175,
        )
    )
    shell.merge(
        TorusGeometry(radius=0.0168, tube=0.0012, radial_segments=16, tubular_segments=48).translate(
            0.0,
            0.0,
            -0.0002,
        )
    )
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_pet = model.material("bottle_pet", rgba=(0.76, 0.90, 0.98, 0.42))
    cap_blue = model.material("cap_blue", rgba=(0.18, 0.36, 0.84, 1.0))
    base_rubber = model.material("base_rubber", rgba=(0.16, 0.16, 0.18, 1.0))

    bottle = model.part("bottle_body")
    bottle_shell = LatheGeometry.from_shell_profiles(
        [
            (0.000, 0.000),
            (0.016, 0.002),
            (0.031, 0.008),
            (0.036, 0.018),
            (0.037, 0.070),
            (0.037, 0.175),
            (0.035, 0.198),
            (0.030, 0.220),
            (0.022, 0.239),
            (0.015, 0.249),
            (0.0145, 0.264),
            (0.0135, 0.272),
        ],
        [
            (0.000, 0.006),
            (0.013, 0.008),
            (0.028, 0.014),
            (0.033, 0.022),
            (0.0335, 0.070),
            (0.0335, 0.176),
            (0.0315, 0.198),
            (0.027, 0.217),
            (0.0195, 0.236),
            (0.0105, 0.248),
            (0.0105, 0.268),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    bottle.visual(
        _save_mesh("bottle_shell", bottle_shell),
        material=bottle_pet,
        name="bottle_shell",
    )
    bottle.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=bottle_pet,
        name="neck_support_ring",
    )
    bottle.visual(
        _save_mesh(
            "bottle_lower_foot_ring",
            TorusGeometry(radius=0.031, tube=0.0025, radial_segments=18, tubular_segments=56),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=bottle_pet,
        name="lower_foot_ring",
    )
    neck_thread_mesh = _save_mesh(
        "bottle_neck_thread",
        tube_from_spline_points(
            _helix_points(
                radius=0.0149,
                z_start=0.246,
                z_end=0.263,
                turns=1.15,
                samples_per_turn=42,
            ),
            radius=0.00115,
            samples_per_segment=3,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    bottle.visual(
        neck_thread_mesh,
        material=bottle_pet,
        name="neck_thread_a",
    )
    bottle.visual(
        neck_thread_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi)),
        material=bottle_pet,
        name="neck_thread_b",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.272),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
    )

    cap = model.part("screw_cap")
    cap.visual(
        _save_mesh("screw_cap_shell", _build_cap_shell()),
        material=cap_blue,
        name="cap_shell",
    )
    cap.visual(
        _save_mesh(
            "screw_cap_grip_ring_upper",
            TorusGeometry(radius=0.0181, tube=0.0011, radial_segments=16, tubular_segments=48),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=cap_blue,
        name="grip_ring_upper",
    )
    cap.visual(
        _save_mesh(
            "screw_cap_grip_ring_lower",
            TorusGeometry(radius=0.0181, tube=0.0011, radial_segments=16, tubular_segments=48),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=cap_blue,
        name="grip_ring_lower",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0185, length=0.023),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
    )

    foot = model.part("support_foot")
    foot.visual(
        Cylinder(radius=0.050, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=base_rubber,
        name="pedestal_base",
    )
    foot.visual(
        _save_mesh(
            "support_foot_bead",
            TorusGeometry(radius=0.041, tube=0.0045, radial_segments=18, tubular_segments=48),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0060)),
        material=base_rubber,
        name="retaining_bead",
    )
    foot.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.018),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
    )

    model.articulation(
        "bottle_to_support_foot",
        ArticulationType.FIXED,
        parent=bottle,
        child=foot,
        origin=Origin(),
    )
    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.272)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=12.0,
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

    bottle = object_model.get_part("bottle_body")
    cap = object_model.get_part("screw_cap")
    foot = object_model.get_part("support_foot")
    cap_spin = object_model.get_articulation("cap_spin")

    ctx.expect_gap(
        bottle,
        foot,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        name="support foot sits directly under the bottle body",
    )
    ctx.expect_contact(
        cap,
        bottle,
        name="cap seats against the bottle neck",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="xy",
        min_overlap=0.030,
        name="cap remains centered over the bottle neck footprint",
    )
    ctx.expect_origin_distance(
        cap,
        bottle,
        axes="xy",
        max_dist=1e-6,
        name="cap articulation is coaxial with the bottle",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_spin: 1.7}):
        ctx.expect_overlap(
            cap,
            bottle,
            axes="xy",
            min_overlap=0.030,
            name="rotated cap still covers the bottle neck footprint",
        )
        ctx.expect_origin_distance(
            cap,
            bottle,
            axes="xy",
            max_dist=1e-6,
            name="rotating the cap keeps it coaxial",
        )
        turned_pos = ctx.part_world_position(cap)

    ctx.check(
        "cap spin preserves cap height",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[2] - turned_pos[2]) <= 1e-9,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
