from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


TAU = 2.0 * math.pi


def _lathe_mesh(profile: list[tuple[float, float]], *, segments: int = 96) -> MeshGeometry:
    """Revolve an (r, z) boundary profile around local +Z."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for i in range(segments):
        theta = TAU * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        ring = []
        for r, z in profile:
            ring.append(geom.add_vertex(r * c, r * s, z))
        rings.append(ring)

    n = len(profile)
    for i in range(segments):
        nxt = (i + 1) % segments
        for j in range(n - 1):
            a = rings[i][j]
            b = rings[nxt][j]
            c = rings[nxt][j + 1]
            d = rings[i][j + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return geom


def _blade_profile(
    *,
    root_x: float = 0.17,
    tip_x: float = 0.60,
    root_half: float = 0.055,
    max_half: float = 0.085,
    tip_half: float = 0.073,
) -> list[tuple[float, float]]:
    """Tapered ABS ceiling-fan blade planform with a rounded outboard tip."""
    tip_base_x = tip_x - tip_half

    def half_width(t: float) -> float:
        # Widens after the root, then narrows gently into the rounded tip.
        return (
            (1.0 - t) * root_half
            + t * tip_half
            + math.sin(math.pi * t) * (max_half - 0.5 * (root_half + tip_half))
        )

    samples = 10
    lower: list[tuple[float, float]] = []
    for i in range(samples + 1):
        t = i / samples
        x = root_x + (tip_base_x - root_x) * t
        lower.append((x, -half_width(t)))

    tip_arc: list[tuple[float, float]] = []
    for i in range(1, 12):
        a = -math.pi / 2.0 + math.pi * i / 11.0
        tip_arc.append((tip_base_x + tip_half * math.cos(a), tip_half * math.sin(a)))

    upper: list[tuple[float, float]] = []
    for i in range(samples, -1, -1):
        t = i / samples
        x = root_x + (tip_base_x - root_x) * t
        upper.append((x, half_width(t)))

    # The flat root is hidden under the blade iron clamp.
    return lower + tip_arc + upper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_folding_ceiling_fan")

    powder_white = model.material("powder_white", rgba=(0.86, 0.88, 0.86, 1.0))
    abs_white = model.material("uv_stable_abs", rgba=(0.94, 0.95, 0.92, 1.0))
    warm_gray = model.material("sealed_gray", rgba=(0.55, 0.57, 0.55, 1.0))
    dark_gasket = model.material("black_epdm_gasket", rgba=(0.02, 0.025, 0.025, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.67, 0.69, 0.68, 1.0))

    # Stationary wet-rated ceiling mount and short downrod.
    mount = model.part("mount")
    canopy_profile = [
        (0.001, 0.355),
        (0.040, 0.348),
        (0.105, 0.365),
        (0.130, 0.405),
        (0.125, 0.455),
        (0.105, 0.478),
        (0.001, 0.485),
        (0.001, 0.355),
    ]
    mount.visual(
        mesh_from_geometry(_lathe_mesh(canopy_profile, segments=96), "ceiling_canopy"),
        material=powder_white,
        name="ceiling_canopy",
    )
    mount.visual(
        Cylinder(radius=0.026, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.2575)),
        material=powder_white,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.042, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=powder_white,
        name="lower_coupler",
    )
    mount.visual(
        mesh_from_geometry(TorusGeometry(radius=0.044, tube=0.004, radial_segments=18, tubular_segments=72), "coupler_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=dark_gasket,
        name="coupler_gasket",
    )

    # The rotating sealed motor body: all blades and the housing spin as a child
    # of the downrod on one continuous central axle.
    fan_head = model.part("fan_head")
    housing_profile = [
        (0.001, -0.070),
        (0.055, -0.092),
        (0.125, -0.086),
        (0.158, -0.052),
        (0.170, -0.016),
        (0.170, 0.016),
        (0.158, 0.052),
        (0.125, 0.086),
        (0.055, 0.092),
        (0.001, 0.070),
        (0.001, -0.070),
    ]
    fan_head.visual(
        mesh_from_geometry(_lathe_mesh(housing_profile, segments=128), "sealed_motor_shell"),
        material=powder_white,
        name="sealed_motor_shell",
    )
    fan_head.visual(
        Cylinder(radius=0.038, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.1075)),
        material=powder_white,
        name="top_collar",
    )
    fan_head.visual(
        Cylinder(radius=0.065, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
        material=powder_white,
        name="bottom_cap",
    )
    fan_head.visual(
        mesh_from_geometry(TorusGeometry(radius=0.158, tube=0.004, radial_segments=16, tubular_segments=96), "upper_seal_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_gasket,
        name="upper_seal_ring",
    )
    fan_head.visual(
        mesh_from_geometry(TorusGeometry(radius=0.158, tube=0.004, radial_segments=16, tubular_segments=96), "lower_seal_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=dark_gasket,
        name="lower_seal_ring",
    )

    hinge_radius = 0.224
    blade_mesh = mesh_from_geometry(
        ExtrudeGeometry(_blade_profile(), 0.012, cap=True, center=True),
        "tapered_abs_blade",
    )
    pitch = math.radians(-13.0)
    blade_joints = []

    for i in range(5):
        theta = TAU * i / 5.0
        c = math.cos(theta)
        s = math.sin(theta)

        # Fixed rim pad on the rotating motor housing, stopping just short of
        # the child's pivot pin to avoid a false current-pose collision.
        fan_head.visual(
            Box((0.056, 0.052, 0.034)),
            origin=Origin(
                xyz=(0.183 * c, 0.183 * s, -0.004),
                rpy=(0.0, 0.0, theta),
            ),
            material=stainless,
            name=f"rim_pad_{i}",
        )

        blade = model.part(f"blade_{i}")
        blade.visual(
            Cylinder(radius=0.014, length=0.074),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=stainless,
            name="pivot_pin",
        )
        blade.visual(
            Cylinder(radius=0.023, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, 0.038)),
            material=stainless,
            name="top_washer",
        )
        blade.visual(
            Cylinder(radius=0.023, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, -0.042)),
            material=stainless,
            name="bottom_washer",
        )
        blade.visual(
            Box((0.060, 0.088, 0.018)),
            origin=Origin(xyz=(0.030, 0.0, -0.020)),
            material=stainless,
            name="hinge_clevis",
        )
        blade.visual(
            Box((0.170, 0.014, 0.014)),
            origin=Origin(xyz=(0.100, 0.032, -0.027), rpy=(pitch * 0.45, 0.0, 0.0)),
            material=stainless,
            name="iron_bar_0",
        )
        blade.visual(
            Box((0.170, 0.014, 0.014)),
            origin=Origin(xyz=(0.100, -0.032, -0.027), rpy=(pitch * 0.45, 0.0, 0.0)),
            material=stainless,
            name="iron_bar_1",
        )
        blade.visual(
            Box((0.080, 0.126, 0.018)),
            origin=Origin(xyz=(0.165, 0.0, -0.032), rpy=(pitch, 0.0, 0.0)),
            material=stainless,
            name="root_clamp",
        )
        blade.visual(
            blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.042), rpy=(pitch, 0.0, 0.0)),
            material=abs_white,
            name="abs_blade",
        )
        for j, y in enumerate((-0.036, 0.036)):
            blade.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(0.175, y, -0.020), rpy=(pitch, 0.0, 0.0)),
                material=warm_gray,
                name=f"clamp_screw_{j}",
            )

        joint = model.articulation(
            f"fan_head_to_blade_{i}",
            ArticulationType.REVOLUTE,
            parent=fan_head,
            child=blade,
            origin=Origin(xyz=(hinge_radius * c, hinge_radius * s, 0.0), rpy=(0.0, 0.0, theta)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-1.35, upper=0.0),
        )
        blade_joints.append(joint)

    model.articulation(
        "mount_to_fan_head",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=fan_head,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    fan_head = object_model.get_part("fan_head")
    spin = object_model.get_articulation("mount_to_fan_head")
    blade_0 = object_model.get_part("blade_0")
    fold_0 = object_model.get_articulation("fan_head_to_blade_0")

    for i in range(5):
        ctx.allow_overlap(
            f"blade_{i}",
            fan_head,
            elem_a="pivot_pin",
            elem_b=f"rim_pad_{i}",
            reason="Each folding blade pin is intentionally captured in a compact rim boss proxy.",
        )
        ctx.expect_overlap(
            f"blade_{i}",
            fan_head,
            axes="z",
            elem_a="pivot_pin",
            elem_b=f"rim_pad_{i}",
            min_overlap=0.030,
            name=f"blade_{i} pin spans rim boss height",
        )

    ctx.expect_gap(
        mount,
        fan_head,
        axis="z",
        positive_elem="downrod",
        negative_elem="top_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="downrod seats on rotating top collar",
    )
    ctx.expect_overlap(
        mount,
        fan_head,
        axes="xy",
        elem_a="downrod",
        elem_b="top_collar",
        min_overlap=0.040,
        name="downrod and axle collar are coaxial",
    )

    rest_pos = ctx.part_world_position(blade_0)
    with ctx.pose({spin: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(blade_0)
    ctx.check(
        "central continuous axle spins blade circle",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - 0.224) < 0.003
        and abs(rest_pos[1]) < 0.003
        and abs(spun_pos[0]) < 0.003
        and abs(spun_pos[1] - 0.224) < 0.003,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    unfolded = ctx.part_element_world_aabb(blade_0, elem="abs_blade")
    with ctx.pose({fold_0: -1.20}):
        folded = ctx.part_element_world_aabb(blade_0, elem="abs_blade")
    ctx.check(
        "blade folds inward around rim pin",
        unfolded is not None
        and folded is not None
        and folded[1][0] < unfolded[1][0] - 0.12,
        details=f"unfolded={unfolded}, folded={folded}",
    )

    return ctx.report()


object_model = build_object_model()
