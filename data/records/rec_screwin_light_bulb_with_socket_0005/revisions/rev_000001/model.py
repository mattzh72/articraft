from __future__ import annotations

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
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _ring_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    radial_segments: int = 64,
):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -0.5 * height), (outer_radius, 0.5 * height)],
        [(inner_radius, -0.5 * height), (inner_radius, 0.5 * height)],
        segments=radial_segments,
        start_cap="flat",
        end_cap="flat",
    ).translate(0.0, 0.0, z_center)


def _helix_points(
    *,
    radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    phase: float = 0.0,
    samples_per_turn: int = 28,
) -> list[tuple[float, float, float]]:
    steps = max(10, int(math.ceil(abs(turns) * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for step in range(steps + 1):
        t = step / steps
        angle = phase + (turns * math.tau * t)
        z = z_start + ((z_end - z_start) * t)
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return points


def _thread_wire(
    *,
    radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    wire_radius: float,
    phase: float = 0.0,
    samples_per_turn: int = 28,
):
    return wire_from_points(
        _helix_points(
            radius=radius,
            z_start=z_start,
            z_end=z_end,
            turns=turns,
            phase=phase,
            samples_per_turn=samples_per_turn,
        ),
        radius=wire_radius,
        radial_segments=14,
        cap_ends=True,
        corner_mode="miter",
    )


def _build_socket_housing():
    outer_body = LatheGeometry(
        [
            (0.0, 0.000),
            (0.0200, 0.000),
            (0.0223, 0.004),
            (0.0236, 0.014),
            (0.0230, 0.024),
            (0.0217, 0.031),
            (0.0192, 0.044),
            (0.0190, 0.048),
            (0.0, 0.048),
        ],
        segments=80,
    )
    bore = CylinderGeometry(radius=0.0154, height=0.022, radial_segments=72).translate(
        0.0,
        0.0,
        0.037,
    )
    return boolean_difference(outer_body, bore)


def _build_glass_envelope():
    outer_profile = [
        (0.0160, 0.000),
        (0.0220, 0.010),
        (0.0330, 0.028),
        (0.0445, 0.060),
        (0.0480, 0.092),
        (0.0455, 0.126),
        (0.0360, 0.154),
        (0.0220, 0.173),
        (0.0060, 0.184),
    ]
    inner_profile = [
        (0.0115, 0.004),
        (0.0185, 0.014),
        (0.0270, 0.030),
        (0.0350, 0.061),
        (0.0385, 0.092),
        (0.0362, 0.124),
        (0.0282, 0.150),
        (0.0160, 0.170),
        (0.0035, 0.180),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_screw_light_bulb_socket", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_nickel = model.material("satin_nickel", rgba=(0.70, 0.72, 0.75, 1.0))
    warm_ceramic = model.material("warm_ceramic", rgba=(0.86, 0.85, 0.81, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.95, 0.94, 0.90, 0.52))
    contact_brass = model.material("contact_brass", rgba=(0.72, 0.60, 0.33, 1.0))
    dark_insulator = model.material("dark_insulator", rgba=(0.10, 0.10, 0.11, 1.0))
    soft_light_core = model.material("soft_light_core", rgba=(0.96, 0.91, 0.78, 0.72))

    socket = model.part("socket")
    socket.visual(
        _save_mesh(_build_socket_housing(), "socket_housing.obj"),
        material=matte_graphite,
        name="socket_housing",
    )
    socket.visual(
        _save_mesh(
            _ring_shell(
                outer_radius=0.0197,
                inner_radius=0.0170,
                height=0.0030,
                z_center=0.0475,
                radial_segments=72,
            ),
            "socket_trim.obj",
        ),
        material=satin_nickel,
        name="socket_trim",
    )
    socket.visual(
        _save_mesh(
            _ring_shell(
                outer_radius=0.01555,
                inner_radius=0.01460,
                height=0.0058,
                z_center=0.0430,
                radial_segments=72,
            ),
            "socket_liner.obj",
        ),
        material=warm_ceramic,
        name="socket_liner",
    )
    socket.visual(
        Cylinder(radius=0.0045, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0250)),
        material=contact_brass,
        name="socket_contact",
    )
    socket.visual(
        _save_mesh(
            TorusGeometry(
                radius=0.0229,
                tube=0.00090,
                radial_segments=14,
                tubular_segments=60,
            ).translate(0.0, 0.0, 0.0175),
            "socket_seam.obj",
        ),
        material=satin_nickel,
        name="socket_seam",
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.050),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        _save_mesh(_build_glass_envelope(), "glass_envelope.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=frosted_glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0122, length=0.0218),
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=satin_nickel,
        name="screw_shell",
    )
    bulb.visual(
        _save_mesh(
            _thread_wire(
                radius=0.01295,
                z_start=0.0068,
                z_end=0.0262,
                turns=3.1,
                wire_radius=0.00095,
                phase=0.05,
                samples_per_turn=30,
            ),
            "thread_ridge.obj",
        ),
        material=satin_nickel,
        name="thread_ridge",
    )
    bulb.visual(
        Cylinder(radius=0.0160, length=0.0048),
        origin=Origin(xyz=(0.0, 0.0, 0.0292)),
        material=satin_nickel,
        name="neck_sleeve",
    )
    bulb.visual(
        Cylinder(radius=0.0060, length=0.0028),
        origin=Origin(xyz=(0.0, 0.0, 0.0047)),
        material=dark_insulator,
        name="insulator_band",
    )
    bulb.visual(
        Cylinder(radius=0.0035, length=0.0036),
        origin=Origin(xyz=(0.0, 0.0, 0.0018)),
        material=contact_brass,
        name="tip_contact",
    )
    bulb.visual(
        Box((0.0032, 0.0012, 0.0060)),
        origin=Origin(xyz=(0.0109, 0.0, 0.0182)),
        material=warm_ceramic,
        name="maker_mark",
    )
    bulb.visual(
        Cylinder(radius=0.0043, length=0.0440),
        origin=Origin(xyz=(0.0, 0.0, 0.0522)),
        material=warm_ceramic,
        name="stem_support",
    )
    bulb.visual(
        Cylinder(radius=0.0100, length=0.0420),
        origin=Origin(xyz=(0.0, 0.0, 0.0934)),
        material=soft_light_core,
        name="light_core",
    )
    bulb.visual(
        _save_mesh(
            TorusGeometry(
                radius=0.0137,
                tube=0.00055,
                radial_segments=12,
                tubular_segments=56,
            ).translate(0.0, 0.0, 0.0275),
            "neck_break.obj",
        ),
        material=satin_nickel,
        name="neck_break",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.215),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=8.0,
            lower=0.0,
            upper=4.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    socket_to_bulb = object_model.get_articulation("socket_to_bulb")

    socket_trim = socket.get_visual("socket_trim")
    socket_liner = socket.get_visual("socket_liner")
    socket_contact = socket.get_visual("socket_contact")
    screw_shell = bulb.get_visual("screw_shell")
    thread_ridge = bulb.get_visual("thread_ridge")
    tip_contact = bulb.get_visual("tip_contact")
    glass_envelope = bulb.get_visual("glass_envelope")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        bulb,
        socket,
        elem_a=tip_contact,
        elem_b=socket_contact,
        contact_tol=0.0006,
        name="bulb_tip_meets_socket_contact",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="xy",
        min_overlap=0.024,
        elem_a=screw_shell,
        elem_b=socket_liner,
        name="screw_base_is_centered_in_socket_collar",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        margin=0.002,
        inner_elem=thread_ridge,
        outer_elem=socket_liner,
        name="thread_ridge_stays_within_socket_receiver",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        min_gap=0.003,
        max_gap=0.006,
        positive_elem=glass_envelope,
        negative_elem=socket_trim,
        name="glass_shoulder_clears_socket_trim_cleanly",
    )

    def _aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    marker_rest_aabb = ctx.part_element_world_aabb(bulb, elem="maker_mark")
    assert marker_rest_aabb is not None
    marker_rest = _aabb_center(marker_rest_aabb)
    assert marker_rest[0] > 0.010
    assert abs(marker_rest[1]) < 0.002

    with ctx.pose({socket_to_bulb: math.pi * 0.5}):
        marker_quarter_aabb = ctx.part_element_world_aabb(bulb, elem="maker_mark")
        assert marker_quarter_aabb is not None
        marker_quarter = _aabb_center(marker_quarter_aabb)
        assert abs(marker_quarter[0]) < 0.002
        assert marker_quarter[1] > 0.010
        ctx.expect_contact(
            bulb,
            socket,
            elem_a=tip_contact,
            elem_b=socket_contact,
            contact_tol=0.0006,
            name="rotated_bulb_tip_stays_on_socket_contact",
        )
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            margin=0.002,
            inner_elem=thread_ridge,
            outer_elem=socket_liner,
            name="rotated_thread_ridge_stays_coaxial_in_socket_receiver",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
