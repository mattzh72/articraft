from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _helix_points(
    radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    samples: int,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(samples + 1):
        t = i / samples
        angle = math.tau * turns * t
        z = z_start + (z_end - z_start) * t
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_screwin_light_bulb_with_socket", assets=ASSETS)

    opal_glass = model.material("opal_glass", rgba=(0.97, 0.95, 0.90, 0.48))
    matte_graphite = model.material("matte_graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    satin_nickel = model.material("satin_nickel", rgba=(0.72, 0.74, 0.77, 1.0))
    matte_ceramic = model.material("matte_ceramic", rgba=(0.88, 0.86, 0.82, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.78, 0.65, 0.36, 1.0))
    black_insulator = model.material("black_insulator", rgba=(0.08, 0.08, 0.09, 1.0))

    socket_housing_geom = LatheGeometry(
        [
            (0.0, 0.000),
            (0.018, 0.000),
            (0.024, 0.003),
            (0.028, 0.010),
            (0.028, 0.021),
            (0.026, 0.027),
            (0.022, 0.030),
            (0.0, 0.030),
        ],
        segments=64,
    )
    socket_housing_mesh = mesh_from_geometry(
        socket_housing_geom,
        ASSETS.mesh_path("socket_housing.obj"),
    )

    collar_sleeve_geom = ExtrudeWithHolesGeometry(
        outer_profile=_circle_profile(0.0215, segments=56),
        hole_profiles=[_circle_profile(0.0168, segments=56)],
        height=0.028,
        cap=True,
        center=True,
        closed=True,
    ).translate(0.0, 0.0, 0.046)
    collar_sleeve_mesh = mesh_from_geometry(
        collar_sleeve_geom,
        ASSETS.mesh_path("socket_collar.obj"),
    )

    bulb_glass_geom = LatheGeometry(
        [
            (0.0, 0.028),
            (0.010, 0.031),
            (0.018, 0.039),
            (0.028, 0.058),
            (0.0315, 0.078),
            (0.029, 0.094),
            (0.018, 0.108),
            (0.007, 0.114),
            (0.0, 0.117),
        ],
        segments=72,
    )
    bulb_glass_mesh = mesh_from_geometry(
        bulb_glass_geom,
        ASSETS.mesh_path("bulb_glass.obj"),
    )

    thread_geom = tube_from_spline_points(
        _helix_points(radius=0.0142, z_start=0.0050, z_end=0.0265, turns=2.15, samples=20),
        radius=0.0012,
        samples_per_segment=5,
        radial_segments=18,
        cap_ends=True,
    )
    thread_mesh = mesh_from_geometry(
        thread_geom,
        ASSETS.mesh_path("bulb_thread.obj"),
    )

    socket = model.part("socket")
    socket.visual(socket_housing_mesh, material=matte_graphite, name="housing")
    socket.visual(
        Cylinder(radius=0.0215, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=satin_nickel,
        name="collar_floor",
    )
    socket.visual(collar_sleeve_mesh, material=satin_nickel, name="receiving_collar")
    socket.visual(
        Cylinder(radius=0.0045, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.03275)),
        material=warm_brass,
        name="contact_button",
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.060),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    bulb = model.part("bulb")
    bulb.visual(bulb_glass_mesh, material=opal_glass, name="glass_envelope")
    bulb.visual(
        Cylinder(radius=0.0150, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=opal_glass,
        name="glass_neck",
    )
    bulb.visual(
        Cylinder(radius=0.0153, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=matte_ceramic,
        name="neck_sleeve",
    )
    bulb.visual(
        Cylinder(radius=0.0152, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
        material=satin_nickel,
        name="retaining_band",
    )
    bulb.visual(
        Cylinder(radius=0.0134, length=0.0254),
        origin=Origin(xyz=(0.0, 0.0, 0.0161)),
        material=satin_nickel,
        name="threaded_shell",
    )
    bulb.visual(thread_mesh, material=satin_nickel, name="thread_helix")
    bulb.visual(
        Cylinder(radius=0.0064, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0028)),
        material=black_insulator,
        name="base_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0042, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, 0.0012)),
        material=warm_brass,
        name="contact_tip",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.117),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.0, 0.0585)),
    )

    model.articulation(
        "bulb_rotation",
        ArticulationType.CONTINUOUS,
        parent="socket",
        child="bulb",
        origin=Origin(xyz=(0.0, 0.0, 0.0335)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.0005,
        overlap_volume_tol=0.0,
    )
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.0005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    # Add narrow allowances here when conservative QC reports acceptable cases.
    # Add prompt-specific expect_* semantic checks below; they are the main regressions.
    ctx.expect_origin_distance("bulb", "socket", axes="xy", max_dist=0.001)
    ctx.expect_aabb_overlap("bulb", "socket", axes="xy", min_overlap=0.040)
    ctx.expect_aabb_overlap("bulb", "socket", axes="z", min_overlap=0.025)
    ctx.expect_aabb_contact("bulb", "socket")
    ctx.expect_aabb_gap(
        "bulb",
        "socket",
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="contact_tip",
        negative_elem="contact_button",
    )
    ctx.expect_aabb_gap(
        "bulb",
        "socket",
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="glass_envelope",
        negative_elem="receiving_collar",
    )

    with ctx.pose(bulb_rotation=math.pi * 0.5):
        ctx.expect_origin_distance("bulb", "socket", axes="xy", max_dist=0.001)
        ctx.expect_aabb_contact("bulb", "socket")
        ctx.expect_aabb_gap(
            "bulb",
            "socket",
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem="contact_tip",
            negative_elem="contact_button",
        )

    with ctx.pose(bulb_rotation=math.pi):
        ctx.expect_aabb_overlap("bulb", "socket", axes="xy", min_overlap=0.040)
        ctx.expect_aabb_gap(
            "bulb",
            "socket",
            axis="z",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="glass_envelope",
            negative_elem="receiving_collar",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
