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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_light_bulb_with_socket", assets=ASSETS)

    frosted_glass = model.material("frosted_glass", rgba=(0.95, 0.96, 0.97, 0.52))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    brushed_nickel = model.material("brushed_nickel", rgba=(0.66, 0.68, 0.71, 1.0))
    matte_ceramic = model.material("matte_ceramic", rgba=(0.18, 0.18, 0.19, 1.0))
    warm_insulator = model.material("warm_insulator", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.38, 0.40, 0.43, 1.0))

    def save_mesh(name: str, geometry: MeshGeometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def helical_thread(
        *,
        radius: float,
        z_start: float,
        z_end: float,
        turns: float,
        tube_radius: float,
        start_angle: float = 0.0,
        samples_per_turn: int = 36,
    ) -> MeshGeometry:
        points: list[tuple[float, float, float]] = []
        sample_count = max(12, int(math.ceil(turns * samples_per_turn)))
        for step in range(sample_count + 1):
            t = step / sample_count
            angle = start_angle + turns * math.tau * t
            z = z_start + (z_end - z_start) * t
            points.append((radius * math.cos(angle), radius * math.sin(angle), z))
        return wire_from_points(
            points,
            radius=tube_radius,
            radial_segments=14,
            cap_ends=True,
            corner_mode="miter",
        )

    socket = model.part("socket")

    socket_ceramic_outer = LatheGeometry(
        [
            (0.0, -0.028),
            (0.018, -0.028),
            (0.022, -0.026),
            (0.025, -0.018),
            (0.027, -0.006),
            (0.026, 0.008),
            (0.021, 0.014),
            (0.0, 0.014),
        ],
        segments=72,
    )
    socket_ceramic_body = boolean_difference(
        socket_ceramic_outer,
        CylinderGeometry(radius=0.0152, height=0.018, radial_segments=72).translate(0.0, 0.0, 0.008),
    )
    socket.visual(
        save_mesh("socket_ceramic_body.obj", socket_ceramic_body),
        material=matte_ceramic,
        name="socket_ceramic_body",
    )

    socket.visual(
        Cylinder(radius=0.0245, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=brushed_nickel,
        name="socket_trim_ring",
    )

    socket_collar_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0187, 0.002),
            (0.0193, 0.008),
            (0.0196, 0.016),
            (0.0191, 0.024),
        ],
        [
            (0.0151, 0.004),
            (0.0148, 0.010),
            (0.0147, 0.018),
            (0.0149, 0.024),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    socket.visual(
        save_mesh("socket_collar_shell.obj", socket_collar_shell),
        material=satin_aluminum,
        name="socket_collar_shell",
    )

    socket_thread_guides = MeshGeometry()
    socket_thread_guides.merge(
        helical_thread(
            radius=0.01425,
            z_start=0.006,
            z_end=0.020,
            turns=3.2,
            tube_radius=0.00075,
            start_angle=0.10,
        )
    )
    socket_thread_guides.merge(
        helical_thread(
            radius=0.01425,
            z_start=0.008,
            z_end=0.0195,
            turns=2.8,
            tube_radius=0.0006,
            start_angle=math.pi + 0.38,
        )
    )
    socket.visual(
        save_mesh("socket_thread_guides.obj", socket_thread_guides),
        material=brushed_nickel,
        name="socket_thread_guides",
    )

    socket.visual(
        Cylinder(radius=0.0038, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=dark_hardware,
        name="socket_center_contact",
    )

    socket.visual(
        Cylinder(radius=0.0018, length=0.006),
        origin=Origin(xyz=(0.0187, 0.0, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="retention_screw_right",
    )
    socket.visual(
        Cylinder(radius=0.0018, length=0.006),
        origin=Origin(xyz=(-0.0187, 0.0, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="retention_screw_left",
    )
    socket.inertial = Inertial.from_geometry(
        Box((0.056, 0.056, 0.052)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )

    bulb = model.part("bulb")

    bulb_envelope = LatheGeometry(
        [
            (0.0, 0.098),
            (0.011, 0.094),
            (0.022, 0.084),
            (0.030, 0.064),
            (0.034, 0.040),
            (0.035, 0.016),
            (0.031, 0.004),
            (0.017, -0.002),
            (0.0, -0.002),
        ],
        segments=80,
    )
    bulb.visual(
        save_mesh("bulb_envelope.obj", bulb_envelope),
        material=frosted_glass,
        name="bulb_envelope",
    )

    transition_band = LatheGeometry(
        [
            (0.0, 0.008),
            (0.017, 0.008),
            (0.016, 0.003),
            (0.014, -0.001),
            (0.0150, -0.004),
            (0.0, -0.004),
        ],
        segments=72,
    )
    bulb.visual(
        save_mesh("bulb_transition_band.obj", transition_band),
        material=warm_insulator,
        name="transition_band",
    )

    bulb.visual(
        Cylinder(radius=0.0128, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=satin_aluminum,
        name="screw_shoulder",
    )
    bulb.visual(
        Cylinder(radius=0.0117, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=satin_aluminum,
        name="screw_core",
    )

    bulb_thread = MeshGeometry()
    bulb_thread.merge(
        helical_thread(
            radius=0.01235,
            z_start=-0.0205,
            z_end=-0.0058,
            turns=3.4,
            tube_radius=0.0010,
            start_angle=0.10,
        )
    )
    bulb_thread.merge(
        helical_thread(
            radius=0.01220,
            z_start=-0.0172,
            z_end=-0.0048,
            turns=2.7,
            tube_radius=0.00075,
            start_angle=math.pi + 0.42,
        )
    )
    bulb.visual(
        save_mesh("bulb_thread.obj", bulb_thread),
        material=satin_aluminum,
        name="bulb_thread",
    )

    bulb.visual(
        Cylinder(radius=0.0040, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, -0.0255)),
        material=brushed_nickel,
        name="contact_tip",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.126),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    model.articulation(
        "socket_to_bulb_rotation",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    bulb_spin = object_model.get_articulation("socket_to_bulb_rotation")

    socket_ceramic_body = socket.get_visual("socket_ceramic_body")
    socket_collar_shell = socket.get_visual("socket_collar_shell")
    socket_center_contact = socket.get_visual("socket_center_contact")
    bulb_envelope = bulb.get_visual("bulb_envelope")
    transition_band = bulb.get_visual("transition_band")
    screw_core = bulb.get_visual("screw_core")
    contact_tip = bulb.get_visual("contact_tip")

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

    ctx.expect_origin_distance(socket, bulb, axes="xy", max_dist=0.0003)
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        margin=0.0035,
        inner_elem=screw_core,
        outer_elem=socket_collar_shell,
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a=transition_band,
        elem_b=socket_collar_shell,
        contact_tol=0.0005,
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a=contact_tip,
        elem_b=socket_center_contact,
        contact_tol=0.0005,
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem=bulb_envelope,
        negative_elem=socket_ceramic_body,
        min_gap=0.007,
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem=transition_band,
        negative_elem=socket_collar_shell,
        min_gap=0.0,
        max_gap=0.0008,
        max_penetration=0.0,
    )

    for angle in (1.1, 2.8):
        with ctx.pose({bulb_spin: angle}):
            ctx.expect_origin_distance(socket, bulb, axes="xy", max_dist=0.0003)
            ctx.expect_within(
                bulb,
                socket,
                axes="xy",
                margin=0.0035,
                inner_elem=screw_core,
                outer_elem=socket_collar_shell,
            )
            ctx.expect_contact(
                bulb,
                socket,
                elem_a=transition_band,
                elem_b=socket_collar_shell,
                contact_tol=0.0005,
            )
            ctx.expect_contact(
                bulb,
                socket,
                elem_a=contact_tip,
                elem_b=socket_center_contact,
                contact_tol=0.0005,
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
