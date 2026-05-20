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
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_screwin_light_bulb_with_socket", assets=ASSETS)

    painted_socket = model.material("painted_socket", rgba=(0.34, 0.38, 0.35, 1.0))
    collar_steel = model.material("collar_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    ceramic = model.material("ceramic", rgba=(0.86, 0.83, 0.75, 1.0))
    zinc = model.material("zinc", rgba=(0.73, 0.75, 0.78, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.88, 0.91, 0.94, 0.40))
    nickel = model.material("nickel", rgba=(0.76, 0.77, 0.79, 1.0))
    brass = model.material("brass", rgba=(0.76, 0.62, 0.27, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.10, 0.10, 0.11, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def helix_points(
        *,
        radius: float,
        z_start: float,
        z_end: float,
        turns: float,
        phase: float = 0.0,
        samples_per_turn: int = 36,
    ) -> list[tuple[float, float, float]]:
        samples = max(12, int(math.ceil(turns * samples_per_turn)))
        return [
            (
                radius * math.cos(phase + 2.0 * math.pi * turns * t),
                radius * math.sin(phase + 2.0 * math.pi * turns * t),
                z_start + (z_end - z_start) * t,
            )
            for t in [index / samples for index in range(samples + 1)]
        ]

    socket = model.part("socket")
    socket.inertial = Inertial.from_geometry(
        Box((0.116, 0.096, 0.074)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
    )
    socket.visual(
        Box((0.112, 0.092, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=painted_socket,
        name="mount_flange",
    )
    socket.visual(
        Box((0.096, 0.078, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=painted_socket,
        name="housing_body",
    )
    socket.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=painted_socket,
        name="deck_ring",
    )
    socket.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.047, 0.0, 0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_socket,
        name="conduit_boss_right",
    )
    socket.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(-0.047, 0.0, 0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_socket,
        name="conduit_boss_left",
    )
    for x_sign in (-1.0, 1.0):
        socket.visual(
            Box((0.020, 0.012, 0.012)),
            origin=Origin(xyz=(0.032 * x_sign, 0.0, 0.036)),
            material=painted_socket,
            name=f"reinforcement_web_x_{'p' if x_sign > 0 else 'n'}",
        )
    for y_sign in (-1.0, 1.0):
        socket.visual(
            Box((0.012, 0.020, 0.012)),
            origin=Origin(xyz=(0.0, 0.032 * y_sign, 0.036)),
            material=painted_socket,
            name=f"reinforcement_web_y_{'p' if y_sign > 0 else 'n'}",
        )

    collar_shell_mesh = save_mesh(
        "socket_collar_shell.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.029, 0.0), (0.029, 0.030)],
            inner_profile=[(0.0183, 0.0), (0.0183, 0.030)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    socket.visual(
        collar_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=collar_steel,
        name="collar_shell",
    )
    collar_rim_mesh = save_mesh(
        "socket_collar_rim.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.032, 0.0), (0.032, 0.004)],
            inner_profile=[(0.0183, 0.0), (0.0183, 0.004)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    socket.visual(
        collar_rim_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=collar_steel,
        name="collar_rim",
    )
    liner_mesh = save_mesh(
        "socket_ceramic_liner.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.0183, 0.0), (0.0183, 0.028)],
            inner_profile=[(0.01695, 0.0), (0.01695, 0.028)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    socket.visual(
        liner_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=ceramic,
        name="ceramic_liner",
    )
    socket.visual(
        Cylinder(radius=0.01695, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0419)),
        material=ceramic,
        name="contact_bridge",
    )
    female_thread_mesh = save_mesh(
        "socket_female_thread.obj",
        wire_from_points(
            helix_points(
                radius=0.01655,
                z_start=0.003,
                z_end=0.026,
                turns=4.9,
                phase=0.35,
            ),
            radius=0.00050,
            radial_segments=12,
            cap_ends=True,
            corner_mode="miter",
        ),
    )
    socket.visual(
        female_thread_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=ceramic,
        name="female_thread",
    )
    socket.visual(
        Cylinder(radius=0.0074, length=0.0026),
        origin=Origin(xyz=(0.0, 0.0, 0.0434)),
        material=ceramic,
        name="contact_insulator",
    )
    socket.visual(
        Cylinder(radius=0.0034, length=0.0022),
        origin=Origin(xyz=(0.0, 0.0, 0.0447)),
        material=brass,
        name="contact_pad",
    )
    for index, (x, y) in enumerate(((-0.028, -0.022), (-0.028, 0.022), (0.028, -0.022), (0.028, 0.022))):
        socket.visual(
            Cylinder(radius=0.0053, length=0.0040),
            origin=Origin(xyz=(x, y, 0.0350)),
            material=zinc,
            name=f"housing_screw_head_{index}",
        )
        socket.visual(
            Box((0.0060, 0.0014, 0.0010)),
            origin=Origin(xyz=(x, y, 0.0371)),
            material=painted_socket,
            name=f"housing_screw_slot_{index}",
        )
    for x_sign in (-1.0, 1.0):
        socket.visual(
            Box((0.012, 0.006, 0.026)),
            origin=Origin(xyz=(0.029 * x_sign, 0.0, 0.055)),
            material=collar_steel,
            name=f"collar_rib_x_{'p' if x_sign > 0 else 'n'}",
        )
    for y_sign in (-1.0, 1.0):
        socket.visual(
            Box((0.006, 0.012, 0.026)),
            origin=Origin(xyz=(0.0, 0.029 * y_sign, 0.055)),
            material=collar_steel,
            name=f"collar_rib_y_{'p' if y_sign > 0 else 'n'}",
        )

    bulb = model.part("bulb")
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.175),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )
    bulb.visual(
        Cylinder(radius=0.0183, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=nickel,
        name="seating_flange",
    )
    bulb.visual(
        Cylinder(radius=0.0150, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=nickel,
        name="neck_adapter",
    )
    bulb.visual(
        Cylinder(radius=0.0183, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=gasket_black,
        name="gasket_ring",
    )
    bulb.visual(
        Cylinder(radius=0.0133, length=0.0259),
        origin=Origin(xyz=(0.0, 0.0, -0.00895)),
        material=nickel,
        name="screw_shell_core",
    )
    thread_mesh = save_mesh(
        "bulb_thread_ridge.obj",
        wire_from_points(
            helix_points(
                radius=0.01395,
                z_start=-0.0185,
                z_end=0.0035,
                turns=4.9,
                phase=0.35,
            ),
            radius=0.00075,
            radial_segments=12,
            cap_ends=True,
            corner_mode="miter",
        ),
    )
    bulb.visual(
        thread_mesh,
        material=nickel,
        name="thread_ridge",
    )
    bulb.visual(
        Cylinder(radius=0.0048, length=0.0026),
        origin=Origin(xyz=(0.0, 0.0, -0.0232)),
        material=gasket_black,
        name="tip_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0032, length=0.0022),
        origin=Origin(xyz=(0.0, 0.0, -0.0219)),
        material=brass,
        name="tip_contact",
    )
    bulb.visual(
        Cylinder(radius=0.0170, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=nickel,
        name="neck_retaining_band",
    )
    glass_envelope_mesh = save_mesh(
        "glass_envelope.obj",
        LatheGeometry(
            [
                (0.0000, 0.000),
                (0.0105, 0.000),
                (0.0150, 0.006),
                (0.0225, 0.019),
                (0.0315, 0.047),
                (0.0365, 0.079),
                (0.0340, 0.108),
                (0.0240, 0.134),
                (0.0090, 0.145),
                (0.0000, 0.149),
            ],
            segments=72,
        ),
    )
    bulb.visual(
        glass_envelope_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=frosted_glass,
        name="glass_envelope",
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0688)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    bulb_spin = object_model.get_articulation("bulb_spin")
    collar_shell = socket.get_visual("collar_shell")
    collar_rim = socket.get_visual("collar_rim")
    contact_pad = socket.get_visual("contact_pad")
    female_thread = socket.get_visual("female_thread")
    glass_envelope = bulb.get_visual("glass_envelope")
    seating_flange = bulb.get_visual("seating_flange")
    screw_shell = bulb.get_visual("screw_shell_core")
    thread_ridge = bulb.get_visual("thread_ridge")
    tip_contact = bulb.get_visual("tip_contact")

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

    ctx.expect_origin_distance(
        bulb,
        socket,
        axes="xy",
        max_dist=0.0005,
        name="bulb stays coaxial with socket",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a=seating_flange,
        elem_b=collar_rim,
        name="seating flange contacts collar rim",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="xy",
        elem_a=screw_shell,
        elem_b=collar_shell,
        min_overlap=0.026,
        name="screw shell sits within collar footprint",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem=tip_contact,
        negative_elem=contact_pad,
        max_gap=0.0006,
        max_penetration=0.0002,
        name="center contact lands on socket pad",
    )

    collar_aabb = ctx.part_element_world_aabb(socket, elem=collar_shell)
    screw_aabb = ctx.part_element_world_aabb(bulb, elem=screw_shell)
    thread_aabb = ctx.part_element_world_aabb(bulb, elem=thread_ridge)
    envelope_aabb = ctx.part_element_world_aabb(bulb, elem=glass_envelope)
    socket_aabb = ctx.part_world_aabb(socket)
    bulb_aabb = ctx.part_world_aabb(bulb)

    if collar_aabb is None or screw_aabb is None or thread_aabb is None or envelope_aabb is None or socket_aabb is None or bulb_aabb is None:
        ctx.fail("measurable geometry available", "Expected measurable AABBs for socket and bulb visuals.")
    else:
        engagement_depth = min(collar_aabb[1][2], screw_aabb[1][2]) - max(collar_aabb[0][2], screw_aabb[0][2])
        exposed_thread = thread_aabb[1][2] - collar_aabb[1][2]
        total_height = max(socket_aabb[1][2], bulb_aabb[1][2]) - min(socket_aabb[0][2], bulb_aabb[0][2])
        envelope_diameter = envelope_aabb[1][0] - envelope_aabb[0][0]
        socket_width = socket_aabb[1][0] - socket_aabb[0][0]
        ctx.check(
            "thread engagement depth is practical",
            engagement_depth >= 0.018,
            f"Measured engagement depth {engagement_depth:.4f} m.",
        )
        ctx.check(
            "some male thread remains visible above collar",
            exposed_thread >= 0.0025,
            f"Visible thread above collar {exposed_thread:.4f} m.",
        )
        ctx.check(
            "overall proportions read as a utility bulb with socket",
            0.19 <= total_height <= 0.24 and 0.068 <= envelope_diameter <= 0.078 and 0.10 <= socket_width <= 0.13,
            (
                f"total_height={total_height:.4f} m, "
                f"envelope_diameter={envelope_diameter:.4f} m, "
                f"socket_width={socket_width:.4f} m"
            ),
        )

    with ctx.pose({bulb_spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            bulb,
            socket,
            axes="xy",
            max_dist=0.0005,
            name="rotation remains centered on socket axis",
        )
        ctx.expect_contact(
            bulb,
            socket,
            elem_a=seating_flange,
            elem_b=collar_rim,
            name="flange remains seated during rotation",
        )
        ctx.expect_overlap(
            bulb,
            socket,
            axes="xy",
            elem_a=thread_ridge,
            elem_b=female_thread,
            min_overlap=0.026,
            name="threaded interface stays concentrically aligned during rotation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
