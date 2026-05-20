from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _bulb_glass_mesh():
    profile = [
        (0.0, 0.020),
        (0.010, 0.022),
        (0.020, 0.030),
        (0.030, 0.048),
        (0.035, 0.070),
        (0.034, 0.092),
        (0.026, 0.112),
        (0.014, 0.126),
        (0.004, 0.134),
        (0.0, 0.136),
        (0.0, 0.020),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=72),
        ASSETS.mesh_path("utility_bulb_glass.obj"),
    )


def _threaded_shell_mesh():
    profile = [
        (0.0, -0.012),
        (0.006, -0.012),
        (0.006, -0.008),
        (0.0138, -0.008),
        (0.0148, -0.004),
        (0.0159, -0.001),
        (0.0147, 0.002),
        (0.0160, 0.005),
        (0.0149, 0.008),
        (0.0161, 0.011),
        (0.0150, 0.014),
        (0.0160, 0.017),
        (0.0148, 0.020),
        (0.0132, 0.023),
        (0.0105, 0.027),
        (0.0, 0.027),
        (0.0, -0.012),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=72),
        ASSETS.mesh_path("utility_bulb_threaded_shell.obj"),
    )


def _socket_receiver_mesh():
    profile = [
        (0.031, 0.008),
        (0.031, 0.015),
        (0.028, 0.040),
        (0.020, 0.040),
        (0.020, 0.012),
        (0.023, 0.008),
        (0.031, 0.008),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=72),
        ASSETS.mesh_path("utility_socket_receiver.obj"),
    )


def _contact_sleeve_mesh():
    profile = [
        (0.0188, 0.013),
        (0.0188, 0.029),
        (0.0164, 0.029),
        (0.0164, 0.013),
        (0.0188, 0.013),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=64),
        ASSETS.mesh_path("utility_socket_contact_sleeve.obj"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_screw_light_bulb_with_socket", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.20, 0.23, 0.26, 1.0))
    collar_paint = model.material("collar_paint", rgba=(0.29, 0.32, 0.35, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    brass = model.material("brass", rgba=(0.70, 0.58, 0.31, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.08, 0.08, 0.09, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.96, 0.97, 0.98, 0.58))

    socket = model.part("socket")
    socket.visual(
        Box((0.100, 0.100, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=painted_steel,
        name="mount_plate",
    )
    socket.visual(
        Box((0.014, 0.074, 0.020)),
        origin=Origin(xyz=(0.030, 0.0, 0.005)),
        material=painted_steel,
        name="housing_wall_pos_x",
    )
    socket.visual(
        Box((0.014, 0.074, 0.020)),
        origin=Origin(xyz=(-0.030, 0.0, 0.005)),
        material=painted_steel,
        name="housing_wall_neg_x",
    )
    socket.visual(
        Box((0.046, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.030, 0.005)),
        material=painted_steel,
        name="housing_wall_pos_y",
    )
    socket.visual(
        Box((0.046, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.030, 0.005)),
        material=painted_steel,
        name="housing_wall_neg_y",
    )
    socket.visual(
        Box((0.016, 0.050, 0.026)),
        origin=Origin(xyz=(0.026, 0.0, 0.012)),
        material=painted_steel,
        name="reinforcement_rib_pos_x",
    )
    socket.visual(
        Box((0.016, 0.050, 0.026)),
        origin=Origin(xyz=(-0.026, 0.0, 0.012)),
        material=painted_steel,
        name="reinforcement_rib_neg_x",
    )
    socket.visual(
        Box((0.050, 0.016, 0.026)),
        origin=Origin(xyz=(0.0, 0.026, 0.012)),
        material=painted_steel,
        name="reinforcement_rib_pos_y",
    )
    socket.visual(
        Box((0.050, 0.016, 0.026)),
        origin=Origin(xyz=(0.0, -0.026, 0.012)),
        material=painted_steel,
        name="reinforcement_rib_neg_y",
    )
    socket.visual(
        _socket_receiver_mesh(),
        material=collar_paint,
        name="socket_receiver",
    )
    socket.visual(
        _contact_sleeve_mesh(),
        material=brass,
        name="contact_sleeve",
    )
    socket.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=brass,
        name="center_contact",
    )
    for x in (-0.034, 0.034):
        for y in (-0.034, 0.034):
            socket.visual(
                Cylinder(radius=0.0055, length=0.004),
                origin=Origin(xyz=(x, y, -0.001)),
                material=galvanized_steel,
                name=f"fastener_{'p' if x > 0 else 'n'}x_{'p' if y > 0 else 'n'}y",
            )
    socket.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.050)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        _threaded_shell_mesh(),
        material=galvanized_steel,
        name="threaded_shell",
    )
    bulb.visual(
        Cylinder(radius=0.0135, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=black_polymer,
        name="insulator_ring",
    )
    bulb.visual(
        _bulb_glass_mesh(),
        material=frosted_glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=brass,
        name="contact_tip",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.136),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent="socket",
        child="bulb",
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
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
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance("bulb", "socket", axes="xy", max_dist=0.002)
    ctx.expect_aabb_overlap("bulb", "socket", axes="xy", min_overlap=0.038)
    ctx.expect_aabb_contact("bulb", "socket")
    ctx.expect_aabb_gap(
        "bulb",
        "socket",
        axis="z",
        min_gap=-0.025,
        max_gap=-0.010,
        positive_elem="threaded_shell",
        negative_elem="socket_receiver",
        name="thread_engagement_depth",
    )
    ctx.expect_aabb_gap(
        "bulb",
        "socket",
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="contact_tip",
        negative_elem="center_contact",
        name="center_contact_seated",
    )
    ctx.expect_aabb_gap(
        "bulb",
        "socket",
        axis="z",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="glass_envelope",
        negative_elem="socket_receiver",
        name="glass_clears_socket_collar",
    )

    with ctx.pose(socket_to_bulb=2.4):
        ctx.expect_origin_distance("bulb", "socket", axes="xy", max_dist=0.002)
        ctx.expect_aabb_contact("bulb", "socket")
        ctx.expect_aabb_gap(
            "bulb",
            "socket",
            axis="z",
            min_gap=-0.025,
            max_gap=-0.010,
            positive_elem="threaded_shell",
            negative_elem="socket_receiver",
            name="thread_engagement_depth_rotated",
        )

    with ctx.pose(socket_to_bulb=4.8):
        ctx.expect_aabb_gap(
            "bulb",
            "socket",
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="contact_tip",
            negative_elem="center_contact",
            name="center_contact_seated_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
