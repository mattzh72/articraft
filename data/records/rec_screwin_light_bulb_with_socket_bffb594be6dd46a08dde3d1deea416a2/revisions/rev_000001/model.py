from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_light_bulb_in_socket")

    socket_black = model.material("socket_black", rgba=(0.14, 0.12, 0.10, 1.0))
    ceramic_white = model.material("ceramic_white", rgba=(0.93, 0.91, 0.86, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.61, 0.33, 1.0))
    nickel = model.material("nickel", rgba=(0.69, 0.71, 0.74, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.95, 0.96, 0.90, 0.38))
    dark_contact = model.material("dark_contact", rgba=(0.26, 0.23, 0.20, 1.0))

    socket_body_shell = _mesh(
        "socket_body_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.0215, 0.000),
                (0.0230, 0.006),
                (0.0230, 0.016),
                (0.0218, 0.021),
                (0.0208, 0.023),
            ],
            inner_profile=[
                (0.0170, 0.004),
                (0.0178, 0.015),
                (0.0175, 0.020),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    guide_sleeve_shell = _mesh(
        "guide_sleeve_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.0152, 0.000),
                (0.0152, 0.022),
                (0.0150, 0.028),
            ],
            inner_profile=[
                (0.0139, 0.0015),
                (0.0139, 0.0265),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    bulb_glass_shell = _mesh(
        "bulb_glass_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.0108, 0.026),
                (0.0132, 0.032),
                (0.0175, 0.040),
                (0.0245, 0.053),
                (0.0300, 0.070),
                (0.0285, 0.090),
                (0.0200, 0.106),
                (0.0080, 0.114),
                (0.0000, 0.117),
            ],
            inner_profile=[
                (0.0095, 0.028),
                (0.0117, 0.033),
                (0.0158, 0.041),
                (0.0228, 0.054),
                (0.0278, 0.070),
                (0.0262, 0.089),
                (0.0182, 0.104),
                (0.0070, 0.111),
                (0.0000, 0.114),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    socket_body = model.part("socket_body")
    socket_body.visual(
        socket_body_shell,
        material=socket_black,
        name="socket_shell",
    )
    socket_body.visual(
        Cylinder(radius=0.0185, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=socket_black,
        name="socket_foot",
    )
    socket_body.inertial = Inertial.from_geometry(
        Box((0.046, 0.046, 0.023)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
    )

    support_sleeve = model.part("support_sleeve")
    support_sleeve.visual(
        guide_sleeve_shell,
        material=nickel,
        name="threaded_collar_sleeve",
    )
    support_sleeve.visual(
        Cylinder(radius=0.0151, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=ceramic_white,
        name="insulator_flange",
    )
    support_sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0152, length=0.028),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0124, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=brass,
        name="thread_shell",
    )
    for index, z_pos in enumerate((0.0045, 0.0085, 0.0125, 0.0165, 0.0205)):
        bulb.visual(
            Cylinder(radius=0.0133, length=0.0018),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=brass,
            name=f"thread_ridge_{index}",
        )
    bulb.visual(
        Cylinder(radius=0.0036, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0015)),
        material=dark_contact,
        name="contact_tip",
    )
    bulb.visual(
        Cylinder(radius=0.0101, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=dark_contact,
        name="neck_transition",
    )
    bulb.visual(
        Cylinder(radius=0.0042, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=dark_contact,
        name="stem_post",
    )
    bulb.visual(
        bulb_glass_shell,
        material=warm_glass,
        name="glass_envelope",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.117),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, 0.0585)),
    )

    model.articulation(
        "socket_to_support",
        ArticulationType.FIXED,
        parent=socket_body,
        child=support_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "support_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=support_sleeve,
        child=bulb,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bulb = object_model.get_part("bulb")
    support_sleeve = object_model.get_part("support_sleeve")
    socket_body = object_model.get_part("socket_body")
    spin = object_model.get_articulation("support_to_bulb")

    ctx.expect_within(
        bulb,
        support_sleeve,
        axes="xy",
        inner_elem="thread_shell",
        outer_elem="threaded_collar_sleeve",
        margin=0.0,
        name="thread shell stays centered within guide sleeve",
    )
    ctx.expect_overlap(
        bulb,
        support_sleeve,
        axes="z",
        elem_a="thread_shell",
        elem_b="threaded_collar_sleeve",
        min_overlap=0.020,
        name="thread shell remains inserted in guide sleeve",
    )
    ctx.expect_gap(
        bulb,
        bulb,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="thread_shell",
        min_gap=0.0,
        max_gap=0.010,
        name="glass envelope rises just above the threaded shell",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({spin: 1.7}):
        turned_pos = ctx.part_world_position(bulb)
        ctx.expect_within(
            bulb,
            support_sleeve,
            axes="xy",
            inner_elem="thread_shell",
            outer_elem="threaded_collar_sleeve",
            margin=0.0,
            name="thread shell stays centered after rotation",
        )
    same_position = (
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) <= 1e-6
        and abs(rest_pos[1] - turned_pos[1]) <= 1e-6
        and abs(rest_pos[2] - turned_pos[2]) <= 1e-6
    )
    ctx.check(
        "continuous spin keeps bulb origin on socket axis",
        same_position,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
