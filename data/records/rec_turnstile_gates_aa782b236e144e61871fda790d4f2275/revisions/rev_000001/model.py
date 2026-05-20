from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_PLATE_SIZE = 0.22
BASE_PLATE_THICKNESS = 0.012
POST_RADIUS = 0.045
POST_HEIGHT = 0.900
POST_TOP_Z = BASE_PLATE_THICKNESS + POST_HEIGHT

THRUST_WASHER_RADIUS = 0.066
THRUST_WASHER_THICKNESS = 0.008
THRUST_WASHER_CENTER_Z = POST_TOP_Z + (THRUST_WASHER_THICKNESS * 0.5)

BEARING_CORE_RADIUS = 0.025
BEARING_CORE_LENGTH = 0.082
BEARING_CORE_CENTER_Z = POST_TOP_Z + THRUST_WASHER_THICKNESS + (BEARING_CORE_LENGTH * 0.5)

SPINDLE_PIN_RADIUS = 0.012
SPINDLE_PIN_LENGTH = 0.122
SPINDLE_PIN_CENTER_Z = POST_TOP_Z + THRUST_WASHER_THICKNESS + (SPINDLE_PIN_LENGTH * 0.5)

CAPTURE_CAP_RADIUS = 0.034
CAPTURE_CAP_THICKNESS = 0.010
CAPTURE_CAP_CENTER_Z = 1.021

ROTOR_CENTER_Z = 0.967


def _build_rotor_mesh():
    hub = LatheGeometry.from_shell_profiles(
        [
            (0.066, -0.047),
            (0.066, -0.038),
            (0.060, -0.022),
            (0.060, 0.030),
            (0.050, 0.047),
        ],
        [
            (0.032, -0.047),
            (0.032, 0.047),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )

    for index in range(3):
        angle = index * (math.tau / 3.0)

        socket = CylinderGeometry(radius=0.027, height=0.086, radial_segments=28)
        socket.rotate_y(math.pi / 2.0).translate(0.097, 0.0, 0.0).rotate_z(angle)
        hub.merge(socket)

        arm = CylinderGeometry(radius=0.018, height=0.466, radial_segments=28)
        arm.rotate_y(math.pi / 2.0).translate(0.287, 0.0, 0.0).rotate_z(angle)
        hub.merge(arm)

        gusset = BoxGeometry((0.078, 0.018, 0.048))
        gusset.translate(0.082, 0.0, -0.016).rotate_z(angle)
        hub.merge(gusset)

    return hub


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mass_manufacturable_turnstile_gate")

    powder_coat = model.material("powder_coat", rgba=(0.23, 0.25, 0.27, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.70, 0.73, 0.76, 1.0))
    molded_polymer = model.material("molded_polymer", rgba=(0.12, 0.13, 0.14, 1.0))

    support_post = model.part("support_post")
    support_post.visual(
        Box((BASE_PLATE_SIZE, BASE_PLATE_SIZE, BASE_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS * 0.5)),
        material=powder_coat,
        name="base_plate",
    )
    support_post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS + (POST_HEIGHT * 0.5))),
        material=powder_coat,
        name="post_tube",
    )
    support_post.visual(
        Cylinder(radius=THRUST_WASHER_RADIUS, length=THRUST_WASHER_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, THRUST_WASHER_CENTER_Z)),
        material=galvanized_steel,
        name="thrust_washer",
    )
    support_post.visual(
        Cylinder(radius=BEARING_CORE_RADIUS, length=BEARING_CORE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, BEARING_CORE_CENTER_Z)),
        material=molded_polymer,
        name="bearing_core",
    )
    support_post.visual(
        Cylinder(radius=SPINDLE_PIN_RADIUS, length=SPINDLE_PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_PIN_CENTER_Z)),
        material=galvanized_steel,
        name="spindle_pin",
    )
    support_post.visual(
        Cylinder(radius=CAPTURE_CAP_RADIUS, length=CAPTURE_CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, CAPTURE_CAP_CENTER_Z)),
        material=galvanized_steel,
        name="capture_cap",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(_build_rotor_mesh(), "turnstile_rotor_body"),
        material=galvanized_steel,
        name="rotor_body",
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=support_post,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, ROTOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_post = object_model.get_part("support_post")
    rotor = object_model.get_part("rotor")
    rotor_spin = object_model.get_articulation("rotor_spin")

    thrust_washer = support_post.get_visual("thrust_washer")
    bearing_core = support_post.get_visual("bearing_core")
    capture_cap = support_post.get_visual("capture_cap")
    rotor_body = rotor.get_visual("rotor_body")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.check(
        "rotor_joint_is_continuous",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS,
        f"unexpected articulation type: {rotor_spin.articulation_type}",
    )
    ctx.check(
        "rotor_joint_axis_is_vertical",
        tuple(round(value, 6) for value in rotor_spin.axis) == (0.0, 0.0, 1.0),
        f"axis={rotor_spin.axis}",
    )
    ctx.expect_origin_distance(
        rotor,
        support_post,
        axes="xy",
        max_dist=0.0005,
        name="rotor_is_centered_on_support_post",
    )
    ctx.expect_contact(
        rotor,
        support_post,
        elem_a=rotor_body,
        elem_b=thrust_washer,
        contact_tol=1e-5,
        name="rotor_is_supported_by_thrust_washer",
    )
    ctx.expect_overlap(
        rotor,
        support_post,
        axes="xy",
        elem_a=rotor_body,
        elem_b=bearing_core,
        min_overlap=0.045,
        name="rotor_hub_wraps_bearing_core",
    )
    ctx.expect_gap(
        support_post,
        rotor,
        axis="z",
        positive_elem=capture_cap,
        negative_elem=rotor_body,
        min_gap=0.001,
        max_gap=0.004,
        name="capture_cap_retains_rotor_without_dragging",
    )

    with ctx.pose({rotor_spin: math.tau / 3.0}):
        ctx.expect_contact(
            rotor,
            support_post,
            elem_a=rotor_body,
            elem_b=thrust_washer,
            contact_tol=1e-5,
            name="rotor_support_contact_persists_after_rotation",
        )
        ctx.expect_gap(
            support_post,
            rotor,
            axis="z",
            positive_elem=capture_cap,
            negative_elem=rotor_body,
            min_gap=0.001,
            max_gap=0.004,
            name="capture_cap_clearance_persists_after_rotation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
