from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)


BODY_WIDTH = 0.050
BODY_DEPTH = 0.021
BODY_HEIGHT = 0.034
BODY_FRONT_Y = BODY_DEPTH * 0.5

DIAL_RADIUS = 0.0052
DIAL_WIDTH = 0.0095
DIAL_BORE_RADIUS = 0.0028
DIAL_CENTER_Z = 0.0049
DIAL_CENTER_Y = BODY_FRONT_Y + DIAL_WIDTH * 0.5
DIAL_XS = (-0.018, -0.006, 0.006, 0.018)

SHACKLE_RADIUS = 0.0032
SHACKLE_PIVOT_X = -0.014
SHACKLE_FREE_X = 0.014
SHACKLE_PIVOT_Z = 0.038
SHACKLE_ARCH_Z = 0.068
SHACKLE_BARREL_LENGTH = 0.010


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_xz_section(width: float, height: float, radius: float, y: float) -> list[tuple[float, float, float]]:
    return [(x, y, z + BODY_HEIGHT * 0.5) for x, z in rounded_rect_profile(width, height, radius, corner_segments=10)]


def _dial_mesh(name: str):
    half_width = DIAL_WIDTH * 0.5
    outer_profile = [
        (DIAL_RADIUS * 0.82, -half_width),
        (DIAL_RADIUS * 0.95, -half_width * 0.42),
        (DIAL_RADIUS, -half_width * 0.12),
        (DIAL_RADIUS, half_width * 0.12),
        (DIAL_RADIUS * 0.95, half_width * 0.42),
        (DIAL_RADIUS * 0.82, half_width),
    ]
    inner_profile = [
        (DIAL_BORE_RADIUS, -half_width),
        (DIAL_BORE_RADIUS, half_width),
    ]
    dial_geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(pi / 2.0)
    return _save_mesh(name, dial_geom)


def _shackle_arch_mesh(name: str):
    arch_geom = wire_from_points(
        [
            (0.0, 0.0, SHACKLE_RADIUS),
            (0.0, 0.0, 0.017),
            (0.006, 0.0, 0.026),
            (0.014, 0.0, 0.030),
            (0.022, 0.0, 0.026),
            (SHACKLE_FREE_X - SHACKLE_PIVOT_X, 0.0, 0.017),
            (SHACKLE_FREE_X - SHACKLE_PIVOT_X, 0.0, 0.011),
        ],
        radius=SHACKLE_RADIUS,
        radial_segments=20,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.009,
        corner_segments=10,
        up_hint=(0.0, 1.0, 0.0),
    )
    return _save_mesh(name, arch_geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gym_locker_combo_padlock")

    body_finish = model.material("body_finish", rgba=(0.13, 0.14, 0.16, 1.0))
    bezel_finish = model.material("bezel_finish", rgba=(0.23, 0.25, 0.28, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.72, 0.75, 0.79, 1.0))
    clip_finish = model.material("clip_finish", rgba=(0.56, 0.59, 0.63, 1.0))
    shackle_finish = model.material("shackle_finish", rgba=(0.83, 0.86, 0.89, 1.0))

    dial_mesh = _dial_mesh("dial_wheel")
    shackle_arch_mesh = _shackle_arch_mesh("shackle_arch")

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, 0.041)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
    )

    shell_geom = section_loft(
        [
            _rounded_xz_section(BODY_WIDTH, BODY_HEIGHT, 0.0052, -BODY_FRONT_Y),
            _rounded_xz_section(BODY_WIDTH + 0.0016, BODY_HEIGHT + 0.0012, 0.0058, 0.0),
            _rounded_xz_section(BODY_WIDTH, BODY_HEIGHT, 0.0052, BODY_FRONT_Y),
        ]
    )
    body.visual(_save_mesh("body_shell", shell_geom), material=body_finish, name="main_shell")
    body.visual(
        Box((0.034, 0.0024, 0.017)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.0012, 0.017)),
        material=bezel_finish,
        name="front_bezel",
    )
    body.visual(
        Box((0.010, BODY_DEPTH, 0.006)),
        origin=Origin(xyz=(SHACKLE_PIVOT_X, 0.0, 0.031)),
        material=body_finish,
        name="pivot_pedestal",
    )
    body.visual(
        Box((0.010, 0.006, 0.009)),
        origin=Origin(xyz=(SHACKLE_PIVOT_X, 0.008, 0.0365)),
        material=body_finish,
        name="left_front_cheek",
    )
    body.visual(
        Box((0.010, 0.006, 0.009)),
        origin=Origin(xyz=(SHACKLE_PIVOT_X, -0.008, 0.0365)),
        material=body_finish,
        name="left_back_cheek",
    )
    body.visual(
        Box((0.010, BODY_DEPTH, 0.010)),
        origin=Origin(xyz=(SHACKLE_FREE_X, 0.0, 0.0330)),
        material=body_finish,
        name="receiver_pedestal",
    )
    body.visual(
        Cylinder(radius=SHACKLE_RADIUS, length=0.010),
        origin=Origin(xyz=(SHACKLE_FREE_X, 0.0, 0.0368), rpy=(pi / 2.0, 0.0, 0.0)),
        material=clip_finish,
        name="right_receiver",
    )

    shaft_radius = 0.0022
    retainer_radius = 0.0038
    shaft_length = BODY_FRONT_Y + DIAL_WIDTH + 0.0019
    shaft_center_y = BODY_FRONT_Y + shaft_length * 0.5
    retainer_length = 0.0019
    retainer_center_y = BODY_FRONT_Y + DIAL_WIDTH + retainer_length * 0.5
    for index, dial_x in enumerate(DIAL_XS, start=1):
        body.visual(
            Cylinder(radius=shaft_radius, length=shaft_length),
            origin=Origin(xyz=(dial_x, shaft_center_y, DIAL_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=clip_finish,
            name=f"shaft_{index}",
        )
        body.visual(
            Cylinder(radius=retainer_radius, length=retainer_length),
            origin=Origin(xyz=(dial_x, retainer_center_y, DIAL_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=clip_finish,
            name=f"retainer_{index}",
        )

    shackle = model.part("shackle")
    shackle.inertial = Inertial.from_geometry(
        Box((0.034, SHACKLE_BARREL_LENGTH, 0.035)),
        mass=0.05,
        origin=Origin(xyz=(0.014, 0.0, 0.018)),
    )
    shackle.visual(shackle_arch_mesh, material=shackle_finish, name="shackle_arch")
    shackle.visual(
        Cylinder(radius=SHACKLE_RADIUS, length=0.012),
        origin=Origin(xyz=(SHACKLE_FREE_X - SHACKLE_PIVOT_X, 0.0, 0.046 - SHACKLE_PIVOT_Z), rpy=(0.0, 0.0, 0.0)),
        material=shackle_finish,
        name="free_leg",
    )
    shackle.visual(
        Cylinder(radius=SHACKLE_RADIUS, length=SHACKLE_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shackle_finish,
        name="hinge_barrel",
    )

    model.articulation(
        "shackle_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(SHACKLE_PIVOT_X, 0.0, SHACKLE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.2),
    )

    for index, dial_x in enumerate(DIAL_XS, start=1):
        dial = model.part(f"dial_{index}")
        dial.inertial = Inertial.from_geometry(
            Cylinder(radius=DIAL_RADIUS, length=DIAL_WIDTH),
            mass=0.012,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        dial.visual(dial_mesh, material=dial_finish, name="wheel_shell")
        model.articulation(
            f"dial_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=dial,
            origin=Origin(xyz=(dial_x, DIAL_CENTER_Y, DIAL_CENTER_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.3, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    shackle_hinge = object_model.get_articulation("shackle_hinge")
    dials = [object_model.get_part(f"dial_{index}") for index in range(1, 5)]
    dial_joints = [object_model.get_articulation(f"dial_{index}_spin") for index in range(1, 5)]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "shackle_hinge_axis",
        tuple(abs(value) for value in shackle_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"expected shackle axis parallel to y, got {shackle_hinge.axis}",
    )
    for index, joint in enumerate(dial_joints, start=1):
        ctx.check(
            f"dial_{index}_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"expected dial axis (0, 1, 0), got {joint.axis}",
        )

    ctx.expect_contact(shackle, body, elem_a="hinge_barrel", elem_b="left_front_cheek", contact_tol=2e-4)
    ctx.expect_contact(shackle, body, elem_a="hinge_barrel", elem_b="left_back_cheek", contact_tol=2e-4)
    ctx.expect_contact(shackle, body, elem_a="free_leg", elem_b="right_receiver", contact_tol=2e-4)

    for index, dial in enumerate(dials, start=1):
        ctx.expect_gap(
            dial,
            body,
            axis="y",
            positive_elem="wheel_shell",
            negative_elem="main_shell",
            min_gap=-1e-6,
            max_gap=3e-4,
            name=f"dial_{index}_rear_face_seated",
        )
        ctx.expect_contact(
            dial,
            body,
            elem_a="wheel_shell",
            elem_b=f"retainer_{index}",
            contact_tol=3e-4,
            name=f"dial_{index}_captured_by_retainer",
        )

    for index in range(1, 4):
        ctx.expect_gap(
            dials[index],
            dials[index - 1],
            axis="x",
            min_gap=0.001,
            name=f"dial_{index}_to_{index + 1}_spacing",
        )

    with ctx.pose({shackle_hinge: 1.05}):
        ctx.expect_gap(
            shackle,
            body,
            axis="z",
            positive_elem="free_leg",
            negative_elem="main_shell",
            min_gap=0.016,
            name="free_leg_lifts_clear_when_open",
        )
        ctx.expect_contact(
            shackle,
            body,
            elem_a="hinge_barrel",
            elem_b="left_front_cheek",
            contact_tol=3e-4,
            name="hinge_stays_captive_when_open",
        )

    dial_pose = {
        dial_joints[0]: pi / 3.0,
        dial_joints[1]: pi / 2.0,
        dial_joints[2]: -pi / 4.0,
        dial_joints[3]: pi,
    }
    with ctx.pose(dial_pose):
        for index, dial in enumerate(dials, start=1):
            ctx.expect_contact(
                dial,
                body,
                elem_a="wheel_shell",
                elem_b=f"retainer_{index}",
                contact_tol=3e-4,
                name=f"dial_{index}_stays_captured_while_spun",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
