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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PLATE_LENGTH = 0.190
PLATE_WIDTH = 0.100
PLATE_THICKNESS = 0.008
MOUNT_HOLE_DIAMETER = 0.008
MOUNT_HOLE_X = 0.060
MOUNT_HOLE_Y = 0.032

LINK_THICKNESS = 0.008
PIN_RADIUS = 0.0045
EYE_WIDTH = 0.010
EAR_THICKNESS = 0.005
PIN_LENGTH = EYE_WIDTH + 2.0 * EAR_THICKNESS
KNUCKLE_RADIUS = 0.013
YOKE_OFFSET = EYE_WIDTH / 2.0 + EAR_THICKNESS / 2.0

ROOT_HINGE_X = 0.055
ROOT_HINGE_Z = 0.022
ROOT_CHEEK_BACKSET = 0.028

LINK1_PITCH = 0.082
LINK2_PITCH = 0.070
LINK3_PITCH = 0.058
END_TAB_LENGTH = 0.036
END_TAB_WIDTH = 0.018

PROXIMAL_STRAP_START = 0.010
DISTAL_YOKE_LENGTH = 0.018


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_y_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_mount_plate_visuals(part, material: str) -> None:
    _add_box(
        part,
        (PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS),
        (0.0, 0.0, PLATE_THICKNESS / 2.0),
        material=material,
        name="plate_body",
    )

    cheek_height = ROOT_HINGE_Z - PLATE_THICKNESS + 0.001
    cheek_size = (ROOT_CHEEK_BACKSET, EAR_THICKNESS, cheek_height)
    cheek_center_x = ROOT_HINGE_X - ROOT_CHEEK_BACKSET / 2.0
    cheek_center_z = PLATE_THICKNESS + cheek_height / 2.0
    for index, side in enumerate((-1.0, 1.0), start=1):
        y_pos = side * YOKE_OFFSET
        _add_box(
            part,
            cheek_size,
            (cheek_center_x, y_pos, cheek_center_z),
            material=material,
            name=f"root_cheek_{index}",
        )
        _add_y_cylinder(
            part,
            KNUCKLE_RADIUS,
            EAR_THICKNESS,
            (ROOT_HINGE_X, y_pos, ROOT_HINGE_Z),
            material=material,
            name=f"root_ear_{index}",
        )


def _add_link_visuals(part, pitch: float, body_width: float, *, material: str, prefix: str) -> None:
    mid_end = pitch - DISTAL_YOKE_LENGTH
    mid_length = mid_end - PROXIMAL_STRAP_START
    arm_length = pitch - mid_end

    _add_y_cylinder(
        part,
        KNUCKLE_RADIUS,
        EYE_WIDTH,
        (0.0, 0.0, 0.0),
        material=material,
        name=f"{prefix}_prox_eye",
    )
    _add_box(
        part,
        (mid_length, body_width, LINK_THICKNESS),
        (PROXIMAL_STRAP_START + mid_length / 2.0, 0.0, 0.0),
        material=material,
        name=f"{prefix}_strap",
    )

    for index, side in enumerate((-1.0, 1.0), start=1):
        y_pos = side * YOKE_OFFSET
        _add_box(
            part,
            (arm_length, EAR_THICKNESS, LINK_THICKNESS),
            (mid_end + arm_length / 2.0, y_pos, 0.0),
            material=material,
            name=f"{prefix}_arm_{index}",
        )
        _add_y_cylinder(
            part,
            KNUCKLE_RADIUS,
            EAR_THICKNESS,
            (pitch, y_pos, 0.0),
            material=material,
            name=f"{prefix}_distal_ear_{index}",
        )


def _add_end_tab_visuals(part, material: str) -> None:
    _add_y_cylinder(
        part,
        KNUCKLE_RADIUS,
        EYE_WIDTH,
        (0.0, 0.0, 0.0),
        material=material,
        name="end_tab_eye",
    )
    _add_box(
        part,
        (0.020, 0.014, LINK_THICKNESS),
        (0.016, 0.0, 0.0),
        material=material,
        name="end_tab_neck",
    )
    _add_box(
        part,
        (0.022, END_TAB_WIDTH, LINK_THICKNESS),
        (0.034, 0.0, 0.0),
        material=material,
        name="end_tab_pad",
    )
    _add_y_cylinder(
        part,
        0.009,
        END_TAB_WIDTH,
        (0.045, 0.0, 0.0),
        material=material,
        name="end_tab_nose",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_revolute_chain")

    model.material("powder_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("anodized_aluminum", rgba=(0.69, 0.72, 0.76, 1.0))
    model.material("dark_anodized", rgba=(0.34, 0.37, 0.40, 1.0))

    mount_plate = model.part("mount_plate")
    _add_mount_plate_visuals(mount_plate, "powder_steel")
    mount_plate.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, ROOT_HINGE_Z + KNUCKLE_RADIUS)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, (ROOT_HINGE_Z + KNUCKLE_RADIUS) / 2.0)),
    )

    link_1 = model.part("link_1")
    _add_link_visuals(link_1, LINK1_PITCH, 0.028, material="anodized_aluminum", prefix="link_1")
    link_1.inertial = Inertial.from_geometry(
        Box((LINK1_PITCH + KNUCKLE_RADIUS, PIN_LENGTH, KNUCKLE_RADIUS * 2.0)),
        mass=0.28,
        origin=Origin(xyz=(LINK1_PITCH * 0.52, 0.0, 0.0)),
    )

    link_2 = model.part("link_2")
    _add_link_visuals(link_2, LINK2_PITCH, 0.025, material="anodized_aluminum", prefix="link_2")
    link_2.inertial = Inertial.from_geometry(
        Box((LINK2_PITCH + KNUCKLE_RADIUS, PIN_LENGTH, KNUCKLE_RADIUS * 2.0)),
        mass=0.23,
        origin=Origin(xyz=(LINK2_PITCH * 0.52, 0.0, 0.0)),
    )

    link_3 = model.part("link_3")
    _add_link_visuals(link_3, LINK3_PITCH, 0.022, material="anodized_aluminum", prefix="link_3")
    link_3.inertial = Inertial.from_geometry(
        Box((LINK3_PITCH + KNUCKLE_RADIUS, PIN_LENGTH, KNUCKLE_RADIUS * 2.0)),
        mass=0.18,
        origin=Origin(xyz=(LINK3_PITCH * 0.52, 0.0, 0.0)),
    )

    end_tab = model.part("end_tab")
    _add_end_tab_visuals(end_tab, "dark_anodized")
    end_tab.inertial = Inertial.from_geometry(
        Box((END_TAB_LENGTH + KNUCKLE_RADIUS, 0.022, KNUCKLE_RADIUS * 2.0)),
        mass=0.08,
        origin=Origin(xyz=(END_TAB_LENGTH * 0.45, 0.0, 0.0)),
    )

    model.articulation(
        "mount_to_link_1",
        ArticulationType.REVOLUTE,
        parent=mount_plate,
        child=link_1,
        origin=Origin(xyz=(ROOT_HINGE_X, 0.0, ROOT_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.15, effort=28.0, velocity=2.0),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK1_PITCH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.25, effort=22.0, velocity=2.2),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK2_PITCH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.30, effort=18.0, velocity=2.4),
    )
    model.articulation(
        "link_3_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=end_tab,
        origin=Origin(xyz=(LINK3_PITCH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.35, effort=10.0, velocity=2.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount_plate = object_model.get_part("mount_plate")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    end_tab = object_model.get_part("end_tab")

    mount_to_link_1 = object_model.get_articulation("mount_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")
    link_3_to_end_tab = object_model.get_articulation("link_3_to_end_tab")

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

    for joint in (
        mount_to_link_1,
        link_1_to_link_2,
        link_2_to_link_3,
        link_3_to_end_tab,
    ):
        ctx.check(
            f"{joint.name}_axis_is_planar",
            joint.axis == (0.0, -1.0, 0.0),
            f"{joint.name} axis should be (0, -1, 0), got {joint.axis}",
        )

    ctx.expect_contact(mount_plate, link_1, contact_tol=0.0005, name="mount_plate_contacts_link_1")
    ctx.expect_contact(link_1, link_2, contact_tol=0.0005, name="link_1_contacts_link_2")
    ctx.expect_contact(link_2, link_3, contact_tol=0.0005, name="link_2_contacts_link_3")
    ctx.expect_contact(link_3, end_tab, contact_tol=0.0005, name="link_3_contacts_end_tab")
    ctx.expect_origin_gap(link_1, mount_plate, axis="x", min_gap=0.015, name="chain_starts_beyond_plate")

    closed_tip = ctx.part_world_position(end_tab)
    with ctx.pose(
        {
            mount_to_link_1: 0.85,
            link_1_to_link_2: 0.70,
            link_2_to_link_3: 0.55,
            link_3_to_end_tab: 0.35,
        }
    ):
        folded_tip = ctx.part_world_position(end_tab)

    ctx.check(
        "positive_pose_lifts_distal_tab",
        closed_tip is not None
        and folded_tip is not None
        and folded_tip[2] > closed_tip[2] + 0.050
        and folded_tip[0] < closed_tip[0] - 0.030,
        f"closed={closed_tip}, folded={folded_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
