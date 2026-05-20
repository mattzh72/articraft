from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOTAL_YOKE_WIDTH = 0.034
EAR_LENGTH = 0.010
INNER_GAP_WIDTH = 0.014
BARREL_RADIUS = 0.008
LINK_THICKNESS = 0.006
EAR_CENTER_Y = INNER_GAP_WIDTH / 2.0 + EAR_LENGTH / 2.0
TONGUE_START_X = BARREL_RADIUS
TONGUE_LENGTH = 0.020
TONGUE_WIDTH = INNER_GAP_WIDTH
SPINE_WIDTH = 0.018
ARM_WIDTH = 0.010

LINK_1_LENGTH = 0.105
LINK_2_LENGTH = 0.094
LINK_3_LENGTH = 0.078

JOINT_2_Z = -0.012
JOINT_3_Z = 0.012
JOINT_4_Z = -0.010


def _barrel(x_pos: float, y_pos: float, z_pos: float, radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_pos, z_pos)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y_pos, 0.0))
    )


def _double_ear_yoke(x_pos: float, z_pos: float) -> cq.Workplane:
    left_ear = _barrel(x_pos, -EAR_CENTER_Y, z_pos, BARREL_RADIUS, EAR_LENGTH)
    right_ear = _barrel(x_pos, EAR_CENTER_Y, z_pos, BARREL_RADIUS, EAR_LENGTH)
    return left_ear.union(right_ear)


def _box_span(x0: float, x1: float, y_size: float, z_size: float, z_center: float, y_center: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, y_size, z_size).translate(((x0 + x1) / 2.0, y_center, z_center))


def _strap_loft(
    x0: float,
    z0: float,
    x1: float,
    z1: float,
    width0: float,
    width1: float,
    thickness0: float,
    thickness1: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(x0, 0.0, z0))
        .rect(width0, thickness0)
        .workplane(offset=x1 - x0)
        .center(0.0, z1 - z0)
        .rect(width1, thickness1)
        .loft(combine=True)
    )


def _offset_strap(
    x0: float,
    z0: float,
    x1: float,
    z1: float,
    width0: float,
    width1: float,
    thickness0: float,
    thickness1: float,
    y_pos: float,
) -> cq.Workplane:
    return _strap_loft(x0, z0, x1, z1, width0, width1, thickness0, thickness1).translate((0.0, y_pos, 0.0))


def _make_base_mount_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.150, 0.092, 0.008)
        .translate((-0.062, 0.0, -0.014))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.040, -0.028), (-0.040, 0.028), (0.020, -0.028), (0.020, 0.028)])
        .hole(0.008)
    )
    back_pad = cq.Workplane("XY").box(0.046, 0.058, 0.010).translate((-0.035, 0.0, -0.010))
    root_web = _box_span(-0.030, -0.004, 0.024, 0.010, -0.007)
    left_cheek = _box_span(-0.028, -0.008, EAR_LENGTH, 0.010, -0.004, -EAR_CENTER_Y)
    right_cheek = _box_span(-0.028, -0.008, EAR_LENGTH, 0.010, -0.004, EAR_CENTER_Y)
    return (
        plate.union(back_pad)
        .union(root_web)
        .union(left_cheek)
        .union(right_cheek)
        .union(_double_ear_yoke(0.0, 0.0))
    )


def _make_link_shape(length: float, distal_joint_z: float) -> cq.Workplane:
    mid_z = distal_joint_z * 0.35
    rail_z = distal_joint_z * 0.82

    tongue = _box_span(TONGUE_START_X, TONGUE_START_X + TONGUE_LENGTH, TONGUE_WIDTH, LINK_THICKNESS, 0.0)
    main_spine = _strap_loft(
        TONGUE_START_X + 0.012,
        0.0,
        length - 0.030,
        mid_z,
        SPINE_WIDTH,
        SPINE_WIDTH,
        LINK_THICKNESS,
        LINK_THICKNESS,
    )
    transition = _box_span(length - 0.034, length - 0.020, 0.022, LINK_THICKNESS, distal_joint_z * 0.55)
    left_rail = _offset_strap(
        length - 0.026,
        distal_joint_z * 0.58,
        length - BARREL_RADIUS,
        rail_z,
        ARM_WIDTH,
        ARM_WIDTH,
        LINK_THICKNESS,
        LINK_THICKNESS,
        -EAR_CENTER_Y,
    )
    right_rail = _offset_strap(
        length - 0.026,
        distal_joint_z * 0.58,
        length - BARREL_RADIUS,
        rail_z,
        ARM_WIDTH,
        ARM_WIDTH,
        LINK_THICKNESS,
        LINK_THICKNESS,
        EAR_CENTER_Y,
    )
    return (
        tongue.union(main_spine)
        .union(transition)
        .union(left_rail)
        .union(right_rail)
        .union(_double_ear_yoke(length, distal_joint_z))
    )


def _make_platform_bracket_shape() -> cq.Workplane:
    tongue = _box_span(TONGUE_START_X, TONGUE_START_X + TONGUE_LENGTH, TONGUE_WIDTH, LINK_THICKNESS, 0.0)
    arm = _strap_loft(TONGUE_START_X + 0.012, 0.0, 0.040, 0.011, 0.016, 0.018, LINK_THICKNESS, LINK_THICKNESS)
    support_block = _box_span(0.030, 0.046, 0.022, 0.010, 0.010)
    platform = _box_span(0.042, 0.090, 0.036, 0.005, 0.018)
    front_lip = _box_span(0.088, 0.092, 0.036, 0.012, 0.021)
    left_gusset = _box_span(0.034, 0.052, 0.006, 0.014, 0.010, -0.009)
    right_gusset = _box_span(0.034, 0.052, 0.006, 0.014, 0.010, 0.009)
    return (
        tongue.union(arm)
        .union(arm)
        .union(support_block)
        .union(platform)
        .union(front_lip)
        .union(left_gusset)
        .union(right_gusset)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldout_arm")

    mount_finish = model.material("mount_finish", rgba=(0.18, 0.20, 0.22, 1.0))
    link_finish = model.material("link_finish", rgba=(0.71, 0.74, 0.78, 1.0))
    bracket_finish = model.material("bracket_finish", rgba=(0.30, 0.33, 0.36, 1.0))

    base_mount = model.part("base_mount")
    base_mount.visual(mesh_from_cadquery(_make_base_mount_shape(), "base_mount"), material=mount_finish)
    base_mount.inertial = Inertial.from_geometry(
        Box((0.150, 0.092, 0.026)),
        mass=1.3,
        origin=Origin(xyz=(-0.062, 0.0, -0.010)),
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(mesh_from_cadquery(_make_link_shape(LINK_1_LENGTH, JOINT_2_Z), "proximal_link"), material=link_finish)
    proximal_link.inertial = Inertial.from_geometry(
        Box((LINK_1_LENGTH + 0.014, TOTAL_YOKE_WIDTH, 0.026)),
        mass=0.25,
        origin=Origin(xyz=(LINK_1_LENGTH / 2.0, 0.0, JOINT_2_Z * 0.35)),
    )

    middle_link = model.part("middle_link")
    middle_link.visual(mesh_from_cadquery(_make_link_shape(LINK_2_LENGTH, JOINT_3_Z), "middle_link"), material=link_finish)
    middle_link.inertial = Inertial.from_geometry(
        Box((LINK_2_LENGTH + 0.014, TOTAL_YOKE_WIDTH, 0.026)),
        mass=0.21,
        origin=Origin(xyz=(LINK_2_LENGTH / 2.0, 0.0, JOINT_3_Z * 0.35)),
    )

    distal_link = model.part("distal_link")
    distal_link.visual(mesh_from_cadquery(_make_link_shape(LINK_3_LENGTH, JOINT_4_Z), "distal_link"), material=link_finish)
    distal_link.inertial = Inertial.from_geometry(
        Box((LINK_3_LENGTH + 0.014, TOTAL_YOKE_WIDTH, 0.026)),
        mass=0.17,
        origin=Origin(xyz=(LINK_3_LENGTH / 2.0, 0.0, JOINT_4_Z * 0.35)),
    )

    platform_bracket = model.part("platform_bracket")
    platform_bracket.visual(mesh_from_cadquery(_make_platform_bracket_shape(), "platform_bracket"), material=bracket_finish)
    platform_bracket.inertial = Inertial.from_geometry(
        Box((0.080, 0.040, 0.032)),
        mass=0.14,
        origin=Origin(xyz=(0.044, 0.0, 0.016)),
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base_mount,
        child=proximal_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.30, effort=18.0, velocity=1.7),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=middle_link,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, JOINT_2_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.45, effort=14.0, velocity=1.9),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=distal_link,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, JOINT_3_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.45, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "distal_to_platform",
        ArticulationType.REVOLUTE,
        parent=distal_link,
        child=platform_bracket,
        origin=Origin(xyz=(LINK_3_LENGTH, 0.0, JOINT_4_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.20, effort=10.0, velocity=2.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_mount = object_model.get_part("base_mount")
    proximal_link = object_model.get_part("proximal_link")
    middle_link = object_model.get_part("middle_link")
    distal_link = object_model.get_part("distal_link")
    platform_bracket = object_model.get_part("platform_bracket")
    joint_1 = object_model.get_articulation("base_to_proximal")
    joint_2 = object_model.get_articulation("proximal_to_middle")
    joint_3 = object_model.get_articulation("middle_to_distal")
    joint_4 = object_model.get_articulation("distal_to_platform")

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
        "all four pitch joints exist and share one plane axis",
        all(joint.axis == (0.0, -1.0, 0.0) for joint in (joint_1, joint_2, joint_3, joint_4)),
        details="Every revolute axis should be parallel to -Y so the chain bends in the same XZ plane.",
    )

    ctx.expect_contact(base_mount, proximal_link, contact_tol=0.001, name="base hinge remains physically assembled")
    ctx.expect_contact(proximal_link, middle_link, contact_tol=0.001, name="proximal-middle hinge remains physically assembled")
    ctx.expect_contact(middle_link, distal_link, contact_tol=0.001, name="middle-distal hinge remains physically assembled")
    ctx.expect_contact(distal_link, platform_bracket, contact_tol=0.001, name="distal-platform hinge remains physically assembled")
    ctx.expect_origin_gap(platform_bracket, base_mount, axis="x", min_gap=0.22, name="platform projects forward from mounting plate")

    with ctx.pose(
        {
            joint_1: 0.70,
            joint_2: 0.55,
            joint_3: 0.42,
            joint_4: 0.35,
        }
    ):
        ctx.expect_contact(base_mount, proximal_link, contact_tol=0.001, name="base hinge stays assembled while folded")
        ctx.expect_contact(proximal_link, middle_link, contact_tol=0.001, name="proximal hinge stays assembled while folded")
        ctx.expect_contact(middle_link, distal_link, contact_tol=0.001, name="middle hinge stays assembled while folded")
        ctx.expect_contact(distal_link, platform_bracket, contact_tol=0.001, name="distal hinge stays assembled while folded")
        ctx.expect_origin_gap(platform_bracket, base_mount, axis="z", min_gap=0.07, name="positive joint motion lifts the distal bracket")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
