from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import ArticulatedObject, ArticulationType, Box, Cylinder, Inertial, MotionLimits, Origin, TestContext, TestReport


PLATE_LENGTH = 0.145
PLATE_WIDTH = 0.088
PLATE_THICKNESS = 0.008
PLATE_CENTER_X = -0.072
PLATE_BOTTOM_Z = -0.026

BODY_WIDTH = 0.010
EAR_WIDTH = 0.008
FORK_OFFSET = (BODY_WIDTH * 0.5) + (EAR_WIDTH * 0.5)
KNUCKLE_RADIUS = 0.0085
WEB_HEIGHT = 0.014
END_PAD_RADIUS = 0.009
CHEEK_WIDTH = 0.004
CHEEK_CENTER_Y = 0.007
HALF_PI = 1.5707963267948966

PRIMARY_LINK_LENGTH = 0.082
MIDDLE_LINK_LENGTH = 0.070
DISTAL_BODY_LENGTH = 0.040
DISTAL_TAB_LENGTH = 0.018


def _add_box_visual(part, size: tuple[float, float, float], xyz: tuple[float, float, float], material: str, name: str) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_y_cylinder_visual(part, radius: float, length: float, xyz: tuple[float, float, float], material: str, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(HALF_PI, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_mounting_plate_visuals(part) -> None:
    _add_box_visual(
        part,
        (PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS),
        (PLATE_CENTER_X, 0.0, PLATE_BOTTOM_Z + (PLATE_THICKNESS / 2.0)),
        "base_steel",
        "mount_plate",
    )
    _add_box_visual(part, (0.040, 0.040, 0.014), (-0.030, 0.0, -0.011), "base_steel", "mount_pedestal")
    _add_box_visual(part, (0.010, 0.018, 0.010), (-0.023, 0.0, -0.002), "base_steel", "mount_bridge")
    _add_box_visual(part, (0.018, CHEEK_WIDTH, 0.015), (-0.009, -CHEEK_CENTER_Y, 0.0), "base_steel", "mount_left_cheek")
    _add_box_visual(part, (0.018, CHEEK_WIDTH, 0.015), (-0.009, CHEEK_CENTER_Y, 0.0), "base_steel", "mount_right_cheek")
    _add_y_cylinder_visual(part, KNUCKLE_RADIUS, CHEEK_WIDTH, (0.0, -CHEEK_CENTER_Y, 0.0), "base_steel", "mount_left_cap")
    _add_y_cylinder_visual(part, KNUCKLE_RADIUS, CHEEK_WIDTH, (0.0, CHEEK_CENTER_Y, 0.0), "base_steel", "mount_right_cap")


def _add_link_visuals(part, length: float, material: str, prefix: str) -> None:
    beam_start = 0.012
    beam_end = length - 0.018
    _add_box_visual(
        part,
        (beam_end - beam_start, BODY_WIDTH, WEB_HEIGHT),
        ((beam_start + beam_end) / 2.0, 0.0, 0.0),
        material,
        f"{prefix}_beam",
    )
    _add_y_cylinder_visual(part, KNUCKLE_RADIUS, BODY_WIDTH, (0.0, 0.0, 0.0), material, f"{prefix}_root_barrel")
    _add_box_visual(part, (0.018, BODY_WIDTH, WEB_HEIGHT), (0.009, 0.0, 0.0), material, f"{prefix}_root_blend")
    _add_box_visual(part, (0.010, 0.018, 0.010), (length - 0.023, 0.0, 0.0), material, f"{prefix}_fork_bridge")
    _add_box_visual(part, (0.018, CHEEK_WIDTH, 0.012), (length - 0.009, -CHEEK_CENTER_Y, 0.0), material, f"{prefix}_left_cheek")
    _add_box_visual(part, (0.018, CHEEK_WIDTH, 0.012), (length - 0.009, CHEEK_CENTER_Y, 0.0), material, f"{prefix}_right_cheek")
    _add_y_cylinder_visual(part, KNUCKLE_RADIUS, CHEEK_WIDTH, (length, -CHEEK_CENTER_Y, 0.0), material, f"{prefix}_left_cap")
    _add_y_cylinder_visual(part, KNUCKLE_RADIUS, CHEEK_WIDTH, (length, CHEEK_CENTER_Y, 0.0), material, f"{prefix}_right_cap")


def _add_distal_visuals(part) -> None:
    beam_start = 0.012
    beam_end = DISTAL_BODY_LENGTH - 0.006
    _add_box_visual(
        part,
        (beam_end - beam_start, BODY_WIDTH, WEB_HEIGHT * 0.85),
        ((beam_start + beam_end) / 2.0, 0.0, 0.0),
        "distal_dark",
        "distal_beam",
    )
    _add_y_cylinder_visual(part, KNUCKLE_RADIUS, BODY_WIDTH, (0.0, 0.0, 0.0), "distal_dark", "distal_root_barrel")
    _add_box_visual(part, (0.018, BODY_WIDTH, WEB_HEIGHT * 0.85), (0.009, 0.0, 0.0), "distal_dark", "distal_root_blend")
    _add_y_cylinder_visual(part, 0.008, BODY_WIDTH, (DISTAL_BODY_LENGTH - 0.006, 0.0, 0.0), "distal_dark", "distal_head")
    _add_box_visual(
        part,
        (DISTAL_TAB_LENGTH, BODY_WIDTH, 0.008),
        (DISTAL_BODY_LENGTH + (DISTAL_TAB_LENGTH / 2.0), 0.0, 0.0),
        "distal_dark",
        "distal_tab",
    )


def _cylinder_y(radius: float, width: float, *, x: float, y: float, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(width / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )


def _box(length: float, width: float, height: float, *, x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate((x, y, z))


def _mounting_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .rect(PLATE_LENGTH, PLATE_WIDTH)
        .extrude(PLATE_THICKNESS)
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.040, -0.024),
                (-0.040, 0.024),
                (0.028, -0.024),
                (0.028, 0.024),
            ]
        )
        .slot2D(0.018, 0.008, angle=0.0)
        .cutThruAll()
        .edges("|Z")
        .fillet(0.005)
        .translate((PLATE_CENTER_X, 0.0, PLATE_BOTTOM_Z + (PLATE_THICKNESS / 2.0)))
    )

    pedestal = _box(0.040, 0.040, 0.014, x=-0.030, y=0.0, z=-0.011)
    yoke = _box(0.014, 0.024, 0.010, x=-0.022, y=0.0, z=-0.002)
    left_arm = _box(0.028, EAR_WIDTH, 0.015, x=-0.014, y=-FORK_OFFSET, z=0.0)
    right_arm = _box(0.028, EAR_WIDTH, 0.015, x=-0.014, y=FORK_OFFSET, z=0.0)
    left_cap = _cylinder_y(KNUCKLE_RADIUS, EAR_WIDTH, x=0.0, y=-FORK_OFFSET)
    right_cap = _cylinder_y(KNUCKLE_RADIUS, EAR_WIDTH, x=0.0, y=FORK_OFFSET)

    return plate.union(pedestal).union(yoke).union(left_arm).union(right_arm).union(left_cap).union(right_cap)


def _lever_link_shape(length: float) -> cq.Workplane:
    beam_start = 0.012
    yoke_center_x = length - 0.022
    yoke_length = 0.012
    beam_end = yoke_center_x - (yoke_length / 2.0)
    body = _box(beam_end - beam_start, BODY_WIDTH, WEB_HEIGHT, x=(beam_start + beam_end) / 2.0, y=0.0, z=0.0)
    root_knuckle = _cylinder_y(KNUCKLE_RADIUS, BODY_WIDTH, x=0.0, y=0.0)
    root_blend = _box(0.018, BODY_WIDTH, WEB_HEIGHT, x=0.009, y=0.0, z=0.0)
    yoke = _box(yoke_length, 0.024, 0.010, x=yoke_center_x, y=0.0, z=0.0)
    left_arm = _box(0.016, EAR_WIDTH, 0.012, x=length - 0.008, y=-FORK_OFFSET, z=0.0)
    right_arm = _box(0.016, EAR_WIDTH, 0.012, x=length - 0.008, y=FORK_OFFSET, z=0.0)
    left_cap = _cylinder_y(KNUCKLE_RADIUS, EAR_WIDTH, x=length, y=-FORK_OFFSET)
    right_cap = _cylinder_y(KNUCKLE_RADIUS, EAR_WIDTH, x=length, y=FORK_OFFSET)

    return (
        body.union(root_knuckle)
        .union(root_blend)
        .union(yoke)
        .union(left_arm)
        .union(right_arm)
        .union(left_cap)
        .union(right_cap)
    )


def _distal_link_shape() -> cq.Workplane:
    beam_start = 0.012
    beam_end = DISTAL_BODY_LENGTH - 0.012
    body = _box(
        beam_end - beam_start,
        BODY_WIDTH,
        WEB_HEIGHT * 0.85,
        x=(beam_start + beam_end) / 2.0,
        y=0.0,
        z=0.0,
    )
    root_knuckle = _cylinder_y(KNUCKLE_RADIUS, BODY_WIDTH, x=0.0, y=0.0)
    root_blend = _box(0.016, BODY_WIDTH, WEB_HEIGHT * 0.85, x=0.008, y=0.0, z=0.0)
    compact_head = _cylinder_y(0.008, BODY_WIDTH, x=DISTAL_BODY_LENGTH - 0.006, y=0.0)
    end_tab = _box(
        DISTAL_TAB_LENGTH,
        BODY_WIDTH,
        0.008,
        x=DISTAL_BODY_LENGTH + (DISTAL_TAB_LENGTH / 2.0),
        y=0.0,
        z=0.0,
    )
    tab_hole = _cylinder_y(
        0.003,
        BODY_WIDTH * 1.4,
        x=DISTAL_BODY_LENGTH + (DISTAL_TAB_LENGTH * 0.52),
        y=0.0,
    )

    return body.union(root_knuckle).union(root_blend).union(compact_head).union(end_tab).cut(tab_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_lever_chain")

    model.material("base_steel", rgba=(0.32, 0.35, 0.39, 1.0))
    model.material("link_steel", rgba=(0.61, 0.64, 0.68, 1.0))
    model.material("distal_dark", rgba=(0.24, 0.26, 0.29, 1.0))

    mounting_plate = model.part("mounting_plate")
    _add_mounting_plate_visuals(mounting_plate)
    mounting_plate.inertial = Inertial.from_geometry(
        Box((0.150, 0.090, 0.040)),
        mass=0.95,
        origin=Origin(xyz=(-0.060, 0.0, -0.012)),
    )

    primary_link = model.part("primary_link")
    _add_link_visuals(primary_link, PRIMARY_LINK_LENGTH, "link_steel", "primary")
    primary_link.inertial = Inertial.from_geometry(
        Box((PRIMARY_LINK_LENGTH + 0.004, 0.026, 0.020)),
        mass=0.22,
        origin=Origin(xyz=(PRIMARY_LINK_LENGTH * 0.5, 0.0, 0.0)),
    )

    middle_link = model.part("middle_link")
    _add_link_visuals(middle_link, MIDDLE_LINK_LENGTH, "link_steel", "middle")
    middle_link.inertial = Inertial.from_geometry(
        Box((MIDDLE_LINK_LENGTH + 0.004, 0.026, 0.020)),
        mass=0.16,
        origin=Origin(xyz=(MIDDLE_LINK_LENGTH * 0.5, 0.0, 0.0)),
    )

    distal_link = model.part("distal_link")
    _add_distal_visuals(distal_link)
    distal_link.inertial = Inertial.from_geometry(
        Box((DISTAL_BODY_LENGTH + DISTAL_TAB_LENGTH, 0.014, 0.018)),
        mass=0.10,
        origin=Origin(xyz=((DISTAL_BODY_LENGTH + DISTAL_TAB_LENGTH) * 0.45, 0.0, 0.0)),
    )

    model.articulation(
        "mount_to_primary",
        ArticulationType.REVOLUTE,
        parent=mounting_plate,
        child=primary_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=30.0, velocity=1.4),
    )
    model.articulation(
        "primary_to_middle",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=middle_link,
        origin=Origin(xyz=(PRIMARY_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.10, upper=1.35, effort=22.0, velocity=1.6),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=distal_link,
        origin=Origin(xyz=(MIDDLE_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.15, upper=1.10, effort=16.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mounting_plate = object_model.get_part("mounting_plate")
    primary_link = object_model.get_part("primary_link")
    middle_link = object_model.get_part("middle_link")
    distal_link = object_model.get_part("distal_link")

    mount_to_primary = object_model.get_articulation("mount_to_primary")
    primary_to_middle = object_model.get_articulation("primary_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

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
        "all_three_pitch_joints_share_one_bending_axis",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, -1.0, 0.0)
            for joint in (mount_to_primary, primary_to_middle, middle_to_distal)
        ),
        details="Expected three revolute joints all bending in the same XZ plane about -Y.",
    )

    ctx.expect_contact(mounting_plate, primary_link, contact_tol=0.0005, name="mount_contacts_primary_knuckle")
    ctx.expect_contact(primary_link, middle_link, contact_tol=0.0005, name="primary_contacts_middle_knuckle")
    ctx.expect_contact(middle_link, distal_link, contact_tol=0.0005, name="middle_contacts_distal_knuckle")
    ctx.expect_origin_gap(
        distal_link,
        mounting_plate,
        axis="x",
        min_gap=0.145,
        max_gap=0.165,
        name="straight_chain_reaches_outboard",
    )

    with ctx.pose(
        {
            mount_to_primary: 0.85,
            primary_to_middle: 0.65,
            middle_to_distal: 0.40,
        }
    ):
        ctx.expect_origin_gap(
            distal_link,
            mounting_plate,
            axis="z",
            min_gap=0.055,
            name="opened_chain_lifts_the_distal_link",
        )
        ctx.expect_origin_distance(
            distal_link,
            mounting_plate,
            axes="y",
            max_dist=0.001,
            name="opened_chain_stays_in_one_plane",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
