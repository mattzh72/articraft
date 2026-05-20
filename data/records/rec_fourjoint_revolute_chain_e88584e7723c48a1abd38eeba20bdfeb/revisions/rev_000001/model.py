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


LINK_LENGTHS = (0.31, 0.27, 0.33, 0.28, 0.30)

BEAM_WIDTH = 0.018
BEAM_DEPTH = 0.046
WALL_THICKNESS = 0.0035
INNER_END_CAP = 0.006
CAP_STRAP_THICKNESS = 0.003
CAP_STRAP_LENGTH = 0.030

PIN_RADIUS = 0.0075
BARREL_RADIUS = 0.016
CENTER_LUG_WIDTH = 0.012
CHEEK_THICKNESS = 0.008
PIN_LENGTH = CENTER_LUG_WIDTH + 2.0 * CHEEK_THICKNESS
LUG_LENGTH = PIN_LENGTH
CHEEK_CENTER_Y = LUG_LENGTH / 2.0 + CHEEK_THICKNESS / 2.0
BRIDGE_LENGTH = 0.022
BRIDGE_WIDTH = 0.026

PROX_ARM_LENGTH = 0.052
DIST_TONGUE_LENGTH = 0.046
TONGUE_DEPTH = BEAM_DEPTH * 0.82
BRIDGE_DEPTH = TONGUE_DEPTH * 0.72

PAD_SIZE = 0.064
PAD_THICKNESS = 0.012
PAD_STEM_LENGTH = 0.028
PAD_STEM_WIDTH = 0.014
PAD_STEM_DEPTH = 0.026

FOOT_LENGTH = 0.17
FOOT_WIDTH = 0.11
FOOT_THICKNESS = 0.020
FOOT_DROP = 0.075
TOWER_LENGTH = 0.054
TOWER_WIDTH = 0.028
TOWER_HEIGHT = FOOT_DROP - FOOT_THICKNESS / 2.0
RIB_THICKNESS = 0.007
ROOT_BLOCK_LENGTH = 0.060

JOINT_LIMIT = 1.00


def _rect_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _barrel(center_x: float, center_y: float, radius: float, length_y: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, 0.0)
        .circle(radius)
        .extrude(length_y / 2.0, both=True)
        .translate((0.0, center_y, 0.0))
    )


def _tube(
    center_x: float,
    center_y: float,
    outer_radius: float,
    inner_radius: float,
    length_y: float,
) -> cq.Workplane:
    outer = _barrel(center_x, center_y, outer_radius, length_y)
    cutter = _barrel(center_x, center_y, inner_radius, length_y + 0.004)
    return outer.cut(cutter)


def _side_rib(y_center: float) -> cq.Workplane:
    rib_profile = [
        (-0.048, -FOOT_DROP + FOOT_THICKNESS / 2.0),
        (0.038, -FOOT_DROP + FOOT_THICKNESS / 2.0),
        (0.046, -0.008),
        (-0.006, -0.008),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(rib_profile)
        .close()
        .extrude(RIB_THICKNESS / 2.0, both=True)
        .translate((0.0, y_center, 0.0))
    )


def _beam_shell(x_start: float, x_end: float) -> cq.Workplane:
    shell = _rect_box(
        (x_end - x_start, BEAM_WIDTH, BEAM_DEPTH),
        ((x_start + x_end) / 2.0, 0.0, 0.0),
    )
    inner_length = (x_end - x_start) - 2.0 * INNER_END_CAP
    if inner_length > 0.0:
        shell = shell.cut(
            _rect_box(
                (
                    inner_length,
                    BEAM_WIDTH - 2.0 * WALL_THICKNESS,
                    BEAM_DEPTH - 2.0 * WALL_THICKNESS,
                ),
                ((x_start + x_end) / 2.0, 0.0, 0.0),
            )
        )
    strap_z = BEAM_DEPTH / 2.0 - CAP_STRAP_THICKNESS / 2.0 + 0.0003
    for x_center in (
        x_start + CAP_STRAP_LENGTH / 2.0,
        x_end - CAP_STRAP_LENGTH / 2.0,
    ):
        shell = shell.union(
            _rect_box(
                (CAP_STRAP_LENGTH, BEAM_WIDTH + 0.004, CAP_STRAP_THICKNESS),
                (x_center, 0.0, strap_z),
            )
        )
        shell = shell.union(
            _rect_box(
                (CAP_STRAP_LENGTH, BEAM_WIDTH + 0.004, CAP_STRAP_THICKNESS),
                (x_center, 0.0, -strap_z),
            )
        )
    return shell


def _proximal_clevis() -> cq.Workplane:
    cheeks = None
    for y_center in (-CHEEK_CENTER_Y, CHEEK_CENTER_Y):
        cheek_barrel = _tube(0.0, y_center, BARREL_RADIUS, PIN_RADIUS, CHEEK_THICKNESS)
        cheek_arm = _rect_box(
            (PROX_ARM_LENGTH, CHEEK_THICKNESS, TONGUE_DEPTH),
            (PROX_ARM_LENGTH / 2.0, y_center, 0.0),
        )
        cheek_cap = _rect_box(
            (0.028, CHEEK_THICKNESS, CAP_STRAP_THICKNESS),
            (0.026, y_center, BEAM_DEPTH / 2.0 - CAP_STRAP_THICKNESS / 2.0 + 0.0003),
        )
        cheek_cap_bottom = _rect_box(
            (0.028, CHEEK_THICKNESS, CAP_STRAP_THICKNESS),
            (0.026, y_center, -BEAM_DEPTH / 2.0 + CAP_STRAP_THICKNESS / 2.0 - 0.0003),
        )
        feature = cheek_barrel.union(cheek_arm).union(cheek_cap).union(cheek_cap_bottom)
        cheeks = feature if cheeks is None else cheeks.union(feature)
    bridge = _rect_box(
        (BRIDGE_LENGTH, BRIDGE_WIDTH, BRIDGE_DEPTH),
        (PROX_ARM_LENGTH - BRIDGE_LENGTH / 2.0 + 0.004, 0.0, 0.0),
    )
    top_bridge_plate = _rect_box(
        (BRIDGE_LENGTH + 0.006, BRIDGE_WIDTH, CAP_STRAP_THICKNESS),
        (
            PROX_ARM_LENGTH - BRIDGE_LENGTH / 2.0 + 0.004,
            0.0,
            BEAM_DEPTH / 2.0 - CAP_STRAP_THICKNESS / 2.0 + 0.0003,
        ),
    )
    bottom_bridge_plate = _rect_box(
        (BRIDGE_LENGTH + 0.006, BRIDGE_WIDTH, CAP_STRAP_THICKNESS),
        (
            PROX_ARM_LENGTH - BRIDGE_LENGTH / 2.0 + 0.004,
            0.0,
            -BEAM_DEPTH / 2.0 + CAP_STRAP_THICKNESS / 2.0 - 0.0003,
        ),
    )
    return cheeks.union(bridge).union(top_bridge_plate).union(bottom_bridge_plate)


def _distal_lug(length: float) -> cq.Workplane:
    tongue = _rect_box(
        (DIST_TONGUE_LENGTH, CENTER_LUG_WIDTH, TONGUE_DEPTH),
        (length - DIST_TONGUE_LENGTH / 2.0, 0.0, 0.0),
    )
    barrel = _barrel(length, 0.0, PIN_RADIUS, LUG_LENGTH)
    top_plate = _rect_box(
        (0.024, CENTER_LUG_WIDTH + 0.002, CAP_STRAP_THICKNESS),
        (length - 0.020, 0.0, BEAM_DEPTH / 2.0 - CAP_STRAP_THICKNESS / 2.0 + 0.0003),
    )
    bottom_plate = _rect_box(
        (0.024, CENTER_LUG_WIDTH + 0.002, CAP_STRAP_THICKNESS),
        (length - 0.020, 0.0, -BEAM_DEPTH / 2.0 + CAP_STRAP_THICKNESS / 2.0 - 0.0003),
    )
    return tongue.union(barrel).union(top_plate).union(bottom_plate)


def _terminal_pad(length: float) -> cq.Workplane:
    stem = _rect_box(
        (PAD_STEM_LENGTH, PAD_STEM_WIDTH, PAD_STEM_DEPTH),
        (length + PAD_STEM_LENGTH / 2.0, 0.0, 0.0),
    )
    pad = _rect_box(
        (PAD_THICKNESS, PAD_SIZE, PAD_SIZE),
        (length + PAD_STEM_LENGTH + PAD_THICKNESS / 2.0, 0.0, 0.0),
    )
    backing_plate = _rect_box(
        (0.008, PAD_SIZE * 0.72, PAD_SIZE * 0.72),
        (length + PAD_STEM_LENGTH - 0.001, 0.0, 0.0),
    )
    return stem.union(backing_plate).union(pad)


def make_root_link(length: float) -> cq.Workplane:
    foot = _rect_box(
        (FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS),
        (0.0, 0.0, -FOOT_DROP + FOOT_THICKNESS / 2.0),
    )
    tower = _rect_box(
        (TOWER_LENGTH, TOWER_WIDTH, TOWER_HEIGHT),
        (0.014, 0.0, -TOWER_HEIGHT / 2.0),
    )
    root_block = _rect_box((ROOT_BLOCK_LENGTH, BEAM_WIDTH, TONGUE_DEPTH), (ROOT_BLOCK_LENGTH / 2.0, 0.0, 0.0))
    ribs = _side_rib(BEAM_WIDTH / 2.0 + 0.010).union(_side_rib(-BEAM_WIDTH / 2.0 - 0.010))
    beam = _beam_shell(0.040, length - 0.040)
    return foot.union(tower).union(root_block).union(ribs).union(beam).union(_distal_lug(length))


def make_link(length: float, *, terminal: bool = False) -> cq.Workplane:
    beam_end = length - 0.040 if not terminal else length - 0.010
    shape = _proximal_clevis().union(_beam_shell(0.040, beam_end))
    if terminal:
        return shape.union(_terminal_pad(length))
    return shape.union(_distal_lug(length))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serial_linkage")

    root_finish = model.material("root_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    link_finish = model.material("link_finish", rgba=(0.63, 0.66, 0.70, 1.0))
    terminal_finish = model.material("terminal_finish", rgba=(0.45, 0.47, 0.50, 1.0))

    root_shape = make_root_link(LINK_LENGTHS[0])
    root_link = model.part("root_link")
    root_link.visual(mesh_from_cadquery(root_shape, "root_link"), material=root_finish, name="root_link_body")
    root_link.inertial = Inertial.from_geometry(
        Box((LINK_LENGTHS[0] + FOOT_LENGTH * 0.55, FOOT_WIDTH, FOOT_DROP + BEAM_DEPTH)),
        mass=3.6,
        origin=Origin(xyz=((LINK_LENGTHS[0] - FOOT_LENGTH * 0.10) / 2.0, 0.0, -0.020)),
    )

    links = [root_link]
    for index, length in enumerate(LINK_LENGTHS[1:], start=2):
        part = model.part(f"link_{index}")
        shape = make_link(length, terminal=index == 5)
        material = terminal_finish if index == 5 else link_finish
        part.visual(
            mesh_from_cadquery(shape, f"link_{index}"),
            material=material,
            name=f"link_{index}_body",
        )
        inertial_length = length + (PAD_STEM_LENGTH + PAD_THICKNESS if index == 5 else 0.0)
        inertial_height = max(BEAM_DEPTH, PAD_SIZE if index == 5 else BEAM_DEPTH)
        part.inertial = Inertial.from_geometry(
            Box((inertial_length, 0.036, inertial_height)),
            mass=1.35 if index < 5 else 1.10,
            origin=Origin(xyz=(inertial_length / 2.0, 0.0, 0.0)),
        )
        links.append(part)

    for joint_index, (parent, child, parent_length) in enumerate(
        zip(links[:-1], links[1:], LINK_LENGTHS[:-1]),
        start=1,
    ):
        model.articulation(
            f"joint_{joint_index}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=Origin(xyz=(parent_length, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=1.4,
                lower=-JOINT_LIMIT,
                upper=JOINT_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_link = object_model.get_part("root_link")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")
    link_5 = object_model.get_part("link_5")

    joint_1 = object_model.get_articulation("joint_1")
    joint_2 = object_model.get_articulation("joint_2")
    joint_3 = object_model.get_articulation("joint_3")
    joint_4 = object_model.get_articulation("joint_4")

    for link_a, link_b, reason in (
        (
            root_link,
            link_2,
            "installed revolute joint uses a coaxial pin-and-bushing abstraction at the first hinge",
        ),
        (
            link_2,
            link_3,
            "installed revolute joint uses a coaxial pin-and-bushing abstraction at the second hinge",
        ),
        (
            link_3,
            link_4,
            "installed revolute joint uses a coaxial pin-and-bushing abstraction at the third hinge",
        ),
        (
            link_4,
            link_5,
            "installed revolute joint uses a coaxial pin-and-bushing abstraction at the terminal hinge",
        ),
    ):
        ctx.allow_overlap(link_a, link_b, reason=reason)

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

    for name, link in (
        ("root_link_present", root_link),
        ("link_2_present", link_2),
        ("link_3_present", link_3),
        ("link_4_present", link_4),
        ("link_5_present", link_5),
    ):
        ctx.check(name, link is not None, "expected articulated chain part is missing")

    for name, joint in (
        ("joint_1_axis", joint_1),
        ("joint_2_axis", joint_2),
        ("joint_3_axis", joint_3),
        ("joint_4_axis", joint_4),
    ):
        axis_ok = tuple(round(value, 6) for value in joint.axis) == (0.0, 1.0, 0.0)
        limits = joint.motion_limits
        limits_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
        )
        ctx.check(name, axis_ok and limits_ok, "joint must be revolute about +Y/-Y with bilateral travel")

    for positive, negative, check_name in (
        (link_2, root_link, "root_to_link_2_contact"),
        (link_3, link_2, "link_2_to_link_3_contact"),
        (link_4, link_3, "link_3_to_link_4_contact"),
        (link_5, link_4, "link_4_to_link_5_contact"),
    ):
        ctx.expect_contact(positive, negative, contact_tol=0.0015, name=check_name)

    with ctx.pose({joint_1: 0.65, joint_2: -0.55, joint_3: 0.55, joint_4: -0.45}):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_clearance")

    with ctx.pose({joint_1: -0.60, joint_2: 0.50, joint_3: -0.55, joint_4: 0.45}):
        ctx.fail_if_parts_overlap_in_current_pose(name="reverse_folded_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
