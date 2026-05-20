from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LINK_PITCH = 0.24
JOINT_OFFSET = 0.042
BAR_REACH = LINK_PITCH - JOINT_OFFSET
TERMINAL_BAR_LENGTH = 0.205
BAR_THICKNESS = 0.008
BAR_HEIGHT = 0.016
EYE_RADIUS = 0.010
PIN_RADIUS = 0.005
BLOCK_BODY_LENGTH = 0.022
BLOCK_CHEEK_LENGTH = 0.024
BLOCK_BODY_WIDTH = 0.026
BLOCK_BODY_HEIGHT = 0.024
BLOCK_CHEEK_THICKNESS = 0.006
BLOCK_CHEEK_CENTER_Y = BAR_THICKNESS / 2.0 + BLOCK_CHEEK_THICKNESS / 2.0
BLOCK_CHEEK_HEIGHT = 0.020
BLOCK_CHEEK_START = 0.026
ROOT_JOINT_HEIGHT = 0.105
JOINT_LIMIT = 0.85


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _make_root_link() -> cq.Workplane:
    return _box((BAR_REACH, BAR_THICKNESS, BAR_HEIGHT), (BAR_REACH / 2.0, 0.0, 0.0))


def _make_link_bar() -> cq.Workplane:
    eye = _cylinder_y(EYE_RADIUS, BAR_THICKNESS, (0.0, 0.0, 0.0))
    eye = eye.cut(_cylinder_y(PIN_RADIUS, BAR_THICKNESS + 0.004, (0.0, 0.0, 0.0)))
    body_length = BAR_REACH - EYE_RADIUS
    body = _box((body_length, BAR_THICKNESS, BAR_HEIGHT), (EYE_RADIUS + body_length / 2.0, 0.0, 0.0))
    return eye.union(body)


def _make_terminal_link() -> cq.Workplane:
    eye = _cylinder_y(EYE_RADIUS, BAR_THICKNESS, (0.0, 0.0, 0.0))
    eye = eye.cut(_cylinder_y(PIN_RADIUS, BAR_THICKNESS + 0.004, (0.0, 0.0, 0.0)))
    straight_length = TERMINAL_BAR_LENGTH - EYE_RADIUS - BAR_HEIGHT / 2.0
    body = _box((straight_length, BAR_THICKNESS, BAR_HEIGHT), (EYE_RADIUS + straight_length / 2.0, 0.0, 0.0))
    tip = _cylinder_y(BAR_HEIGHT / 2.0, BAR_THICKNESS, (EYE_RADIUS + straight_length, 0.0, 0.0))
    return eye.union(body).union(tip)


def _make_hinge_block() -> cq.Workplane:
    block = _box(
        (BLOCK_BODY_LENGTH, BLOCK_BODY_WIDTH, BLOCK_BODY_HEIGHT),
        (BLOCK_BODY_LENGTH / 2.0, 0.0, 0.0),
    )
    for sign in (-1.0, 1.0):
        block = block.union(
            _box(
                (BLOCK_CHEEK_LENGTH, BLOCK_CHEEK_THICKNESS, BAR_HEIGHT),
                (BLOCK_CHEEK_START + BLOCK_CHEEK_LENGTH / 2.0, sign * BLOCK_CHEEK_CENTER_Y, 0.0),
            )
        )
        block = block.union(
            _cylinder_y(
                EYE_RADIUS + 0.004,
                BLOCK_CHEEK_THICKNESS,
                (JOINT_OFFSET, sign * BLOCK_CHEEK_CENTER_Y, 0.0),
            )
        )
    return block


def _make_base_foot() -> cq.Workplane:
    foot_length = 0.18
    foot_width = 0.11
    foot_thickness = 0.018
    foot_center_z = -(ROOT_JOINT_HEIGHT - foot_thickness / 2.0)
    pedestal_height = ROOT_JOINT_HEIGHT - foot_thickness - BLOCK_BODY_HEIGHT / 2.0
    pedestal_center_z = foot_center_z + foot_thickness / 2.0 + pedestal_height / 2.0

    base = _box((foot_length, foot_width, foot_thickness), (-0.055, 0.0, foot_center_z))
    base = base.union(_box((0.048, 0.050, pedestal_height), (-0.040, 0.0, pedestal_center_z)))
    base = base.union(_box((0.028, 0.024, BAR_HEIGHT * 1.2), (-0.014, 0.0, 0.0)))

    for x_pos in (-0.105, -0.005):
        for y_pos in (-0.034, 0.034):
            base = base.cut(_cylinder_z(0.007, foot_thickness + 0.004, (x_pos, y_pos, foot_center_z)))

    return base


def _add_mesh_part(
    model: ArticulatedObject,
    part_name: str,
    shape: cq.Workplane,
    mesh_name: str,
    material_name: str,
):
    part = model.part(part_name)
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        origin=Origin(),
        material=material_name,
        name=f"{part_name}_shell",
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_bar_serial_linkage")

    model.material("base_black", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("bar_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("block_steel", rgba=(0.36, 0.39, 0.43, 1.0))

    base_foot = _add_mesh_part(model, "base_foot", _make_base_foot(), "base_foot", "base_black")
    link_1 = _add_mesh_part(model, "link_1", _make_root_link(), "link_1", "bar_aluminum")
    block_1 = _add_mesh_part(model, "block_1", _make_hinge_block(), "block_1", "block_steel")
    link_2 = _add_mesh_part(model, "link_2", _make_link_bar(), "link_2", "bar_aluminum")
    block_2 = _add_mesh_part(model, "block_2", _make_hinge_block(), "block_2", "block_steel")
    link_3 = _add_mesh_part(model, "link_3", _make_link_bar(), "link_3", "bar_aluminum")
    block_3 = _add_mesh_part(model, "block_3", _make_hinge_block(), "block_3", "block_steel")
    link_4 = _add_mesh_part(model, "link_4", _make_link_bar(), "link_4", "bar_aluminum")
    block_4 = _add_mesh_part(model, "block_4", _make_hinge_block(), "block_4", "block_steel")
    link_5 = _add_mesh_part(model, "link_5", _make_terminal_link(), "link_5", "bar_aluminum")

    model.articulation(
        "base_to_link_1",
        ArticulationType.FIXED,
        parent=base_foot,
        child=link_1,
        origin=Origin(),
    )

    model.articulation(
        "link_1_to_block_1",
        ArticulationType.FIXED,
        parent=link_1,
        child=block_1,
        origin=Origin(xyz=(BAR_REACH, 0.0, 0.0)),
    )
    model.articulation(
        "block_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=block_1,
        child=link_2,
        origin=Origin(xyz=(JOINT_OFFSET, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-JOINT_LIMIT, upper=JOINT_LIMIT),
    )

    model.articulation(
        "link_2_to_block_2",
        ArticulationType.FIXED,
        parent=link_2,
        child=block_2,
        origin=Origin(xyz=(BAR_REACH, 0.0, 0.0)),
    )
    model.articulation(
        "block_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=block_2,
        child=link_3,
        origin=Origin(xyz=(JOINT_OFFSET, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-JOINT_LIMIT, upper=JOINT_LIMIT),
    )

    model.articulation(
        "link_3_to_block_3",
        ArticulationType.FIXED,
        parent=link_3,
        child=block_3,
        origin=Origin(xyz=(BAR_REACH, 0.0, 0.0)),
    )
    model.articulation(
        "block_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent=block_3,
        child=link_4,
        origin=Origin(xyz=(JOINT_OFFSET, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-JOINT_LIMIT, upper=JOINT_LIMIT),
    )

    model.articulation(
        "link_4_to_block_4",
        ArticulationType.FIXED,
        parent=link_4,
        child=block_4,
        origin=Origin(xyz=(BAR_REACH, 0.0, 0.0)),
    )
    model.articulation(
        "block_4_to_link_5",
        ArticulationType.REVOLUTE,
        parent=block_4,
        child=link_5,
        origin=Origin(xyz=(JOINT_OFFSET, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-JOINT_LIMIT, upper=JOINT_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_foot = object_model.get_part("base_foot")
    links = [object_model.get_part(f"link_{index}") for index in range(1, 6)]
    blocks = [object_model.get_part(f"block_{index}") for index in range(1, 5)]
    fixed_joints = [
        object_model.get_articulation("base_to_link_1"),
        object_model.get_articulation("link_1_to_block_1"),
        object_model.get_articulation("link_2_to_block_2"),
        object_model.get_articulation("link_3_to_block_3"),
        object_model.get_articulation("link_4_to_block_4"),
    ]
    revolute_joints = [
        object_model.get_articulation("block_1_to_link_2"),
        object_model.get_articulation("block_2_to_link_3"),
        object_model.get_articulation("block_3_to_link_4"),
        object_model.get_articulation("block_4_to_link_5"),
    ]

    ctx.allow_overlap(
        base_foot,
        links[0],
        reason="The root bar is captured by the fixed base saddle at the mount interface.",
    )
    for index in range(4):
        ctx.allow_overlap(
            links[index],
            blocks[index],
            reason="Each hinge block is socket-mounted into the preceding bar as a fixed clevis carrier.",
        )
        ctx.allow_overlap(
            blocks[index],
            links[index + 1],
            reason="Each bar eye nests inside its hinge block clevis around the revolute axle axis.",
        )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("part_count", len(object_model.parts) == 10, f"Expected 10 parts, found {len(object_model.parts)}.")
    ctx.check(
        "articulation_count",
        len(object_model.articulations) == 9,
        f"Expected 9 articulations, found {len(object_model.articulations)}.",
    )

    for joint in fixed_joints:
        ctx.check(
            f"{joint.name}_fixed",
            joint.articulation_type == ArticulationType.FIXED,
            f"{joint.name} should be FIXED, found {joint.articulation_type}.",
        )

    for joint in revolute_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            f"{joint.name} should be REVOLUTE, found {joint.articulation_type}.",
        )
        ctx.check(
            f"{joint.name}_axis_parallel",
            tuple(round(value, 6) for value in joint.axis) == (0.0, 1.0, 0.0),
            f"{joint.name} axis should be parallel to +Y, found {joint.axis}.",
        )
        ctx.check(
            f"{joint.name}_limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and round(limits.lower, 3) == -round(limits.upper, 3),
            f"{joint.name} should have symmetric bounded limits, found {limits}.",
        )

    with ctx.pose({joint: 0.0 for joint in revolute_joints}):
        ctx.expect_contact(base_foot, links[0], name="base_to_root_contact")

        for index in range(4):
            ctx.expect_contact(links[index], blocks[index], name=f"link_{index + 1}_to_block_{index + 1}_contact")
            ctx.expect_contact(blocks[index], links[index + 1], name=f"block_{index + 1}_to_link_{index + 2}_contact")
            ctx.expect_origin_gap(
                blocks[index],
                links[index],
                axis="x",
                min_gap=BAR_REACH - 0.001,
                max_gap=BAR_REACH + 0.001,
                name=f"block_{index + 1}_mount_offset",
            )
            ctx.expect_origin_gap(
                links[index + 1],
                links[index],
                axis="x",
                min_gap=LINK_PITCH - 0.001,
                max_gap=LINK_PITCH + 0.001,
                name=f"link_{index + 1}_to_link_{index + 2}_pitch",
            )
            if index < 3:
                ctx.expect_origin_gap(
                    blocks[index + 1],
                    blocks[index],
                    axis="x",
                    min_gap=LINK_PITCH - 0.001,
                    max_gap=LINK_PITCH + 0.001,
                    name=f"block_{index + 1}_to_block_{index + 2}_pitch",
                )
            ctx.expect_origin_gap(
                links[index + 1],
                links[index],
                axis="z",
                min_gap=-0.0005,
                max_gap=0.0005,
                name=f"link_{index + 1}_to_link_{index + 2}_coplanar",
            )

        ctx.expect_origin_gap(
            links[4],
            links[0],
            axis="x",
            min_gap=4.0 * LINK_PITCH - 0.002,
            max_gap=4.0 * LINK_PITCH + 0.002,
            name="overall_chain_pitch",
        )

    for index, joint in enumerate(revolute_joints):
        limits = joint.motion_limits
        assert limits is not None
        assert limits.lower is not None
        assert limits.upper is not None

        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            ctx.expect_contact(blocks[index], links[index + 1], name=f"{joint.name}_lower_contact")

        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
            ctx.expect_contact(blocks[index], links[index + 1], name=f"{joint.name}_upper_contact")

    with ctx.pose(
        {
            revolute_joints[0]: 0.55,
            revolute_joints[1]: -0.70,
            revolute_joints[2]: 0.65,
            revolute_joints[3]: -0.50,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="folded_pose_no_floating")
        for index in range(4):
            ctx.expect_contact(blocks[index], links[index + 1], name=f"folded_pose_joint_{index + 1}_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
