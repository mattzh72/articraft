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


PLATE_LENGTH = 0.180
PLATE_WIDTH = 0.160
PLATE_THICKNESS = 0.014
SHOULDER_HEIGHT = 0.068

LINK_WIDTHS = (0.082, 0.076, 0.070, 0.064)
LINK_DEPTHS = (0.050, 0.045, 0.038, 0.032)
LINK_LENGTHS = (0.138, 0.128, 0.118, 0.108)


def _y_axis_cylinder(length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").cylinder(length, radius).rotate((0, 0, 0), (1, 0, 0), 90)


def _base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        PLATE_LENGTH,
        PLATE_WIDTH,
        PLATE_THICKNESS,
        centered=(False, True, False),
    ).translate((-PLATE_LENGTH, 0.0, 0.0))

    pedestal = cq.Workplane("XY").box(
        0.060,
        0.092,
        SHOULDER_HEIGHT - PLATE_THICKNESS,
        centered=(False, True, False),
    ).translate((-0.060, 0.0, PLATE_THICKNESS))

    shoulder_bridge = cq.Workplane("XY").box(
        0.024,
        0.070,
        0.024,
        centered=(False, True, True),
    ).translate((-0.024, 0.0, SHOULDER_HEIGHT))

    left_cheek = cq.Workplane("XY").box(
        0.010,
        0.018,
        0.032,
        centered=(False, True, True),
    ).translate((-0.010, 0.026, SHOULDER_HEIGHT))
    right_cheek = cq.Workplane("XY").box(
        0.010,
        0.018,
        0.032,
        centered=(False, True, True),
    ).translate((-0.010, -0.026, SHOULDER_HEIGHT))

    body = plate.union(pedestal).union(shoulder_bridge).union(left_cheek).union(right_cheek)

    for x_pos in (-0.145, -0.040):
        for y_pos in (-0.052, 0.052):
            body = body.cut(
                cq.Workplane("XY").cylinder(PLATE_THICKNESS + 0.004, 0.007)
                .translate((x_pos, y_pos, -0.002))
            )

    return body.edges("|Z").fillet(0.004)


def _link_shape(
    *,
    length: float,
    width: float,
    depth: float,
    rail_width: float,
    end_block: float,
    rung_length: float,
    rung_depth: float,
    pin_radius: float,
) -> cq.Workplane:
    left_rail = cq.Workplane("XY").box(
        length - 2.0 * end_block,
        rail_width,
        depth,
        centered=(False, True, True),
    ).translate((end_block, width / 2.0 - rail_width / 2.0, 0.0))
    right_rail = cq.Workplane("XY").box(
        length - 2.0 * end_block,
        rail_width,
        depth,
        centered=(False, True, True),
    ).translate((end_block, -(width / 2.0 - rail_width / 2.0), 0.0))
    root_block = cq.Workplane("XY").box(
        end_block,
        width,
        depth,
        centered=(False, True, True),
    )
    tip_block = cq.Workplane("XY").box(
        end_block,
        width,
        depth,
        centered=(False, True, True),
    ).translate((length - end_block, 0.0, 0.0))
    center_rung = cq.Workplane("XY").box(
        rung_length,
        width - 2.0 * rail_width,
        rung_depth,
        centered=(True, True, True),
    ).translate((length * 0.54, 0.0, 0.0))

    link = (
        root_block.union(tip_block)
        .union(left_rail)
        .union(right_rail)
        .union(center_rung)
    )

    root_hole = _y_axis_cylinder(width + 0.010, pin_radius).translate((0.008, 0.0, 0.0))
    tip_hole = _y_axis_cylinder(width + 0.010, pin_radius).translate((length - 0.008, 0.0, 0.0))
    link = link.cut(root_hole).cut(tip_hole)

    return link.edges("|Y").fillet(min(0.0035, depth * 0.10))


def _platform_bracket_shape() -> cq.Workplane:
    back_wall = cq.Workplane("XY").box(
        0.010,
        0.054,
        0.038,
        centered=(False, True, False),
    )
    platform = cq.Workplane("XY").box(
        0.060,
        0.054,
        0.006,
        centered=(False, True, False),
    ).translate((0.0, 0.0, 0.020))
    front_lip = cq.Workplane("XY").box(
        0.006,
        0.054,
        0.012,
        centered=(False, True, False),
    ).translate((0.054, 0.0, 0.020))
    left_rib = cq.Workplane("XY").box(
        0.028,
        0.006,
        0.018,
        centered=(False, True, False),
    ).translate((0.010, 0.020, 0.0))
    right_rib = cq.Workplane("XY").box(
        0.028,
        0.006,
        0.018,
        centered=(False, True, False),
    ).translate((0.010, -0.020, 0.0))

    bracket = back_wall.union(platform).union(front_lip).union(left_rib).union(right_rib)
    for x_pos in (0.022, 0.042):
        bracket = bracket.cut(
            cq.Workplane("XY").cylinder(0.010, 0.0045).translate((x_pos, 0.0, 0.018))
        )
    return bracket.edges("|Z").fillet(0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_ladder_arm")

    model.material("powder_coat", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("inner_link_finish", rgba=(0.35, 0.38, 0.42, 1.0))
    model.material("outer_link_finish", rgba=(0.56, 0.59, 0.63, 1.0))
    model.material("bracket_finish", rgba=(0.73, 0.75, 0.78, 1.0))

    base = model.part("base_plate")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_plate"),
        material="powder_coat",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, SHOULDER_HEIGHT)),
        mass=3.6,
        origin=Origin(xyz=(-PLATE_LENGTH / 2.0, 0.0, SHOULDER_HEIGHT / 2.0)),
    )

    links = []
    for idx, (length, width, depth) in enumerate(zip(LINK_LENGTHS, LINK_WIDTHS, LINK_DEPTHS), start=1):
        link = model.part(f"link_{idx}")
        rail_width = 0.012 if idx < 3 else 0.010
        end_block = 0.018 if idx < 3 else 0.016
        rung_length = 0.020 if idx < 3 else 0.018
        rung_depth = depth * 0.48
        pin_radius = 0.008 if idx < 3 else 0.0065
        link.visual(
            mesh_from_cadquery(
                _link_shape(
                    length=length,
                    width=width,
                    depth=depth,
                    rail_width=rail_width,
                    end_block=end_block,
                    rung_length=rung_length,
                    rung_depth=rung_depth,
                    pin_radius=pin_radius,
                ),
                f"link_{idx}_frame",
            ),
            material="inner_link_finish" if idx < 3 else "outer_link_finish",
            name=f"link_{idx}_frame",
        )
        link.inertial = Inertial.from_geometry(
            Box((length, width, depth)),
            mass=(0.72, 0.62, 0.48, 0.38)[idx - 1],
            origin=Origin(xyz=(length / 2.0, 0.0, 0.0)),
        )
        links.append(link)

    bracket = model.part("platform_bracket")
    bracket.visual(
        mesh_from_cadquery(_platform_bracket_shape(), "platform_bracket"),
        material="bracket_finish",
        name="platform_shell",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.060, 0.054, 0.038)),
        mass=0.22,
        origin=Origin(xyz=(0.030, 0.0, 0.019)),
    )

    joint_limits = (
        (-0.15, 1.25),
        (-0.25, 1.35),
        (-0.35, 1.30),
        (-0.45, 1.20),
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=joint_limits[0][0],
            upper=joint_limits[0][1],
            effort=24.0,
            velocity=1.4,
        ),
    )

    for idx in range(1, 4):
        model.articulation(
            f"link_{idx}_to_link_{idx + 1}",
            ArticulationType.REVOLUTE,
            parent=links[idx - 1],
            child=links[idx],
            origin=Origin(xyz=(LINK_LENGTHS[idx - 1], 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                lower=joint_limits[idx][0],
                upper=joint_limits[idx][1],
                effort=18.0 - 2.0 * idx,
                velocity=1.8,
            ),
        )

    model.articulation(
        "link_4_to_platform",
        ArticulationType.FIXED,
        parent=links[3],
        child=bracket,
        origin=Origin(xyz=(LINK_LENGTHS[3], 0.0, -0.010)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base_plate")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")
    bracket = object_model.get_part("platform_bracket")

    shoulder = object_model.get_articulation("base_to_link_1")
    elbow_1 = object_model.get_articulation("link_1_to_link_2")
    elbow_2 = object_model.get_articulation("link_2_to_link_3")
    wrist = object_model.get_articulation("link_3_to_link_4")

    serial_joints = (shoulder, elbow_1, elbow_2, wrist)
    ctx.check(
        "arm has four parallel revolute joints",
        len(serial_joints) == 4
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in serial_joints)
        and all(tuple(joint.axis) == (0.0, -1.0, 0.0) for joint in serial_joints),
        details=str(
            [
                (joint.name, str(joint.articulation_type), tuple(joint.axis))
                for joint in serial_joints
            ]
        ),
    )

    ctx.expect_overlap(
        base,
        link_1,
        axes="yz",
        min_overlap=0.025,
        name="base and first link share the shoulder footprint",
    )
    ctx.expect_gap(
        link_1,
        base,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0005,
        name="first link seats at the shoulder plane",
    )

    for parent, child, pair_name in (
        (link_1, link_2, "first elbow"),
        (link_2, link_3, "second elbow"),
        (link_3, link_4, "third elbow"),
    ):
        ctx.expect_overlap(
            parent,
            child,
            axes="yz",
            min_overlap=0.020,
            name=f"{pair_name} keeps aligned hinge footprints",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="x",
            max_gap=0.003,
            max_penetration=0.0005,
            name=f"{pair_name} closes at the hinge plane",
        )

    rest_pos = ctx.part_world_position(bracket)
    with ctx.pose(
        {
            shoulder: 0.55,
            elbow_1: 0.42,
            elbow_2: 0.30,
            wrist: 0.22,
        }
    ):
        deployed_pos = ctx.part_world_position(bracket)

    ctx.check(
        "platform bracket lifts in deployed pose",
        rest_pos is not None
        and deployed_pos is not None
        and deployed_pos[2] > rest_pos[2] + 0.12
        and deployed_pos[0] > 0.18,
        details=f"rest={rest_pos}, deployed={deployed_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
