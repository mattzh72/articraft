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


SIDE_PLATE_THICKNESS = 0.012
LINK_THICKNESS = 0.008
LINK_PITCH = 0.072
LINK_OUTER_RADIUS = 0.014
LINK_WEB_WIDTH = 0.016
LINK_WINDOW_WIDTH = 0.008
JOINT_HOLE_RADIUS = 0.006
PLATFORM_LENGTH = 0.060
PLATFORM_DEPTH = 0.034
PLATFORM_THICKNESS = 0.010

JOINT_LIMIT = 2.35
ROOT_LIMIT = 1.95


def _one_sided_cylinder(radius: float, thickness: float, x_pos: float, z_pos: float, side: int) -> cq.Workplane:
    solid = cq.Workplane("XZ").center(x_pos, z_pos).circle(radius).extrude(thickness)
    if side > 0:
        solid = solid.translate((0.0, thickness, 0.0))
    return solid


def _centered_cut_cylinder(radius: float, depth: float, x_pos: float, z_pos: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_pos, z_pos)
        .circle(radius)
        .extrude(depth)
        .translate((0.0, depth / 2.0, 0.0))
    )


def _one_sided_box(
    size_x: float,
    thickness_y: float,
    size_z: float,
    center_x: float,
    center_z: float,
    side: int,
) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, thickness_y, size_z).translate(
        (center_x, side * thickness_y / 2.0, center_z)
    )


def _link_shape(side: int) -> cq.Workplane:
    body = _one_sided_box(
        LINK_PITCH,
        LINK_THICKNESS,
        LINK_WEB_WIDTH,
        center_x=LINK_PITCH / 2.0,
        center_z=0.0,
        side=side,
    )
    body = body.union(_one_sided_cylinder(LINK_OUTER_RADIUS, LINK_THICKNESS, 0.0, 0.0, side))
    body = body.union(_one_sided_cylinder(LINK_OUTER_RADIUS, LINK_THICKNESS, LINK_PITCH, 0.0, side))

    hole_depth = LINK_THICKNESS + 0.002
    body = body.cut(_centered_cut_cylinder(JOINT_HOLE_RADIUS, hole_depth, 0.0, 0.0))
    body = body.cut(_centered_cut_cylinder(JOINT_HOLE_RADIUS, hole_depth, LINK_PITCH, 0.0))

    window_len = LINK_PITCH - 2.0 * (LINK_OUTER_RADIUS + 0.008)
    if window_len > 0.010:
        window = _one_sided_box(
            window_len,
            LINK_THICKNESS + 0.002,
            LINK_WINDOW_WIDTH,
            center_x=LINK_PITCH / 2.0,
            center_z=0.0,
            side=side,
        )
        window = window.union(
            _one_sided_cylinder(LINK_WINDOW_WIDTH / 2.0, LINK_THICKNESS + 0.002, LINK_PITCH / 2.0 - window_len / 2.0, 0.0, side)
        )
        window = window.union(
            _one_sided_cylinder(LINK_WINDOW_WIDTH / 2.0, LINK_THICKNESS + 0.002, LINK_PITCH / 2.0 + window_len / 2.0, 0.0, side)
        )
        body = body.cut(window)

    return body


def _platform_link_shape(side: int) -> cq.Workplane:
    arm = _one_sided_box(
        LINK_PITCH,
        LINK_THICKNESS,
        LINK_WEB_WIDTH,
        center_x=LINK_PITCH / 2.0,
        center_z=0.0,
        side=side,
    )
    arm = arm.union(_one_sided_cylinder(LINK_OUTER_RADIUS, LINK_THICKNESS, 0.0, 0.0, side))
    arm = arm.cut(
        _centered_cut_cylinder(JOINT_HOLE_RADIUS, LINK_THICKNESS + 0.002, 0.0, 0.0)
    )

    slot = _one_sided_box(
        LINK_PITCH - 2.0 * (LINK_OUTER_RADIUS + 0.010),
        LINK_THICKNESS + 0.002,
        LINK_WINDOW_WIDTH,
        center_x=LINK_PITCH / 2.0,
        center_z=0.0,
        side=side,
    )
    arm = arm.cut(slot)

    platform = _one_sided_box(
        PLATFORM_LENGTH,
        PLATFORM_DEPTH,
        PLATFORM_THICKNESS,
        center_x=LINK_PITCH + PLATFORM_LENGTH / 2.0 - 0.006,
        center_z=0.0,
        side=side,
    )
    nose = _one_sided_cylinder(
        PLATFORM_THICKNESS / 2.0,
        PLATFORM_DEPTH,
        LINK_PITCH + PLATFORM_LENGTH - 0.006,
        0.0,
        side,
    )
    return arm.union(platform).union(nose)


def _side_plate_shape() -> cq.Workplane:
    plate = _one_sided_box(
        0.074,
        SIDE_PLATE_THICKNESS,
        0.200,
        center_x=-0.037,
        center_z=0.000,
        side=-1,
    )
    plate = plate.union(_one_sided_cylinder(0.020, SIDE_PLATE_THICKNESS, 0.0, 0.0, -1))
    plate = plate.union(
        _one_sided_box(
            0.050,
            SIDE_PLATE_THICKNESS,
            0.026,
            center_x=-0.026,
            center_z=-0.087,
            side=-1,
        )
    )
    for z_pos in (-0.055, 0.055):
        plate = plate.cut(_centered_cut_cylinder(0.005, SIDE_PLATE_THICKNESS + 0.004, -0.040, z_pos))
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacking_support_foldout_arm")

    model.material("powder_coat", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("zinc_plate", rgba=(0.74, 0.76, 0.80, 1.0))
    model.material("platform_gray", rgba=(0.70, 0.71, 0.73, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_side_plate_shape(), "side_plate_shell"),
        material="powder_coat",
        name="side_plate_shell",
    )
    side_plate.inertial = Inertial.from_geometry(
        Box((0.080, SIDE_PLATE_THICKNESS, 0.205)),
        mass=1.1,
        origin=Origin(xyz=(-0.032, -SIDE_PLATE_THICKNESS / 2.0, 0.0)),
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_link_shape(1), "link_1_shell"),
        material="zinc_plate",
        name="link_1_shell",
    )
    link_1.inertial = Inertial.from_geometry(
        Box((LINK_PITCH, LINK_THICKNESS, 0.028)),
        mass=0.18,
        origin=Origin(xyz=(LINK_PITCH / 2.0, LINK_THICKNESS / 2.0, 0.0)),
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_link_shape(-1), "link_2_shell"),
        material="zinc_plate",
        name="link_2_shell",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((LINK_PITCH, LINK_THICKNESS, 0.028)),
        mass=0.18,
        origin=Origin(xyz=(LINK_PITCH / 2.0, -LINK_THICKNESS / 2.0, 0.0)),
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(_link_shape(1), "link_3_shell"),
        material="zinc_plate",
        name="link_3_shell",
    )
    link_3.inertial = Inertial.from_geometry(
        Box((LINK_PITCH, LINK_THICKNESS, 0.028)),
        mass=0.18,
        origin=Origin(xyz=(LINK_PITCH / 2.0, LINK_THICKNESS / 2.0, 0.0)),
    )

    link_4 = model.part("link_4")
    link_4.visual(
        mesh_from_cadquery(_platform_link_shape(-1), "link_4_platform_shell"),
        material="platform_gray",
        name="link_4_platform_shell",
    )
    link_4.inertial = Inertial.from_geometry(
        Box((LINK_PITCH + PLATFORM_LENGTH, PLATFORM_DEPTH, 0.030)),
        mass=0.24,
        origin=Origin(
            xyz=(
                (LINK_PITCH + PLATFORM_LENGTH) / 2.0,
                -PLATFORM_DEPTH / 2.0,
                0.0,
            )
        ),
    )

    model.articulation(
        "side_plate_to_link_1",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=ROOT_LIMIT,
        ),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-JOINT_LIMIT,
            upper=JOINT_LIMIT,
        ),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-JOINT_LIMIT,
            upper=JOINT_LIMIT,
        ),
    )
    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=link_4,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-JOINT_LIMIT,
            upper=JOINT_LIMIT,
        ),
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

    side_plate = object_model.get_part("side_plate")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")

    joint_1 = object_model.get_articulation("side_plate_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")
    joint_3 = object_model.get_articulation("link_2_to_link_3")
    joint_4 = object_model.get_articulation("link_3_to_link_4")

    ctx.check(
        "five-part serial assembly present",
        len(object_model.parts) == 5 and len(object_model.articulations) == 4,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )

    with ctx.pose({joint_1: 0.0, joint_2: 0.0, joint_3: 0.0, joint_4: 0.0}):
        ctx.expect_origin_gap(
            link_2,
            link_1,
            axis="x",
            min_gap=LINK_PITCH - 0.001,
            max_gap=LINK_PITCH + 0.001,
            name="link 2 hinge sits one pitch from link 1",
        )
        ctx.expect_origin_gap(
            link_3,
            link_2,
            axis="x",
            min_gap=LINK_PITCH - 0.001,
            max_gap=LINK_PITCH + 0.001,
            name="link 3 hinge sits one pitch from link 2",
        )
        ctx.expect_origin_gap(
            link_4,
            link_3,
            axis="x",
            min_gap=LINK_PITCH - 0.001,
            max_gap=LINK_PITCH + 0.001,
            name="link 4 hinge sits one pitch from link 3",
        )
        ctx.expect_origin_gap(
            link_4,
            side_plate,
            axis="x",
            min_gap=3.0 * LINK_PITCH - 0.001,
            max_gap=3.0 * LINK_PITCH + 0.001,
            name="rest pose reaches full three-link pitch",
        )

    rest_pos = ctx.part_world_position(link_4)
    with ctx.pose(
        {
            joint_1: 1.25,
            joint_2: -1.85,
            joint_3: 1.90,
            joint_4: -1.55,
        }
    ):
        folded_pos = ctx.part_world_position(link_4)
        ctx.check(
            "folded pose retracts the platform toward the side plate",
            rest_pos is not None
            and folded_pos is not None
            and folded_pos[0] < rest_pos[0] - 0.10,
            details=f"rest={rest_pos}, folded={folded_pos}",
        )

    with ctx.pose({joint_1: 1.0, joint_2: 0.0, joint_3: 0.0, joint_4: 0.0}):
        raised_pos = ctx.part_world_position(link_4)
        ctx.check(
            "positive root rotation lifts the serial arm upward",
            rest_pos is not None
            and raised_pos is not None
            and raised_pos[2] > rest_pos[2] + 0.16,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
