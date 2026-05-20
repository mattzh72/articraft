from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.32
BODY_DEPTH = 0.22
BODY_HEIGHT = 0.82
BODY_BOTTOM = 0.055
FRONT_Y = BODY_DEPTH / 2.0
REAR_Y = -BODY_DEPTH / 2.0

DOOR_WIDTH = 0.300
DOOR_HEIGHT = 0.720
DOOR_THICKNESS = 0.026
DOOR_BOTTOM = 0.115

WHEEL_RADIUS = 0.064
WHEEL_WIDTH = 0.038
WHEEL_Y = -0.105
WHEEL_Z = WHEEL_RADIUS + 0.004
WHEEL_X = 0.126

HANDLE_Y = REAR_Y - 0.032
HANDLE_JOINT_Z = BODY_BOTTOM + 0.515
HANDLE_TRAVEL = 0.300


def _toolbox_body_shape() -> cq.Workplane:
    """One continuous hollow shell with rear spine rails and wheel recesses."""
    wall = 0.022
    back_wall = 0.024
    top_wall = 0.035
    bottom_wall = 0.040

    outer = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_BOTTOM + BODY_HEIGHT / 2.0))
    )
    # Modest molded corner radii keep the toolbox from reading as a plain crate.
    try:
        outer = outer.edges("|Z").fillet(0.012)
    except Exception:
        pass

    inner_h = BODY_HEIGHT - top_wall - bottom_wall
    cavity_depth = BODY_DEPTH - back_wall + 0.050
    cavity_center_y = (REAR_Y + back_wall + FRONT_Y + 0.050) / 2.0
    cavity = (
        cq.Workplane("XY")
        .box(BODY_WIDTH - 2.0 * wall, cavity_depth, inner_h)
        .translate((0.0, cavity_center_y, BODY_BOTTOM + bottom_wall + inner_h / 2.0))
    )
    body = outer.cut(cavity)

    # Rectangular side/rear pockets remove the shell where the transport wheels sit.
    for x in (-WHEEL_X, WHEEL_X):
        pocket = (
            cq.Workplane("XY")
            .box(0.080, 0.180, 0.160)
            .translate((x, REAR_Y + 0.020, BODY_BOTTOM + 0.045))
        )
        body = body.cut(pocket)

    # Rear pull-handle spine: two proud rails and small end bridges around the channel.
    rail_h = 0.625
    rail_z = BODY_BOTTOM + 0.385
    for x in (-0.083, 0.083):
        rail = (
            cq.Workplane("XY")
            .box(0.024, 0.024, rail_h)
            .translate((x, REAR_Y - 0.010, rail_z))
        )
        body = body.union(rail)

    for z in (BODY_BOTTOM + 0.105, BODY_BOTTOM + 0.690):
        bridge = (
            cq.Workplane("XY")
            .box(0.190, 0.020, 0.030)
            .translate((0.0, REAR_Y - 0.010, z))
        )
        body = body.union(bridge)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")

    shell_mat = model.material("molded_dark_blue", rgba=(0.06, 0.16, 0.30, 1.0))
    recess_mat = model.material("shadow_black", rgba=(0.015, 0.018, 0.020, 1.0))
    door_mat = model.material("front_panel_blue", rgba=(0.08, 0.22, 0.40, 1.0))
    trim_mat = model.material("black_trim", rgba=(0.025, 0.025, 0.028, 1.0))
    metal_mat = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    rubber_mat = model.material("soft_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_toolbox_body_shape(), "body_shell", tolerance=0.0015),
        material=shell_mat,
        name="body_shell",
    )
    # Dark backing panel makes the rear spine read as a recessed channel.
    body.visual(
        Box((0.122, 0.004, 0.595)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.0125, BODY_BOTTOM + 0.395)),
        material=recess_mat,
        name="rear_channel",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.064),
        origin=Origin(xyz=(-WHEEL_X, WHEEL_Y, WHEEL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="axle_stub_0",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.064),
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y, WHEEL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="axle_stub_1",
    )
    for name, x in (("axle_fork_0", -0.093), ("axle_fork_1", 0.093)):
        body.visual(
            Box((0.025, 0.038, 0.038)),
            origin=Origin(xyz=(x, WHEEL_Y, WHEEL_Z)),
            material=shell_mat,
            name=name,
        )
    body.visual(
        Box((0.190, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.024, BODY_BOTTOM + 0.310)),
        material=trim_mat,
        name="handle_guide_0",
    )
    body.visual(
        Box((0.190, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.024, BODY_BOTTOM + 0.610)),
        material=trim_mat,
        name="handle_guide_1",
    )
    # Main compartment details visible when the front door swings open.
    for i, z in enumerate((BODY_BOTTOM + 0.255, BODY_BOTTOM + 0.455, BODY_BOTTOM + 0.655)):
        body.visual(
            Box((0.282, 0.150, 0.014)),
            origin=Origin(xyz=(0.0, -0.020, z)),
            material=trim_mat,
            name=f"inner_shelf_{i}",
        )
    body.visual(
        Box((0.230, 0.008, 0.600)),
        origin=Origin(xyz=(0.0, REAR_Y + 0.025, BODY_BOTTOM + 0.390)),
        material=recess_mat,
        name="compartment_back",
    )
    # Fixed hinge knuckles and leaves alternate with the door knuckles.
    for i, z in enumerate((DOOR_BOTTOM + 0.095, DOOR_BOTTOM + 0.355, DOOR_BOTTOM + 0.615)):
        body.visual(
            Box((0.022, 0.020, 0.080)),
            origin=Origin(xyz=(-BODY_WIDTH / 2.0 - 0.004, FRONT_Y, z)),
            material=metal_mat,
            name=f"hinge_leaf_{i}",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.085),
            origin=Origin(
                xyz=(-BODY_WIDTH / 2.0 - 0.009, FRONT_Y + 0.011, z),
                rpy=(0.0, 0.0, 0.0),
            ),
            material=metal_mat,
            name=f"body_hinge_{i}",
        )

    front_door = model.part("front_door")
    front_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0 + 0.004, DOOR_HEIGHT / 2.0)),
        material=door_mat,
        name="door_panel",
    )
    # Raised perimeter and recessed central panel treatment on the front face.
    front_door.visual(
        Box((DOOR_WIDTH - 0.035, 0.006, DOOR_HEIGHT - 0.060)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS + 0.003, DOOR_HEIGHT / 2.0)),
        material=trim_mat,
        name="door_recess",
    )
    front_door.visual(
        Box((0.065, 0.010, 0.020)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.055, DOOR_THICKNESS + 0.010, DOOR_HEIGHT * 0.58)),
        material=metal_mat,
        name="door_pull",
    )
    for i, z in enumerate((0.205, 0.505)):
        front_door.visual(
            Cylinder(radius=0.010, length=0.115),
            origin=Origin(xyz=(0.0, DOOR_THICKNESS / 2.0 + 0.004, z)),
            material=metal_mat,
            name=f"door_hinge_{i}",
        )

    pull_handle = model.part("pull_handle")
    # Two upright rods and a rounded cross grip form a single retained telescoping handle.
    pull_handle.visual(
        Cylinder(radius=0.006, length=0.620),
        origin=Origin(xyz=(-0.052, 0.0, 0.000)),
        material=metal_mat,
        name="handle_rod_0",
    )
    pull_handle.visual(
        Cylinder(radius=0.006, length=0.620),
        origin=Origin(xyz=(0.052, 0.0, 0.000)),
        material=metal_mat,
        name="handle_rod_1",
    )
    pull_handle.visual(
        Cylinder(radius=0.012, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.305), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_mat,
        name="handle_grip",
    )
    pull_handle.visual(
        Box((0.118, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.282)),
        material=metal_mat,
        name="handle_bridge",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.044,
            WHEEL_WIDTH,
            rim=WheelRim(inner_radius=0.028, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.015,
                width=0.026,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.022, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.005),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.045,
            tread=TireTread(style="block", depth=0.004, count=16, land_ratio=0.56),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "wheel_tire",
    )
    for name, x in (("wheel_0", -WHEEL_X), ("wheel_1", WHEEL_X)):
        wheel = model.part(name)
        wheel.visual(wheel_mesh, material=metal_mat, name="rim")
        wheel.visual(tire_mesh, material=rubber_mat, name="tire")

        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x, WHEEL_Y, WHEEL_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=12.0),
        )

    model.articulation(
        "body_to_front_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_door,
        origin=Origin(xyz=(-BODY_WIDTH / 2.0 - 0.003, FRONT_Y + 0.002, DOOR_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    model.articulation(
        "body_to_pull_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pull_handle,
        origin=Origin(xyz=(0.0, HANDLE_Y, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=HANDLE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    front_door = object_model.get_part("front_door")
    pull_handle = object_model.get_part("pull_handle")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    door_joint = object_model.get_articulation("body_to_front_door")
    handle_joint = object_model.get_articulation("body_to_pull_handle")
    wheel_joint_0 = object_model.get_articulation("body_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("body_to_wheel_1")

    ctx.allow_overlap(
        body,
        wheel_0,
        elem_a="axle_stub_0",
        elem_b="rim",
        reason="The fixed axle stub is intentionally captured inside the rotating wheel bore.",
    )
    ctx.allow_overlap(
        body,
        wheel_1,
        elem_a="axle_stub_1",
        elem_b="rim",
        reason="The fixed axle stub is intentionally captured inside the rotating wheel bore.",
    )
    for guide_name in ("handle_guide_0", "handle_guide_1"):
        for rod_name in ("handle_rod_0", "handle_rod_1"):
            ctx.allow_overlap(
                body,
                pull_handle,
                elem_a=guide_name,
                elem_b=rod_name,
                reason="The telescoping rods intentionally pass through simplified rear spine guide bushings.",
            )

    ctx.expect_gap(
        front_door,
        body,
        axis="y",
        min_gap=0.001,
        max_gap=0.010,
        positive_elem="door_panel",
        negative_elem="body_shell",
        name="closed door sits just proud of front shell",
    )
    ctx.expect_overlap(
        front_door,
        body,
        axes="xz",
        min_overlap=0.20,
        elem_a="door_panel",
        elem_b="body_shell",
        name="closed door covers the main compartment opening",
    )

    closed_aabb = ctx.part_world_aabb(front_door)
    with ctx.pose({door_joint: 1.35}):
        open_aabb = ctx.part_world_aabb(front_door)
    ctx.check(
        "front door swings outward from vertical hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    ctx.expect_within(
        pull_handle,
        body,
        axes="xy",
        margin=0.025,
        outer_elem="body_shell",
        name="handle nests inside rear spine channel in plan",
    )
    ctx.expect_overlap(
        pull_handle,
        body,
        axes="z",
        min_overlap=0.25,
        elem_b="body_shell",
        name="collapsed handle has retained insertion in spine",
    )
    ctx.expect_overlap(
        body,
        pull_handle,
        axes="xyz",
        min_overlap=0.004,
        elem_a="handle_guide_0",
        elem_b="handle_rod_0",
        name="first lower guide captures the telescoping rod",
    )
    ctx.expect_overlap(
        body,
        pull_handle,
        axes="xyz",
        min_overlap=0.004,
        elem_a="handle_guide_0",
        elem_b="handle_rod_1",
        name="second lower guide captures the telescoping rod",
    )
    ctx.expect_overlap(
        body,
        pull_handle,
        axes="xyz",
        min_overlap=0.004,
        elem_a="handle_guide_1",
        elem_b="handle_rod_0",
        name="first upper guide captures the telescoping rod",
    )
    ctx.expect_overlap(
        body,
        pull_handle,
        axes="xyz",
        min_overlap=0.004,
        elem_a="handle_guide_1",
        elem_b="handle_rod_1",
        name="second upper guide captures the telescoping rod",
    )
    rest_handle_pos = ctx.part_world_position(pull_handle)
    with ctx.pose({handle_joint: HANDLE_TRAVEL}):
        ctx.expect_within(
            pull_handle,
            body,
            axes="xy",
            margin=0.025,
            outer_elem="body_shell",
            name="extended handle remains guided by rear spine",
        )
        ctx.expect_overlap(
            pull_handle,
            body,
            axes="z",
            min_overlap=0.20,
            elem_b="body_shell",
            name="extended handle remains retained in spine",
        )
        extended_handle_pos = ctx.part_world_position(pull_handle)
    ctx.check(
        "pull handle extends upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.25,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    ctx.check(
        "transport wheels use continuous axle joints",
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"{wheel_joint_0.articulation_type=}, {wheel_joint_1.articulation_type=}",
    )
    ctx.expect_overlap(
        body,
        wheel_0,
        axes="x",
        min_overlap=0.020,
        elem_a="axle_stub_0",
        elem_b="rim",
        name="first axle stub remains captured in wheel bore",
    )
    ctx.expect_overlap(
        body,
        wheel_1,
        axes="x",
        min_overlap=0.020,
        elem_a="axle_stub_1",
        elem_b="rim",
        name="second axle stub remains captured in wheel bore",
    )
    ctx.expect_overlap(
        wheel_0,
        body,
        axes="yz",
        min_overlap=0.035,
        elem_a="tire",
        elem_b="body_shell",
        name="first wheel sits recessed in its pocket",
    )
    ctx.expect_overlap(
        wheel_1,
        body,
        axes="yz",
        min_overlap=0.035,
        elem_a="tire",
        elem_b="body_shell",
        name="second wheel sits recessed in its pocket",
    )

    return ctx.report()


object_model = build_object_model()
