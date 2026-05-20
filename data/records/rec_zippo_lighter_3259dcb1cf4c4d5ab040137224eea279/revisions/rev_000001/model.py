from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


CASE_W = 0.036
CASE_D = 0.013
CASE_H = 0.041
CASE_WALL = 0.0013
LID_H = 0.019
HINGE_R = 0.00145
HINGE_X = -CASE_W / 2.0 - 0.0010
OPEN_ANGLE = 1.55


def _case_shell() -> cq.Workplane:
    """Straight-sided open brass case with a bottom and a thin top rim."""
    outer = cq.Workplane("XY").box(CASE_W, CASE_D, CASE_H, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(
            CASE_W - 2.0 * CASE_WALL,
            CASE_D - 2.0 * CASE_WALL,
            CASE_H + CASE_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CASE_WALL))
    )
    shell = outer.cut(inner)
    # Small bevels/fillets read like stamped brass without making the case bulged.
    try:
        shell = shell.edges("|Z").fillet(0.0012)
        shell = shell.edges(">Z").fillet(0.00045)
    except Exception:
        pass
    return shell


def _lid_shell() -> cq.Workplane:
    """Open-bottom cap, hollow so the closed lid clears the chimney insert."""
    wall = 0.00115
    outer = cq.Workplane("XY").box(CASE_W, CASE_D, LID_H, centered=(True, True, True))
    cavity = (
        cq.Workplane("XY")
        .box(
            CASE_W - 2.0 * wall,
            CASE_D - 2.0 * wall,
            LID_H - wall + 0.003,
            centered=(True, True, True),
        )
        .translate((0.0, 0.0, -wall / 2.0 - 0.0015))
    )
    shell = outer.cut(cavity)
    try:
        shell = shell.edges("|Z").fillet(0.0010)
        shell = shell.edges(">Z").fillet(0.00045)
    except Exception:
        pass
    return shell


def _hole_cylinder_y(radius: float, length: float, x: float, y: float, z: float) -> cq.Workplane:
    """A centered cylinder along Y, useful for cutting chimney vent holes."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x, y, z))
    )


def _chimney_insert() -> cq.Workplane:
    """Seated insert with perforated chimney, fork support, and cam brackets."""
    steel = cq.Workplane("XY")
    # The lower plug drops into the case; the flange sits on the top rim.
    plug = cq.Workplane("XY").box(0.029, 0.0088, 0.020, centered=(True, True, True)).translate(
        (0.0, 0.0, CASE_H - 0.0095)
    )
    flange = cq.Workplane("XY").box(0.034, 0.0115, 0.0020, centered=(True, True, True)).translate(
        (0.0, 0.0, CASE_H + 0.0010)
    )

    chimney_w = 0.018
    chimney_d = 0.010
    chimney_h = 0.018
    wall = 0.0008
    zc = CASE_H + 0.0105
    front = cq.Workplane("XY").box(chimney_w, wall, chimney_h, centered=(True, True, True)).translate(
        (0.0, -chimney_d / 2.0, zc)
    )
    back = cq.Workplane("XY").box(chimney_w, wall, chimney_h, centered=(True, True, True)).translate(
        (0.0, chimney_d / 2.0, zc)
    )
    side_a = cq.Workplane("XY").box(wall, chimney_d, chimney_h, centered=(True, True, True)).translate(
        (-chimney_w / 2.0, 0.0, zc)
    )
    side_b = cq.Workplane("XY").box(wall, chimney_d, chimney_h, centered=(True, True, True)).translate(
        (chimney_w / 2.0, 0.0, zc)
    )

    vented = front.union(back).union(side_a).union(side_b)
    for y in (-chimney_d / 2.0, chimney_d / 2.0):
        for z in (CASE_H + 0.0050, CASE_H + 0.0090, CASE_H + 0.0130, CASE_H + 0.0170):
            for x in (-0.0055, 0.0, 0.0055):
                vented = vented.cut(_hole_cylinder_y(0.00085, 0.0040, x, y, z))

    # Flint-wheel fork cheeks and a little cam-pivot bracket, both tied into the insert.
    fork_z = CASE_H + 0.0155
    fork_x = 0.0108
    cheek_a = cq.Workplane("XY").box(0.0012, 0.0010, 0.010, centered=(True, True, True)).translate(
        (fork_x, -0.0044, fork_z)
    )
    cheek_b = cq.Workplane("XY").box(0.0012, 0.0010, 0.010, centered=(True, True, True)).translate(
        (fork_x, 0.0044, fork_z)
    )
    fork_bridge = cq.Workplane("XY").box(0.0040, 0.0098, 0.0012, centered=(True, True, True)).translate(
        (0.0100, 0.0, fork_z - 0.0050)
    )

    cam_x = -0.0130
    cam_z = CASE_H + 0.0045
    cam_post = cq.Workplane("XY").box(0.0011, 0.0088, 0.0085, centered=(True, True, True)).translate(
        (cam_x - 0.0010, 0.0, cam_z)
    )
    cam_cheek_a = cq.Workplane("XY").box(0.0024, 0.0008, 0.0045, centered=(True, True, True)).translate(
        (cam_x, -0.0037, cam_z)
    )
    cam_cheek_b = cq.Workplane("XY").box(0.0024, 0.0008, 0.0045, centered=(True, True, True)).translate(
        (cam_x, 0.0037, cam_z)
    )

    wick_tube = cq.Workplane("XY").circle(0.0014).extrude(0.010).translate((-0.0030, 0.0, CASE_H + 0.0015))
    return (
        steel.union(plug)
        .union(flange)
        .union(vented)
        .union(cheek_a)
        .union(cheek_b)
        .union(fork_bridge)
        .union(cam_post)
        .union(cam_cheek_a)
        .union(cam_cheek_b)
        .union(wick_tube)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brass_side_hinged_lighter")

    model.material("brass", rgba=(0.78, 0.57, 0.22, 1.0))
    model.material("dark_brass", rgba=(0.47, 0.33, 0.11, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    model.material("dark_steel", rgba=(0.14, 0.14, 0.13, 1.0))
    model.material("charred_wick", rgba=(0.025, 0.020, 0.016, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_case_shell(), "straight_brass_case", tolerance=0.00025),
        material="brass",
        name="case_shell",
    )
    # The side hinge barrel is embedded a hair into the case wall, as on a soldered case.
    case.visual(
        Cylinder(radius=HINGE_R, length=0.043),
        origin=Origin(xyz=(HINGE_X, 0.0, 0.0215)),
        material="brass",
        name="case_hinge_barrel",
    )

    chimney = model.part("chimney_insert")
    chimney.visual(
        mesh_from_cadquery(_chimney_insert(), "vented_chimney_insert", tolerance=0.00018),
        material="brushed_steel",
        name="vented_chimney",
    )
    chimney.visual(
        Cylinder(radius=0.00085, length=0.006),
        origin=Origin(xyz=(-0.0030, 0.0, CASE_H + 0.0140)),
        material="charred_wick",
        name="wick_tip",
    )

    lid = model.part("lid")
    # Author the child part in an open display pose at q=0.  At q=-OPEN_ANGLE it
    # returns to the straight-sided closed cap position over the chimney.
    closed_center = (CASE_W / 2.0 + 0.0010, 0.0, LID_H / 2.0)
    open_center = (
        closed_center[0] * cos(OPEN_ANGLE),
        closed_center[0] * sin(OPEN_ANGLE),
        closed_center[2],
    )
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "hollow_brass_lid", tolerance=0.00022),
        origin=Origin(xyz=open_center, rpy=(0.0, 0.0, OPEN_ANGLE)),
        material="brass",
        name="hollow_lid_shell",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material="brass",
        name="lid_hinge_barrel",
    )

    wheel = model.part("striker_wheel")
    wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.0076,
                0.0048,
                body_style="cylindrical",
                grip=KnobGrip(style="knurled", count=24, depth=0.00055, helix_angle_deg=18.0),
                edge_radius=0.00025,
            ),
            "knurled_striker_wheel",
        ),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="knurled_wheel",
    )
    wheel.visual(
        Cylinder(radius=0.00055, length=0.0088),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="wheel_axle",
    )

    cam = model.part("cam_lever")
    cam.visual(
        Cylinder(radius=0.00105, length=0.0066),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="cam_pivot_pin",
    )
    cam.visual(
        Box((0.0022, 0.0010, 0.0105)),
        origin=Origin(xyz=(0.0009, 0.0, 0.0054), rpy=(0.0, -0.16, 0.0)),
        material="dark_steel",
        name="cam_blade",
    )
    cam.visual(
        Cylinder(radius=0.00125, length=0.0012),
        origin=Origin(xyz=(0.0023, 0.0, 0.0105), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="cam_roller_nose",
    )

    model.articulation(
        "case_to_chimney",
        ArticulationType.FIXED,
        parent=case,
        child=chimney,
        origin=Origin(),
    )
    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, CASE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0, lower=-OPEN_ANGLE, upper=0.0),
    )
    model.articulation(
        "chimney_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=chimney,
        child=wheel,
        origin=Origin(xyz=(0.0108, 0.0, CASE_H + 0.0162)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=20.0),
    )
    model.articulation(
        "chimney_to_cam",
        ArticulationType.REVOLUTE,
        parent=chimney,
        child=cam,
        origin=Origin(xyz=(-0.0130, 0.0, CASE_H + 0.0045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=5.0, lower=-0.05, upper=0.42),
        mimic=Mimic(joint="case_to_lid", multiplier=0.25, offset=0.40),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    case = object_model.get_part("case")
    chimney = object_model.get_part("chimney_insert")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("striker_wheel")
    cam = object_model.get_part("cam_lever")
    lid_joint = object_model.get_articulation("case_to_lid")
    wheel_joint = object_model.get_articulation("chimney_to_wheel")
    cam_joint = object_model.get_articulation("chimney_to_cam")

    ctx.allow_overlap(
        case,
        chimney,
        elem_a="case_shell",
        elem_b="vented_chimney",
        reason=(
            "The lower insert is intentionally seated down inside the open brass case; "
            "the mesh collision proxy treats the hollow case cavity as solid."
        ),
    )

    ctx.expect_within(
        chimney,
        case,
        axes="xy",
        margin=0.001,
        name="chimney insert stays within the case footprint",
    )
    ctx.expect_overlap(
        chimney,
        case,
        axes="z",
        min_overlap=0.010,
        name="chimney insert remains seated down in the case opening",
    )
    ctx.expect_origin_distance(
        cam,
        chimney,
        axes="xy",
        max_dist=0.014,
        name="cam lever pivot is supported beside the chimney and hinge",
    )
    ctx.expect_origin_distance(
        wheel,
        chimney,
        axes="xy",
        max_dist=0.013,
        name="striker wheel rides on the transverse chimney fork",
    )
    ctx.check(
        "striker wheel is continuous about the transverse axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.check(
        "cam lever is a short revolute follower beside the hinge",
        cam_joint.articulation_type == ArticulationType.REVOLUTE and tuple(cam_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={cam_joint.articulation_type}, axis={cam_joint.axis}",
    )

    closed_center_xy = None
    open_center_xy = None
    cam_closed_tip_z = None
    cam_open_tip_z = None
    with ctx.pose({lid_joint: -OPEN_ANGLE}):
        closed_aabb = ctx.part_world_aabb(lid)
        cam_closed_aabb = ctx.part_element_world_aabb(cam, elem="cam_blade")
        if closed_aabb is not None:
            closed_center_xy = (
                (closed_aabb[0][0] + closed_aabb[1][0]) / 2.0,
                (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0,
            )
        if cam_closed_aabb is not None:
            cam_closed_tip_z = cam_closed_aabb[1][2]
        ctx.expect_within(
            chimney,
            lid,
            axes="xy",
            margin=0.006,
            name="closed hollow lid surrounds the chimney without losing seating",
        )
    with ctx.pose({lid_joint: 0.0}):
        open_aabb = ctx.part_world_aabb(lid)
        cam_open_aabb = ctx.part_element_world_aabb(cam, elem="cam_blade")
        if open_aabb is not None:
            open_center_xy = (
                (open_aabb[0][0] + open_aabb[1][0]) / 2.0,
                (open_aabb[0][1] + open_aabb[1][1]) / 2.0,
            )
        if cam_open_aabb is not None:
            cam_open_tip_z = cam_open_aabb[1][2]
    ctx.check(
        "side hinge swings lid outward from the closed cap position",
        closed_center_xy is not None
        and open_center_xy is not None
        and (open_center_xy[1] - closed_center_xy[1]) > 0.014,
        details=f"closed={closed_center_xy}, open={open_center_xy}",
    )
    ctx.check(
        "cam lever follows the lid-opening motion on its pivot",
        cam_closed_tip_z is not None and cam_open_tip_z is not None and abs(cam_open_tip_z - cam_closed_tip_z) > 0.0003,
        details=f"closed_tip_z={cam_closed_tip_z}, open_tip_z={cam_open_tip_z}",
    )

    return ctx.report()


object_model = build_object_model()
