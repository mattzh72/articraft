from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="document_wall_safe")

    steel = model.material("dark_gunmetal", rgba=(0.12, 0.13, 0.14, 1.0))
    edge_steel = model.material("worn_steel_edges", rgba=(0.34, 0.35, 0.34, 1.0))
    interior = model.material("shadowed_interior", rgba=(0.035, 0.038, 0.042, 1.0))
    drawer_mat = model.material("drawer_galvanized", rgba=(0.45, 0.47, 0.48, 1.0))
    brass = model.material("aged_brass", rgba=(0.86, 0.65, 0.30, 1.0))
    white = model.material("painted_white_mark", rgba=(0.92, 0.90, 0.82, 1.0))

    # The safe is dimensioned like a small document wall safe: broad enough for
    # papers and folders, shallow enough to live between wall studs.
    body = model.part("safe_body")
    body.visual(
        Box((0.64, 0.035, 0.46)),
        origin=Origin(xyz=(0.0, 0.1325, 0.0)),
        material=interior,
        name="back_panel",
    )
    body.visual(
        Box((0.04, 0.30, 0.46)),
        origin=Origin(xyz=(-0.30, 0.0, 0.0)),
        material=steel,
        name="left_wall",
    )
    body.visual(
        Box((0.04, 0.30, 0.46)),
        origin=Origin(xyz=(0.30, 0.0, 0.0)),
        material=steel,
        name="right_wall",
    )
    body.visual(
        Box((0.64, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=steel,
        name="top_wall",
    )
    body.visual(
        Box((0.64, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.21)),
        material=steel,
        name="bottom_wall",
    )

    # Four front jamb pieces leave a wide rectangular threshold/opening.
    body.visual(
        Box((0.64, 0.015, 0.06)),
        origin=Origin(xyz=(0.0, -0.1575, 0.20)),
        material=edge_steel,
        name="front_top_jamb",
    )
    body.visual(
        Box((0.64, 0.015, 0.06)),
        origin=Origin(xyz=(0.0, -0.1575, -0.20)),
        material=edge_steel,
        name="front_bottom_jamb",
    )
    body.visual(
        Box((0.06, 0.015, 0.34)),
        origin=Origin(xyz=(-0.29, -0.1575, 0.0)),
        material=edge_steel,
        name="left_jamb",
    )
    body.visual(
        Box((0.06, 0.015, 0.34)),
        origin=Origin(xyz=(0.29, -0.1575, 0.0)),
        material=edge_steel,
        name="right_jamb",
    )
    body.visual(
        Box((0.52, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.172, -0.176)),
        material=edge_steel,
        name="threshold_lip",
    )

    # Fixed runner rails inside the safe.  They are attached to the side walls
    # and support the shallow document drawer.
    body.visual(
        Box((0.065, 0.210, 0.016)),
        origin=Origin(xyz=(-0.255, -0.010, -0.162)),
        material=edge_steel,
        name="inner_runner_0",
    )
    body.visual(
        Box((0.065, 0.210, 0.016)),
        origin=Origin(xyz=(0.255, -0.010, -0.162)),
        material=edge_steel,
        name="inner_runner_1",
    )

    # Exposed left hinge leaf and alternating fixed hinge barrels.
    hinge_x = -0.326
    hinge_y = -0.195
    body.visual(
        Box((0.018, 0.024, 0.36)),
        origin=Origin(xyz=(hinge_x, -0.171, 0.0)),
        material=edge_steel,
        name="hinge_leaf",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.140)),
        material=edge_steel,
        name="hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(hinge_x, hinge_y, -0.140)),
        material=edge_steel,
        name="hinge_barrel_1",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.380),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        material=brass,
        name="hinge_pin",
    )

    door = model.part("door")
    door_w = 0.56
    door_h = 0.38
    door_t = 0.035
    display_yaw = -1.05
    c = math.cos(display_yaw)
    s = math.sin(display_yaw)

    def rotated(local_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z = local_xyz
        return (c * x - s * y, s * x + c * y, z)

    def door_box(
        name: str,
        size: tuple[float, float, float],
        local_xyz: tuple[float, float, float],
        material: Material,
    ) -> None:
        door.visual(
            Box(size),
            origin=Origin(xyz=rotated(local_xyz), rpy=(0.0, 0.0, display_yaw)),
            material=material,
            name=name,
        )

    door_box("door_panel", (door_w - 0.040, door_t, door_h), (door_w / 2.0 + 0.020, 0.0, 0.0), steel)
    door_box(
        "raised_border_top",
        (door_w - 0.055, 0.008, 0.018),
        (door_w / 2.0, -door_t / 2.0 - 0.004, door_h / 2.0 - 0.035),
        edge_steel,
    )
    door_box(
        "raised_border_bottom",
        (door_w - 0.055, 0.008, 0.018),
        (door_w / 2.0, -door_t / 2.0 - 0.004, -door_h / 2.0 + 0.035),
        edge_steel,
    )
    door_box(
        "raised_border_strike",
        (0.018, 0.008, door_h - 0.055),
        (door_w - 0.040, -door_t / 2.0 - 0.004, 0.0),
        edge_steel,
    )
    door_box(
        "raised_border_hinge",
        (0.018, 0.008, door_h - 0.055),
        (0.040, -door_t / 2.0 - 0.004, 0.0),
        edge_steel,
    )
    door_box(
        "stiffener_rib",
        (door_w - 0.110, 0.007, 0.016),
        (door_w / 2.0, -door_t / 2.0 - 0.0035, -0.075),
        edge_steel,
    )
    door.visual(
        Cylinder(radius=0.011, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=edge_steel,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.038, 0.012, 0.100)),
        origin=Origin(xyz=rotated((0.030, 0.0, 0.0)), rpy=(0.0, 0.0, display_yaw)),
        material=edge_steel,
        name="hinge_leaf",
    )

    dial_closed_x = 0.365
    dial_mount_y = -door_t / 2.0 - 0.030
    dial_z = 0.040
    dial_origin = rotated((dial_closed_x, dial_mount_y, dial_z))
    door.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(
            xyz=rotated((dial_closed_x, -door_t / 2.0 - 0.0035, dial_z)),
            rpy=(math.pi / 2.0, 0.0, display_yaw),
        ),
        material=edge_steel,
        name="dial_bearing",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-0.55, upper=1.05),
    )

    dial = model.part("dial")
    safe_dial = KnobGeometry(
        0.105,
        0.030,
        body_style="cylindrical",
        edge_radius=0.0025,
        grip=KnobGrip(style="knurled", count=48, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", angle_deg=0.0),
    )
    dial.visual(
        mesh_from_geometry(safe_dial, "safe_dial"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="dial_cap",
    )
    dial.visual(
        Box((0.008, 0.002, 0.035)),
        origin=Origin(xyz=(0.0, -0.0155, 0.020)),
        material=white,
        name="pointer_mark",
    )
    dial.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=edge_steel,
        name="dial_shaft",
    )

    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=dial_origin, rpy=(0.0, 0.0, display_yaw)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.430, 0.170, 0.012)),
        origin=Origin(xyz=(0.0, 0.085, -0.046)),
        material=drawer_mat,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.012, 0.170, 0.080)),
        origin=Origin(xyz=(-0.214, 0.085, 0.0)),
        material=drawer_mat,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.012, 0.170, 0.080)),
        origin=Origin(xyz=(0.214, 0.085, 0.0)),
        material=drawer_mat,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.430, 0.012, 0.080)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=drawer_mat,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.430, 0.012, 0.080)),
        origin=Origin(xyz=(0.0, 0.164, 0.0)),
        material=drawer_mat,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.018, 0.160, 0.014)),
        origin=Origin(xyz=(-0.218, 0.085, -0.052)),
        material=edge_steel,
        name="runner_0",
    )
    drawer.visual(
        Box((0.018, 0.160, 0.014)),
        origin=Origin(xyz=(0.218, 0.085, -0.052)),
        material=edge_steel,
        name="runner_1",
    )
    # A few papers/folders in the tray make the scale and document-safe purpose
    # legible while remaining supported by the drawer bottom.
    drawer.visual(
        Box((0.330, 0.120, 0.006)),
        origin=Origin(xyz=(0.0, 0.090, -0.037)),
        material=Material("cream_folder", rgba=(0.82, 0.76, 0.55, 1.0)),
        name="document_stack",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.125, -0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.13),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("safe_body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    drawer = object_model.get_part("drawer")
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_drawer")

    ctx.allow_overlap(
        dial,
        door,
        elem_a="dial_shaft",
        elem_b="dial_bearing",
        reason="The dial shaft is intentionally captured inside the shallow bearing boss on the door.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The moving hinge barrel is represented as a solid sleeve captured around the fixed hinge pin.",
    )
    ctx.expect_within(
        body,
        door,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="hinge pin stays centered inside moving hinge barrel",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="z",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.090,
        name="moving hinge barrel is retained on the pin",
    )
    ctx.expect_overlap(
        dial,
        door,
        axes="y",
        elem_a="dial_shaft",
        elem_b="dial_bearing",
        min_overlap=0.003,
        name="dial shaft remains inserted in bearing boss",
    )
    ctx.expect_contact(
        dial,
        door,
        elem_a="dial_shaft",
        elem_b="dial_bearing",
        contact_tol=0.003,
        name="dial shaft bears against the door boss",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        inner_elem="drawer_front",
        outer_elem="left_jamb",
        margin=0.58,
        name="drawer front is inside the rectangular safe opening envelope",
    )
    ctx.expect_contact(
        drawer,
        body,
        elem_a="runner_0",
        elem_b="inner_runner_0",
        contact_tol=0.001,
        name="left drawer runner rests on fixed rail",
    )
    ctx.expect_contact(
        drawer,
        body,
        elem_a="runner_1",
        elem_b="inner_runner_1",
        contact_tol=0.001,
        name="right drawer runner rests on fixed rail",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.13}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="runner_0",
            elem_b="inner_runner_0",
            min_overlap=0.020,
            name="extended drawer keeps left runner engaged",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="runner_1",
            elem_b="inner_runner_1",
            min_overlap=0.020,
            name="extended drawer keeps right runner engaged",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends outward along the front axis",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.10,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    with ctx.pose({door_joint: 1.05}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="front_bottom_jamb",
            negative_elem="door_panel",
            min_gap=0.006,
            max_gap=0.025,
            name="closed door stands just proud of the front jamb",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_panel",
            elem_b="front_bottom_jamb",
            min_overlap=0.015,
            name="closed door spans the lower jamb width",
        )

    return ctx.report()


object_model = build_object_model()
